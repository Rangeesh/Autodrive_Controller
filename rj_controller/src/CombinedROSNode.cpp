//# ====================================================================
//$                          Code Information                           
//- Name:               CombinedROSNode.cpp
//- Description:        
//- Language:           C++
//- Custom Header:      LateralFeedforward.h | Lateralfeedback.h | CombinedROSNode.h | Longitudinal.h
//- Created by:         Samir
//- Last Modified:      Aug 2 2019
//- Modified by:        Rangeesh V
//# =====================================================================
//$                        Parameters Calculated
//- 
//# =====================================================================


//@ Header Files

#include <iostream>
#include <string>
#include <math.h>
#include <fstream>

#include "CombinedNodeException.h"
#include "CarConstants.h"
#include "LateralFeedforward.h"
#include "LateralFeedback.h"
#include "CombinedROSNode.h"
#include "Longitudinal.h"
#include "signal.h"
#include "Filter.h"


using namespace std;
using namespace Eigen;
using namespace std_msgs;


//@ Global Variable

Status CombinedROSNode::status = Running;

//@ Default Constructor : Initialising All Values

CombinedROSNode::CombinedROSNode() : loop_rate(80), speed(0), heading(0), angular_velocity(0), highest_gains_speed(0) {}


//@ Capturing Ctrl-C Signal

void CombinedROSNode::sigint_handler(int s) {
    status = Stopped;
    throw CombinedNodeException("Captured control-c signal");
}

//@ Initializing all ROS Publishers and Subscribers


void CombinedROSNode::Initialize(int argc, char **argv)
{

    //= Publisher for Testing
    pub_car_data = node.advertise<bolt_msgs::CarData>("CarData",1);

    //= Publishers to CAN Bus
    pub_steering_angle = node.advertise<std_msgs::Int64>("can_steering_auto", 1);
    pub_throttle = node.advertise<std_msgs::Int64>("can_throttle_auto", 1);
    pub_braking = node.advertise<std_msgs::Int64>("can_brake_auto",1);

    // Lateral publishers
    
    // pub_steering_angle = node.advertise<bolt_msgs::SteeringCmd>("autodrive_sim/input/steering_angle", 1);
    pub_simulation_status = node.advertise<std_msgs::String>("autodrive_sim/input/status", 1);

    // Lateral publishers in order to analyze data after testing
    pub_error_lateral = node.advertise<std_msgs::Float32>("error_lateral", 1);
    pub_error_heading = node.advertise<std_msgs::Float32>("error_heading", 1);
    pub_error_angular_velocity = node.advertise<std_msgs::Float32>("error_angular_velocity", 1);
    pub_feedforward_angle = node.advertise<std_msgs::Float32>("feedforward_steering_angle", 1);
    pub_feedback_angle = node.advertise<std_msgs::Float32>("feedback_steering_angle", 1);

    // Longitudinal publishers
    // last input is 5. Maybe 1?
    
    pub_v_desired = node.advertise<std_msgs::Float32>("v_desired",1);
    pub_v_current = node.advertise<std_msgs::Float32>("v_current",1);
    pub_accel = node.advertise<std_msgs::Float32>("accel",1);

    // Lateral subscriptions
    sub_heading = node.subscribe("autodrive_sim/output/heading", 1, &CombinedROSNode::headingCallback, this);
    sub_angular_velocity = node.subscribe("autodrive_sim/output/angular_velocity", 1, &CombinedROSNode::angularVelocityCallback, this);
    sub_waypoints = node.subscribe("autodrive_sim/output/waypoints", 1,&CombinedROSNode::waypointsCallback, this);
    sub_position = node.subscribe("autodrive_sim/output/position", 1, &CombinedROSNode::positionCallback, this);
    sub_simulation_status = node.subscribe("autodrive_sim/output/status", 1, &CombinedROSNode::simulationStatusCallback, this);

    // Longitudinal subscriptions
    sub_imu = node.subscribe("/vehicle/imu/data_raw",1, &CombinedROSNode::imuCallback,this);
    sub_desired_velocity = node.subscribe("/desired_velocity",1,&CombinedROSNode::desired_velocityCallback,this);
    sub_steering = node.subscribe("/autodrive_sim/output/steering_report",1,&CombinedROSNode::steeringCallback,this);

    // Shared subscriptions
    sub_speed = node.subscribe("/odom", 1, &CombinedROSNode::speedCallback, this);
    //sub_speed = node.subscribe("autodrive_sim/output/speed", 1, &CombinedROSNode::speedCallback, this);

    //$ Setup Control-C handler

    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = CombinedROSNode::sigint_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);

    //$ Declaring a new Longitudinal Class Variable
    longitudinal = new Longitudinal();
    desired_velocity =0.0;
}

//@ Simulation State Callback

void CombinedROSNode::simulationStatusCallback(const std_msgs::String::ConstPtr& msg){
	string message = msg->data;
    if (message == "stop") {
        status = Stopped;
        cout << "Received stop status message from simulation" << endl;
    }
}

//@ Heading Callback

void CombinedROSNode::headingCallback(const std_msgs::Float32::ConstPtr& msg){
	heading = msg->data;
}

//@ Angular Velocity Callback

void CombinedROSNode::angularVelocityCallback(const std_msgs::Float32::ConstPtr& msg){
    angular_velocity = msg->data;
}

//@ Position Callback

void CombinedROSNode::positionCallback(const std_msgs::Float32MultiArray::ConstPtr& msg){
	for(std::vector<float>::const_iterator iterator = msg->data.begin(); iterator != msg->data.end(); iterator++){
        float x = *iterator;
        iterator++;
        float y = *iterator;
        position = Point(x, y);
	}
}


//@ Waypoints Callback

void CombinedROSNode::waypointsCallback(const std_msgs::Float32MultiArray::ConstPtr& msg){
    waypoints.clear();
	for(std::vector<float>::const_iterator iterator = msg->data.begin(); iterator != msg->data.end(); iterator++){
        float x = *iterator;
        iterator++;
        float y = *iterator;
        waypoints.push_back(Point(x, y));
	}
}

//@ Steering Callback

void CombinedROSNode::steeringCallback(const bolt_msgs::SteeringReport& msg)
{
    steering = msg.steering_wheel_angle;
}

//@ IMU Callback

void CombinedROSNode::imuCallback(const sensor_msgs::Imu& msg)
{
    imu = msg;
}

//@ Desired Velocity Callback

void CombinedROSNode::desired_velocityCallback(const std_msgs::Float32& msg)
{
    //RJdesired_velocity=msg.data;
    desired_velocity=3;
}

//@ Gains Callback


void CombinedROSNode::gainsCallback(combined_controller::long_gainsConfig &config, uint32_t level)
{
    Kp = config.speed_kp;
    Ki = config.accel_kp;
    Kd = config.accel_ki;
    //desired_velocity = config.control_v;
    tau = config.accel_tau;
}

//@ Speed Callback

void CombinedROSNode::speedCallback(const nav_msgs::Odometry& msg){
    //need add header to check if it is besing updated
	float x = msg.twist.twist.linear.x;
    float y = msg.twist.twist.linear.y;
    speed = sqrt(x*x +y*y);
}

//@ The function runs until more than 2 waypoints are received

void CombinedROSNode::wait_for_waypoints() {
    cout << "Ready and waiting on waypoints to be sent..." << endl;
    while (waypoints.size() < 3) {
        ros::spinOnce();
        loop_rate.sleep();

        // Send ready message to simulation
        std_msgs::String status_msg;
        status_msg.data = "combined_controller: ready";
        pub_simulation_status.publish(status_msg);
        continue;
    }
}


//@ Inserts the Gain Values into gains_map

void CombinedROSNode::getGains(const char* filename){
    float speed, k_lat, k_head, k_o;
    string input;
    ifstream file_input;
    bool error;


    cout <<filename << endl;
    file_input.open(filename);
    error=false;

    while (true)
    {
        getline(file_input, input);
        if (!file_input) break; //check for eof

        istringstream buffer(input);
        buffer >> speed >> k_lat >> k_head >> k_o;

        //check for non numerical input
        //and less/more input than needed
        if (!buffer || !buffer.eof())
        {
            error=true;
            break;
        }
        LateralGains g(k_lat, k_head, k_o);
        gains_map.insert(pair<float, LateralGains>(speed, g));
        if (highest_gains_speed < speed)
            highest_gains_speed = speed;
    }

    if (error)
        cout << "file is corrupted..." << endl;
    cout << "Highest speed: " << highest_gains_speed << endl;



}


//@ Update Loop : Publishes all Values

void CombinedROSNode::UpdateLoop()
{
    // wait for other inputs maybe
    wait_for_waypoints();

    // Set up input filters
    Filter speed_filter;
    Filter angular_velocity_filter;
    Filter heading_filter;

    // Set up output filters
    Filter feedforward_angle_filter;
    Filter feedback_angle_filter;
    Filter steering_filter;

    //Established previous steering angle to be 0
    float previous_steering_cmd = 0.0f;

    // Begin the main loop
    cout << "Starting Controller" << endl;
    while(ros::ok()){ //&& status == Running) {
        cout<<"fowifwfmm \n\ re \\rg \wg\ wg\m\n \n \n gfeg";
        // Calculates a filtered value for all of the single value subscribed
        speed_filter.single_value_data(speed, 25, 0.1);
        angular_velocity_filter.single_value_data(angular_velocity, 25, 0.1);

        float filtered_speed = speed_filter.get_filtered_value();
        cout<<"Filtered SPeed:"<<filtered_speed;
        float filtered_angular_velocity = angular_velocity_filter.get_filtered_value();

        // Lateral Control
        LateralFeedforward feedforward(position, filtered_speed, waypoints);

        // Filter feedforward angle
        float raw_feedforward_angle = feedforward.get_feedforward_angle();
        feedforward_angle_filter.single_value_data(raw_feedforward_angle, 40, 0.1);
        float filtered_feedforward = feedforward_angle_filter.get_filtered_value();

        float rounded_speed = round(speed);
        if (rounded_speed <= 0)
            rounded_speed = 1;

        if (rounded_speed > highest_gains_speed)
            rounded_speed = highest_gains_speed;

        cout << "speed: " << speed << endl;

        //THE FOLLOWING CODE NEEDS TO BE TESTED OUT

        //LateralGains g = gains_map[rounded_speed];
        K_lat = 0.2;
        K_head = 0.5;
        K_omega = 0.1;
        LateralGains g(K_lat,K_head,K_omega);
        cout << endl << g << endl<<"Rangeesh";

        LateralFeedback feedback(&feedforward, position, heading, filtered_angular_velocity, filtered_speed, g.k_lat, g.k_head, g.k_o);
        // Filter feedback angle
        float raw_feedback_angle = feedback.get_feedback_angle();
        feedback_angle_filter.single_value_data(raw_feedback_angle, 40, 0.1);
        float filtered_feedback = feedback_angle_filter.get_filtered_value();

        // Calculat total steering angle - steering gradient (Relation between Steering angle and Tire Angle) * Radians to Degrees Conversion
        float steering_cmd = 15.4 * 57.2958 * (filtered_feedforward + filtered_feedback);

        // normalize angle
        if (steering_cmd > 502){
            steering_cmd = 502;
        }
        else if (steering_cmd < -510){
            steering_cmd = -510;
        }

        // Filter total steering angle
        steering_filter.single_value_data(steering_cmd, 40, 0.1);
        steering_cmd = steering_filter.get_filtered_value();
        steering_cmd = round(steering_cmd);

        // To smooth out the car
        if (abs(previous_steering_cmd - steering_cmd) < 1)
            steering_cmd = previous_steering_cmd;

        previous_steering_cmd = steering_cmd;

        cout << "feedforward angle: " << filtered_feedforward * 57.2958 * 15.4 << " steering angle" << endl;
        cout << "feedback angle: " << filtered_feedback * 57.2958 * 15.4 << " steering angle" << endl;
        cout << "steering angle: " << steering_cmd << endl << endl;

        //longitudinal->set_waypoints(waypoints);
        longitudinal->set_lateral_ff_angle(feedforward.get_feedforward_angle());
        longitudinal->set_lateral_fb_angle(feedback.get_feedback_angle());
        longitudinal->set_current_velocity(speed);
        longitudinal->set_gains(Kp,Ki,Kd,tau);
        cout<<"Desired Velocity"<<desired_velocity;
        longitudinal->set_v_desired(desired_velocity);
        longitudinal->calculate_force_speed_feedback();
        int throttle_output = longitudinal->get_throttle();
        int brake_output = longitudinal ->get_brake();
        if (throttle_output > CarConstants::max_throttle_torque)
        {
            throttle_output = CarConstants::max_throttle_torque;
        }
        else if ( brake_output <= CarConstants::max_brake_torque)
        {
            brake_output = CarConstants::max_brake_torque;
        }
        cout << "Throttle output:"  << throttle_output << endl;
        cout << "Brake output: " << brake_output << endl;
        // Construct new steering msg and publish to the simulation

        // Lateral
        std_msgs::Int64 msg_str;
        msg_str.data = (int)steering_cmd;
        // bolt_msgs::SteeringCmd msg;
        // msg.steering_wheel_angle_cmd = steering_cmd;

         pub_steering_angle.publish(msg_str); //~Publishing Data

        // // Yangwoo added to fix steering angle at the tunnel
        // if (gps_lat != 0 && gps_lon != 0) { // TODO: Add condition from StateAnalyzer's flag
        //     pub_steering_angle.publish(msg);
        // }

        // Lateral for analyzing data
        std_msgs::Float32 lateral;
        lateral.data = feedback.lateral_error;
        pub_error_lateral.publish(lateral); //~Publishing Data

        std_msgs::Float32 head;
        head.data = feedback.heading_error;
        pub_error_heading.publish(head); //~Publishing Data

        std_msgs::Float32 omega;
        omega.data = feedback.angular_velocity_error;
        pub_error_angular_velocity.publish(omega); //~Publishing Data

        std_msgs::Float32 forward;
        forward.data = filtered_feedforward * 15.4 * 57.2958;
        pub_feedforward_angle.publish(forward); //~Publishing Data

        std_msgs::Float32 back;
        back.data = filtered_feedback * 15.4 * 57.2958;
        pub_feedback_angle.publish(back); //~Publishing Data

        // Longitudinal
        std_msgs::Int64 throttle_cmd;
        std_msgs::Int64 brake_cmd;
        std_msgs::Float32 accel;
        std_msgs::Float32 v_crt;
        std_msgs::Float32 v_desire;
        throttle_cmd.data = throttle_output;
        brake_cmd.data = brake_output;
        accel.data = longitudinal->get_accel();
        v_crt.data = speed;
        v_desire.data = desired_velocity;
        pub_throttle.publish(throttle_cmd);//~Publishing Data
        pub_braking.publish(brake_cmd); //~Publishing Data
        pub_accel.publish(accel); //~Publishing Data
        pub_v_current.publish(v_crt); //~Publishing Data
        pub_v_desired.publish(v_desire); //~Publishing Data

        //= Testing Area
        //= Make sure it's in Degrees 
        Data.stamp                  = ros::Time::now();
        Data.x                      = position.x;
        Data.y                      = position.y ;
        Data.current_velocity       = speed;
        Data.desired_velocity       = desired_velocity;
        Data.desired_acceleration   = longitudinal->get_accel();
        Data.throttle_command       = throttle_output;
        Data.brake_command          = brake_output;
        Data.heading                = heading;
        Data.yaw_rate               = imu.angular_velocity.z;
        Data.lateral_error          = feedback.lateral_error;
        Data.heading_error          = feedback.heading_error;
        Data.yaw_rate_error         = feedback.angular_velocity_error;
        Data.feedforward_angle      = filtered_feedforward*57.2958; //YES
        Data.feedback_angle         = filtered_feedback*57.2958;
        Data.total_angle            = (filtered_feedforward + filtered_feedback)*57.2958;
        Data.steering_command       = steering_cmd;
        pub_car_data.publish(Data);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

//@ Cleanup Function


void CombinedROSNode::Cleanup()
{
    std_msgs::String msg;
    msg.data = "stop";
    pub_simulation_status.publish(msg);
    cout << "Combined Controller has stopped and cleaned up successfully." << endl;
}


// THE FOLLOWING FUNCTIONS AREN'T USED ANYWHERE


//@ Quaternion to Euler Conversion Function

geometry_msgs::Vector3 CombinedROSNode::Quaternion2Euler(geometry_msgs::Quaternion q)
{
    //https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    geometry_msgs::Vector3 e;
    //roll
    float sinr = +2.0 * (q.w * q.x + q.y * q.z);
    float cosr = +1.0 - 2.0 * (q.x * q.x + q.y + q.y);
    e.x = atan2(sinr,cosr);
    //pitch
    float sinp = +2.0 * (q.w * q.y - q.z * q.x);
    if (fabs(sinp) >= 1)
        e.y = copysign(M_PI/2,sinp);
    else
        e.y = asin(sinp);
    //yaw
    float siny = +2.0 * (q.w * q.z + q.x * q.y);
    float cosy = +1.0  - 2.0 * (q.y * q.y +q.z * q.z);
    e.z = atan2(siny,cosy);
    return e;

}
