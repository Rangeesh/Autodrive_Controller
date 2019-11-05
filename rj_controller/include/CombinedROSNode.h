#ifndef COMBINEDROSNODE_H
#define COMBINEDROSNODE_H

#include <map>
#include <vector>
#include <eigen3/Eigen/Dense>
#include "ros/ros.h"

#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"

//These shall be gone
#include "bolt_msgs/SteeringCmd.h"
#include "bolt_msgs/SteeringReport.h"
#include "bolt_msgs/ThrottleCmd.h"
#include "bolt_msgs/BrakeCmd.h"
#include "std_msgs/Float32MultiArray.h"
#include "bolt_msgs/CarData.h"

//#include "teb_local_planner/FeedbackMsg.h"

#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Vector3.h"// to contain Euler cordinates x = roll, y=pitch,z = yaw
#include "std_msgs/Int64.h"
#include "Point.h"
#include "LateralGains.h"
#include "CombinedNodeException.h"
#include "Longitudinal.h"

#include <dynamic_reconfigure/server.h>
#include <rj_controller/long_gainsConfig.h>


enum Status {
    Running, Stopped
};

class CombinedROSNode {
public:
    CombinedROSNode();
    void Initialize(int argc, char **argv);
    void UpdateLoop();
    void Cleanup();

    // Signal Handers
    static Status status;
    static void sigint_handler(int s);

    // Helpers
    void wait_for_waypoints();

    // Callbacks
    void simulationStatusCallback(const std_msgs::String::ConstPtr& msg);

    // Lateral Callbacks
    void headingCallback(const std_msgs::Float32::ConstPtr& msg);
    void angularVelocityCallback(const std_msgs::Float32::ConstPtr& msg);
    void positionCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void waypointsCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
    //void waypointsTebCallback(const teb_local_planner::FeedbackMsg::ConstPtr& msg);

    // Longitudinal Callbacks
    void steeringCallback(const bolt_msgs::SteeringReport& msg);
    void imuCallback(const sensor_msgs::Imu& msg);
    void desired_velocityCallback(const std_msgs::Float32& msg);
    void gainsCallback(combined_controller::long_gainsConfig &config, uint32_t level);
    
    // Shared Callbacks
    void speedCallback(const nav_msgs::Odometry& msg);
    //void speedCallback(const std_msgs::Float32::ConstPtr& msg);

    // Read file
    void getGains(const char* filename);

    //support functions
    geometry_msgs::Vector3 Quaternion2Euler(geometry_msgs::Quaternion orientation);
    std::map<float, LateralGains> gains_map;
    float highest_gains_speed;

private:

    ros::NodeHandle node;
    ros::Rate loop_rate;

    // variables
    float speed;
    float heading;
    float angular_velocity;
    Point position;
    std::vector<Point> waypoints;
    float desired_velocity;
    float steering;
    float Kp;
    float Ki;
    float Kd;
    float K_lat;
    float K_head;
    float K_omega;

    float tau;
    bolt_msgs::CarData Data;
    sensor_msgs::Imu imu;
    std_msgs::Int64 throttle_cmd;
    std_msgs::Int64 brake_cmd;
    bolt_msgs::SteeringReport steering_report;

    Longitudinal *longitudinal;


    ros::Publisher pub_car_data;

    // Lateral Publishers
    ros::Publisher pub_steering_angle;
    ros::Publisher pub_simulation_status;

    // Lateral Publishers for analyzing data
    ros::Publisher pub_error_lateral;
    ros::Publisher pub_error_heading;
    ros::Publisher pub_error_angular_velocity;
    ros::Publisher pub_feedforward_angle;
    ros::Publisher pub_feedback_angle;

    // Longitudinal Publishers
    ros::Publisher pub_throttle;
    ros::Publisher pub_braking;
    ros::Publisher pub_v_desired;
    ros::Publisher pub_v_current;
    ros::Publisher pub_accel;
    // Lateral Subscribers
    ros::Subscriber sub_heading;
    ros::Subscriber sub_angular_velocity;
    ros::Subscriber sub_waypoints;
    ros::Subscriber sub_position;
    ros::Subscriber sub_simulation_status;

    // Longitudinal Subscribers
    ros::Subscriber sub_imu;
    ros::Subscriber sub_desired_velocity;
    ros::Subscriber sub_steering;

    // Shared Subscribers
    ros::Subscriber sub_speed;



};

#endif

