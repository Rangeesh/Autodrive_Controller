//# ====================================================================
//$                          Code Information                           
//- Name:               velocity.cpp
//- Description:        Publishes the Desired Velocity and Stop Distance
//- Language:           C++
//- Custom Header:      velocity.h
//- Created by:         Samir
//- Last Modified:      Aug 2 2019
//- Modified by:        Rangeesh V
//# =====================================================================
//$                        Parameters Calculated
//- Stop Distance
//- Desired Velocity
//? - Probable Velocity from Stop Distance
//? - Probable Velocity from Curvature of Path
//? - Probable Velocity from Speed Limit
//- Ahead Waypoints
//# =====================================================================

//@ Header Files

#include "velocity.h"
#include <math.h>
#include<algorithm>
#include <stdlib.h>
#include<iostream>

using namespace std;

//@ Default Constructor

Velocity::Velocity():loop_rate(30){
}

//@ Initialize: Initializes all variables and ROS Publishers and Subscribers

void Velocity::Initialize(int argc,char **argv){// initial values, these will be updated as the new data are received.

    //= Variables 

    flag=0;
    publish_velocity = 0.0;
    stopping = 0;
    v_max = 0.0;
    stop_distance = -1.0;// just a number

    //= ROS Publishers

    pub_velocity = nh.advertise<std_msgs::Float32>("/desired_velocity",1); //~ Publishes the Velocity at which the car should Travel
    pub_stop_distance = nh.advertise<std_msgs::Float32>("/waypoint_planner/stop_distance",1); //~ Publishes the Distance between the Car and the Last Waypoints

    //= ROS Subscribers

    sub_radius=nh.subscribe("autodrive_sim/output/radius_of_curvature",1,&Velocity::radiusCallback,this);
    //! Let's calculate the Radius of Curvature here as well - What's the harm?

    
    sub_speed_limit = nh.subscribe("/speed_limit",1,&Velocity::speedLimitCallback,this);
    sub_waypoints = nh.subscribe("autodrive_sim/output/waypoints", 1,&Velocity::waypointsCallback, this);
    sub_odom = nh.subscribe("/odom",1,&Velocity::odomCallback,this);
    //!efwerf wfwfwef
    cout<<"INITIALSED PERFECTO";

}

//~ ROS Subscriber CallBacks

//@     Odometry CallBack(msg)                   
//@ Assigns the Current Position in Point my_pose
//@ Assigns the Current Velocity v_current       


void Velocity::odomCallback(const nav_msgs::Odometry& msg){
    my_pose.x = msg.pose.pose.position.x;
    my_pose.y = msg.pose.pose.position.y;
    float x = msg.twist.twist.linear.x;
    float y = msg.twist.twist.linear.y;
    v_current  = sqrt(x*x +y*y);
}

//@      Waypoints CallBack(msg)                 
//@ Assigns read Waypoints to vector wps         
//@ Assigns no. of Waypoints to wp_size          

void Velocity::waypointsCallback(const std_msgs::Float32MultiArray::ConstPtr& msg){//receive 1D waypoints array and convert it to 2D array 
    wps.clear();
    cout<<"Inside waypointsCallback";
    flag =1; // $ NEED THIS
	for(std::vector<float>::const_iterator iterator = msg->data.begin(); iterator != msg->data.end(); iterator++){
        float x = *iterator;
        iterator++;
        float y = *iterator;
        wps.push_back(Point(x, y));
	}
    wp_size = wps.size();
    cout<<"wp_size="<<wp_size;
    //FIXME
    // if (wp_size >20)
    //     wp_size = 20;
}

//@ Radius Callback(msg) : Assigns the Radius of Curvature to radius

void Velocity::radiusCallback(const std_msgs::Float32& msg)
{
    radius=msg.data;
}

//@ Speed Limit Callback(msg) : Assign the Value of Speed Limit to v_max

void Velocity::speedLimitCallback(const std_msgs::Float32& msg){//receive the speed limit
    v_max = msg.data;
}

//@ UpdateLoop() : Publishes the Desired Velocity and Stop Distance
//FIXME: THERE's some iisue with the get_stop_distance function

void Velocity::UpdateLoop(){

    while(ros::ok()){
        if (flag==0)
        continue;
        
        // Obtaining the Desired Velocity
        //!publish_velocity = get_desired_velocity(); 

        std_msgs::Float32 msg;
        //! msg.data = publish_velocity;
        msg.data = 3;
        pub_velocity.publish(msg); //~ Publishing Data


        // // Obtaining the Desired Stop Distance
        //!stop_distance = get_stop_distance();
        // msg.data = stop_distance;
        // pub_stop_distance.publish(msg); //~ Publishing Data


        ros::spinOnce();
        loop_rate.sleep();
    }
}


//@             get_stop_distance()                   
//@ Identifies the Waypoints that are ahead of the Car
//@ Returns the Stop Distance                         

float Velocity::get_stop_distance()
{
    int i=0, stop_dist=0;


        // Skips all points that have been passed by the car
        while (i<wps.size())
        {
            float x1=wps[i].x;
            float y1=wps[i].y;
            float x2=wps[i+1].x;
            float y2=wps[i+1].y;
            float ai=x1-my_pose.x;
            float aj=y1-my_pose.y;
            float bi=-(y2-y1);
            float bj=(x2-x1);
            float determinant = ai*bj-aj*bi;
            if (determinant<=0)
                i++;
        }
    wps_ahead.clear();
    wps_ahead.assign(wps.begin()+i,wps.end()); // Assigning the Ahead WayPoints

    i=1;

    stop_dist += calc_dist_bw_pws(my_pose.x,my_pose.y,wps_ahead[0].x,wps_ahead[0].y); // Distance between Car and first Ahead Waypoint
    
    
    while (i+1<wps_ahead.size())
    {
        stop_dist+=calc_dist_bw_pws(wps_ahead[i].x,wps_ahead[i].y,wps_ahead[i+1].x,wps_ahead[i+1].y);
        i++;
    }
    return stop_dist;
}

//@ Calculates the Distance needed to decrease the Current Velocity to the Target Velocity 


float Velocity::calc_dist_to_output_vel(float v_current,float v_target)
{
    
    //$ Uses Maximum Deceleration 
    return (v_target*v_target - v_current*v_current)/(2*a_y_max);
}


//@ Calculates the Distance between two Waypoints 

float Velocity::calc_dist_bw_pws(float x1,float y1,float x2,float y2){
    return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

//@ Identifies the Probable Velocity of the Car based on the Stop Distance

void Velocity::Set_Velocity_from_Distance()
{
    v_max_dist = pow(fabs(2*a_y_max*(stop_distance)),0.5);
}

//@ Identifies the Probable Velocity of the Car based on the Curvature of the Path

void Velocity::Set_Velocity_from_Path()
{
    v_max_path = sqrt(a_x_max*radius);
}

//@ get_desired_velocity() : Identifies the Desired Velocity

float Velocity::get_desired_velocity()
{
    cout <<"stop distance: " <<stop_distance << endl;

    float speed_lim = float(v_max);

    // //$  If near the end of the Path, the car slows down considerably
    if(stop_distance > calc_dist_to_output_vel(1.0,0) && stop_distance < calc_dist_to_output_vel(float(v_max),0)){
        return 1.0;
    }else if(stop_distance<=calc_dist_to_output_vel(1.0,0)){
        return 0.0;
    }



    //$ Identifying the Minimum Velocity

    Set_Velocity_from_Distance();
    Set_Velocity_from_Path();
    

    float temp=std::min(v_max_dist,v_max_path);
    temp=std::min(temp,v_max);

    return temp;
}

// THE FOLLOWING FUNCTIONS AREN'T BEING USED     

//! THIS FUNCTION ISN'T BEING USED RIGHT NOW
void Velocity::set_vel_from_dist(){

    if(stop_distance < 0.0 || wp_size == 0){

         return;
    }
    //set the target velocity based on the provided stop distance.
    //for the first waypoint
    dist_v_max.clear();
    new_wps.clear();
    float dist_sum = 0.0;

    //calc the distance from the vehicle to the first waypoint ahead of the waypoint array.
    //The current waypoint array is not get updated until the vehicle has passed the second wayoint on the array.
    //If the waypoint array is changed to where the waypoint array get updated after the vehicle has passesed the first waypoint
    //This will be changed to wps[0].x and wps[0].y

    //Adjust distance based on path input
    dist_sum += calc_dist_bw_pws(my_pose.x,my_pose.y,wps[1].x,wps[1].y);
    //cout << "got after first sum" << endl;
     int i = 1;
    while(stop_distance > dist_sum && i < wp_size){//go through the stop distance
        //cout <<"get to while loop"<<endl;
        float temp =pow(fabs(2*a_y_max*(stop_distance - dist_sum)),0.5);
        dist_v_max.push_back(temp);
      //  cout << "get after push back" << endl;
        dist_sum += calc_dist_bw_pws(wps[i].x,wps[i].y,wps[i+1].x,wps[i+1].y);
        new_wps.push_back(wps[i]);
        //cout << dist_v_max[i-1]<< " ";
        i++;
    }

}

//! THIS FUNCTION ISN'T BEING USED RIGHT NOW
Point Velocity::calc_vector(float x1,float y1,float x2,float y2){//calc vector matrix based on 2-point input
    float x = x2 - x1;
    float y = y2 - y1;
    return Point(x,y);
}

//! THIS FUNCTION ISN'T BEING USED RIGHT NOW
void Velocity::set_vel_from_path(){
    calc_radius_of_curvature ();
    //int wp_size = wps.size();

    path_v_max.clear();
    for( int i = 1; i < wp_size; i++)
    {//calc the target velocities of the path based on the radius of curvature and the maxmum allowed lateral acceleration
    //This acceleration is arbitrary 
        path_v_max.push_back(sqrt(a_x_max * radius_of_c[i]));
        //cout << path_v_max[i] << " ";
    }
    //cout << endl;

}

//! THIS FUNCTION ISN'T BEING USED RIGHT NOW
Point Velocity::interpolation(float d_prime, Point p1, Point p2){
    float d = sqrt((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y));

    if (p2.y > p1.y){//top half
        float theta = acos(fabs(p1.x-p2.x)/d);
        if (p2.x > p1.x){//first quadrant
            Point p;
            p.x = cos(theta)*d_prime + p1.x;
            p.y = sin(theta)*d_prime +p1.y;
            return p;
        }else{//second quadrant
            Point p;
            p.x = - cos(theta)*d_prime + p1.x;
            p.y = sin(theta)*d_prime + p1.y;
            return p;
        }

    }else{//bottom half
        float theta = acos(fabs(p2.y-p1.y)/d);
        if(p2.x >p1.x){//fourth quadrant
            Point p;
            p.x = sin(theta)*d_prime + p1.x;
            p.y = -cos(theta)*d_prime + p1.y;
            return p;
        }else{//third quadrant
            Point p;
            p.x = -sin(theta)*d_prime + p1.x;
            p.y = -cos(theta)*d_prime + p1.y;
            return p;
        }
    }

}

//! THIS FUNCTION ISN'T BEING USED RIGHT NOW
float Velocity::calc_radius_of_curvature_3_points(float x1,float y1,float x2,float y2,float x3,float y3)
{//this formula can be found online. http://www.ambrsoft.com/TrigoCalc/Circle3D.htm  The link is not permanent 
    float a = x1*(y2-y3) - y1*(x2-x3)+x2*y3-x3*y2;
    float b = (x1*x1+y1*y1)*(y3-y2)+(x2*x2+y2*y2)*(y1-y3)+(x3*x3+y3*y3)*(y2-y1);
    float c = (x1*x1+y1*y1)*(x2-x3)+(x2*x2+y2*y2)*(x3-x1)+(x3*x3+y3*y3)*(x1-x2);
    float d = (x1*x1+y1*y1)*(x3*y2 - x2*y3)+(x2*x2+y2*y2)*(x1*y3-x3*y1)+(x3*x3+y3*y3)*(x2*x1-x1*y2);
    float x = - b/(2*a);
    float y = -c/(2*a);
    float r = sqrt((x-x1)*(x-x1)+(y-y1)*(y-y1));
    return r;
}




//! THIS FUNCTION ISN'T BEING USED RIGHT NOW
float Velocity::calc_angle_bw_vectors(float x1,float y1,float x2,float y2){
    float uv = x1*x2+y1*y2;
    float du = sqrt(x1*x1+y2*y2);
    float dv = sqrt(x2*x2+y2*y2);
    float temp = uv/(du*dv);
    if( temp > 1.0)
        temp = 1.0;
    return acos(temp)*180.0/PI;

}


//! THIS FUNCTION ISN'T BEING USED RIGHT NOW
void Velocity::calc_radius_of_curvature()//calculate the radius of the curvature of the whole waypoint array
{
    radius_of_c = new float[wp_size];
    if (wp_size < 3){// not enough points to calc. assume straght path
        for(int i = 0;i<3;i++)
            radius_of_c[i]= 999;// random big number
    }
    else{
        for(int i = 1; i <wp_size-2;i++){//look at three waypoint at a time 
            float x1 = wps[i].x;
            float x2 = wps[i+1].x;
            float x3 = wps[i+2].x;
            float y1 = wps[i].y;
            float y2 = wps[i+1].y;
            float y3 = wps[i+2].y;
            radius_of_c[i] = calc_radius_of_curvature_3_points(x1,y1,x2,y2,x3,y3);
            //cout << i <<": " << radius_of_c[i] << " ";
        }
        //cout << endl;
        radius_of_c[wp_size-1] = radius_of_c[wp_size-2] = radius_of_c[wp_size- 3];// set the last 3 waypoints to be the same.

    }
}

