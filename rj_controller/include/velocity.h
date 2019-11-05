#ifndef VELOCITY_H
#define VELOCITY_H

//@ Headers

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "nav_msgs/Odometry.h"
#include "Point.h"
#include <vector>

//@ Definitions
#define PI 3.14159265

//@ Class Declarations
class Velocity{

private:

    //= ROS MEMBERS
    ros::NodeHandle nh;
    ros::Rate loop_rate;

    //= ROS PUBLISHERS
    ros::Publisher pub_velocity;
    ros::Publisher pub_stop_distance;
    
    //= ROS SUBSCRIBERS
    ros::Subscriber sub_speed_limit;
    ros::Subscriber sub_radius;
    ros::Subscriber sub_waypoints;
    ros::Subscriber sub_odom;

    float publish_velocity;
    float speed_limit;
    float stop_distance;
    float v_current;
    std::vector<Point> wps;
    std::vector<Point> wps_ahead;
    std::vector<Point> new_wps;
    Point my_pose;
    float *radius_of_c;
    int wp_size;
    bool stopping;
    float radius;
    std::vector<float> path_v_max;
    std::vector<float> dist_v_max;
    std::vector<float> desired_velocity;
    const static float a_y_max= -1; // longitudinal - Deceleration
    const static float a_x_max = 1.5; //lateral
    float v_max; // 2.5 (TBA) lateral 
				    // 3.5 obstacle 
				    // 9.0 sign 
    float v_max_dist;
    float v_max_path;
    int flag; //$ This is a flag for the waypoints to start getting published
    // THE FOLLOWING VARIABLES AREN'T BEING USED
public:
    Velocity();
    void Initialize(int argc, char **argv);
    void UpdateLoop();
    void Set_Velocity_from_Distance();
    void Set_Velocity_from_Path();
    void speedLimitCallback(const std_msgs::Float32& msg);
    void waypointsCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void radiusCallback(const std_msgs::Float32& msg);
    void odomCallback(const nav_msgs::Odometry& msg);
    float get_desired_velocity();
    float get_stop_distance(); 
    float calc_dist_bw_pws(float x1,float y1,float x2,float y2);
    float calc_dist_to_output_vel(float v_current, float v_target);
    
    // THE FOLLOWING FUNCTIONS AREN'T BEING USED
    void set_vel_from_dist();
    void set_vel_from_path();
    float calc_angle_bw_vectors(float x1,float y1,float x2,float y2);
    Point calc_vector(float x1,float y1,float x2,float y2);
    Point interpolation(float d_prime, Point p1, Point p2);
    void calc_radius_of_curvature();
    float calc_radius_of_curvature_3_points(float x1,float y1,float x2,float y2,float x3,float y3);
    

};
#endif
