#include "ros/ros.h"
#include "velocity.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "velocity_control_node");
    Velocity *vel = new Velocity();
    vel->Initialize(argc, argv);
    vel->UpdateLoop();
    return 0;
}