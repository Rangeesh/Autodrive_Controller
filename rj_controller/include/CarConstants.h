#ifndef CARCONSTANTS_H
#define CARCONSTANTS_H
// Commented by RJ
// What's the use of that CARCONSTANTS_H there??
// This is a preprocessor technique that you can use to prevent the multiple declaration of the header file - defined in all header files

class CarConstants 
{
public:
    static const float mass;
    static const float yaw_inertia;
    static const float front_length;
    static const float rear_length;
    static const float front_stiffness;
    static const float rear_stiffness;
    static const float gravity;
    static const float front_weight;
    static const float rear_weight;
    static const float steering_gradient;
    static const float tire_radius;
    static const float drag_coefficient;
    static const float constant_rolling;
    static const float max_throttle_torque;
    static const float max_brake_torque;
};

#endif  //  CARCONSTANTS_H  
