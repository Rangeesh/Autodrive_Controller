#ifndef LONGITUDINAL_H
#define LONGITUDINAL_H

#include "PidControl.h"
#include "LowPass.h"
#include "Point.h"
// Forward declare LateralFeedforward and Lateralfeedback classes
class LateralFeedforward;
class LateralFeedback;

class Longitudinal {
    // private
private:
    
    float lateral_ff_angle;
    float lateral_fb_angle; 
    float radius_of_curvature;
    float current_velocity;
    float prev_velocity;
    float *path_v_max;
    float *dist_v_max;
    float *v_max;
    float v_desired;
    float prev_v_max;
    float wp_pitch;
    float control_v;
    //std::vector<Point> waypoints;
    float ay_max;
    float  ax_max;
    float *waypoints_x;
    float *waypoints_y;
    float *radius_of_c;
    Point my_pose;
    float path_dist;
    int wp_size;
    float raw_accel;
    float accel_cmd;
    float poly_coefficients[4];
    float miu;
    float Kp;
    float Ki;
    float Kd;
    float tau;
    int wp_number;
    int torque_output;
    int throttle_cmd;
    int brake_cmd;
    int prev_throttle_cmd;
    int prev_brake_cmd;
    PidControl speed_pid;
    PidControl accel_pid;
    PidControl pid_to_achieve_speed;
    float reqd_accn;
    LowPass lpf_accel;
    LowPass lpf_reqd_accn;
    
    double control_period;
public:
    // Constructor to do all the tasks when established 
    Longitudinal();

    // Calculate all the forces needed
    float calculate_force_drag();
    void calculate_force_speed_feedback();
    float calculate_force_feedforward();
    float calculate_force_slip();

    void set_lateral_ff_angle(float);
    void set_lateral_fb_angle(float);
    void set_current_velocity(float);

    void set_gains(float,float,float,float);

    float get_v_desired();

    void set_v_desired(float vd);


    int get_torque();
    int get_throttle();
    int get_brake();
    int get_accel(){return lpf_accel.get();};
    int smooth_output(int prev_val,int val, float tau);
    // Retrieve the private variable torqu_output
    
};

#endif
