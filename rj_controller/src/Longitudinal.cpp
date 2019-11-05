//# ========================================================================================
//$                          Code Information                           
//- Name:               Longitudinal.cpp
//- Description:        Identifies the Throttle and Brake Commands and Required Acceleration
//- Language:           C++
//- Custom Header:      LateralFeedback.h | LateralFeedforward.h | Longitudinal.h 
//- Created by:         Samir
//- Last Modified:      Aug 2 2019
//- Modified by:        Rangeesh V
//# ======================================================================================== 
//$                        Parameters Calculated
//- 
//# ======================================================================================== 

//@ Headers
#include "CarConstants.h"
#include "LateralFeedforward.h"
#include "LateralFeedback.h"
#include "Longitudinal.h"
#include "PidControl.h"
#include <math.h>
#include <stdlib.h>
#include <vector>
#include <algorithm>
#define _USE_MATH_DEFINES

using namespace std;


/*
calculate_force_speed_feedback() - There are two such functions defined - comment out 1 ... or do you want to overload them??? We'll do that later

The first one is the recently developed one - needs to be checked if it's working

Parameters that can be changed

lpf_reqd_accn.setParams(0.005,0.02); // Defining the Low Pass Filter
pid_to_achieve_speed.setgains(10,0.5,2); // Setting kp, ki and kd values
What's the point of a brake deadband??? Putting it at 0 for now







 */

//@ Default Constructor : Assigning the default values
Longitudinal::Longitudinal() 
{
    control_period = 1.0/80.0;// this is the frequency controller used to publish data
    current_velocity = 0.0;
    prev_velocity = 0.0;
    throttle_cmd = 0;
    brake_cmd = 0;
    prev_throttle_cmd = 0;
    prev_brake_cmd = 0;
    control_v = 12; // m/s
    miu = 0.3; // coefficient of friction
    ax_max = 2.5; // longitudinal accel
    ay_max = 3.3; // lateral accel
    wp_size = 0;
    wp_pitch = 1.0;
    my_pose = Point(0.0,0.0);
    v_desired = 0.0;

}

//@ Exponential Filter : To Return a Smooth output
int Longitudinal::smooth_output(int prev_val,int val, float tau)
{
    return (1-tau)*prev_val + tau*val;
}


//@ Sets the Gain Values for the Longitudinal Controller

void Longitudinal::set_gains(float Kpt, float Kit, float Kdt, float taut)
{//get the gains from dynamic reconfigure for easy tunning.

    Kp = Kpt;
    Ki = Kit;
    Kd = Kdt;
    tau = taut;
}

//@ Sets the Feedforward Angle

void Longitudinal::set_lateral_ff_angle(float angle)
{
    lateral_ff_angle = angle;
}

//@ Sets the Feedback Angle
void Longitudinal::set_lateral_fb_angle(float angle)
{
    lateral_fb_angle = angle;
}

//@ Sets the Current Velocity
void Longitudinal::set_current_velocity( float velocity)
{
    current_velocity = velocity;
}

//@ Sets the Desired Velocity
void Longitudinal::set_v_desired( float velocity)
{
    v_desired= velocity;
}


//@ Returns the Desired Velocity
float Longitudinal::get_v_desired()
{
    return v_desired;
}




//@ Returns the Throttle Command 

int Longitudinal::get_throttle()
{
    int temp;

    if (accel_cmd > 0.0)
    {
        temp  = smooth_output(prev_throttle_cmd,throttle_cmd,0.01);
        prev_throttle_cmd = temp;
    }
    else
    {
        temp  = 0;
        prev_throttle_cmd = 0;
    }
    //cout << "throttle_cmd" << throttle_cmd<<endl;
    //cout << "prev_throttle_cmd" << prev_throttle_cmd << endl;
    //brake_cmd = 0;
    return temp;

}

//@ Returns the Brake Command 
 
int Longitudinal::get_brake()
{
    int temp;
    if(accel_cmd < 0.0)
    {
        temp = smooth_output(prev_brake_cmd,brake_cmd, 0.01);
        prev_brake_cmd = temp;
    }
    else
    {
        temp = 0;
        prev_brake_cmd =0;
    }

    //throttle_cmd = 0;
    return temp;
}


//@ Calculates the Throttle and Brake Command using PID

void Longitudinal::calculate_force_speed_feedback()
{
    float force_speed_feedback = 0.0;

    lpf_reqd_accn.setParams(tau,0.02); //$ Setting parameters for the Low Pass Filter

    double vel_error = v_desired - current_velocity; //$ Error in Velocity

    pid_to_achieve_speed.setRange(-1.0,1.0); //$ Setting Range to the Output - Which in this case is required Acceleration
    pid_to_achieve_speed.setGains(Kp,Ki,Kd); //$ Setting the Gain Values

    pid_to_achieve_speed.resetIntegrator();//$ Resetting the integrator everytime the calculate force speed feedback function is called

    reqd_accn = pid_to_achieve_speed.step(vel_error,control_period); //$ This is the required Acceleration
    lpf_reqd_accn.filt(reqd_accn); //$ Filtering the Acceleration
    double reqd_torque = reqd_accn*CarConstants::mass*CarConstants::tire_radius; //$ Torque to the Wheels
     // calculate lpf_accel

    raw_accel = 80*(current_velocity - prev_velocity); //$ Acceleration = dV*dt
       
    lpf_accel.setParams(tau,0.02); //$ Setting Parameters for Low Pass Filter for required Acceleration
    lpf_accel.filt(raw_accel); //$ Filtering the required Acceleration

    if (reqd_accn>=0)
    {
        throttle_cmd = std::min(float(reqd_torque),CarConstants::max_throttle_torque); //$ This is not needed if I set Range from -1 to 1 ... 
        brake_cmd=int(0);
    }
    
    double brake_deadband = 0; //$ This value might be diffferent for the new code
    if (reqd_accn < - brake_deadband)
    {
        brake_cmd =  std::max(float(reqd_torque),CarConstants::max_brake_torque);
        //cout << "Brake engaging-----------------------: "<< brake_cmd << endl;
        brake_cmd = (int) brake_cmd;
        if (current_velocity < 0.2 && v_desired <=(double)1e-2)
        { //slow enough to floor the brake
            brake_cmd = CarConstants::max_brake_torque;
        }

        throttle_cmd = (int)0;
    }
    
    else
    {
    brake_cmd = int(0);
    }
    return;//torque output
}
// THE FOLLOWING FUNCTIONS AREN'T BEING USED

//NOT USED
float Longitudinal::calculate_force_feedforward() 
{
    //TODO:: Implement if needed
    //return mass*(velocity_desired - velocity_current)*CarConstants::tire_radius;
    return 0.0;
}
//NOT USED
float Longitudinal::calculate_force_slip() 
{
    //TODO:: Implement if needed. I don't understand how one would get the required data.
    return 0;
}


//NOT USED
int Longitudinal::get_torque()
{
    return (int)
    calculate_force_drag() +
    //calculate_force_speed_feedback() +
    calculate_force_feedforward() +
    calculate_force_slip();
}

//NOT USED
float Longitudinal::calculate_force_drag() 
{
    // Current lateral accleration from using the formula for centripetal acceleration
    float lateral_acceleration =0.0;// pow(current_velocity,2) / radius_of_curvature ;

    // Future steering wheel angle
    // TODO:: Check if this is correct. I assumed this is the tire wheel angles respect to the car and it's the one lateral is aiming for.
    float road_wheel_angle = lateral_ff_angle + lateral_fb_angle;

    // Turning force =  m*(b/(a+b))*a_y*tan(abs(delta))
    float force_turning = CarConstants::mass * (CarConstants::rear_length / (CarConstants::front_length + CarConstants::rear_length)) * lateral_acceleration * tan(fabs(road_wheel_angle));

    // Rolling force = Constant rolling
    float force_rolling = CarConstants::constant_rolling;

    // Aero Force = C_d * V_d^2
    float force_aero =0.0;// CarConstants::drag_coefficient * pow(desired_velocity[0],2);

    // Grade Force = m * g * cos(theta_grade)
    // TODO:: Implement later if we have IMU data and also add as an input
    float force_grade = 0;

    float force_drag = force_turning + force_rolling + force_aero + force_grade;
    return force_drag*CarConstants::tire_radius;
}
//THIS is the only ouput at the moment





