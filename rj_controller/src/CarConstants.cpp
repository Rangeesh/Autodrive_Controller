#include <CarConstants.h>


const float CarConstants::mass = 1950.0f; //! Changed
const float CarConstants::yaw_inertia = 2045.0f; //@  - No Point in Changing
const float CarConstants::front_length = 1.232f; //! Changed
const float CarConstants::rear_length = 1.368f;
// Assuming a slip of 4 deg - it can be calculated, but let's consider it to be a constant for now
// Front axle - 0.827*4*1033.5*9.81 = CorneringStiffness*SlipAngle*AxleMass*Gravity
const float CarConstants::front_stiffness = 33538.6f; //! Changed
const float CarConstants::rear_stiffness = 31863.6f;  //! Changed
const float CarConstants::gravity = 9.81f;
const float CarConstants::front_weight = (mass*rear_length*gravity)/(front_length+rear_length);
const float CarConstants::rear_weight = (mass*front_length*gravity)/(front_length+rear_length);
const float CarConstants::steering_gradient = (front_weight/front_stiffness) - (rear_weight/rear_stiffness); //~Verify - mostly correct
const float CarConstants::tire_radius = 0.31f;  //! Changed
const float CarConstants::drag_coefficient = 0.36f; //@ Find it out
const float CarConstants::constant_rolling = 60.0f; //@Find it out
const float CarConstants::max_throttle_torque = 1199.5; 
const float CarConstants::max_brake_torque = -848;