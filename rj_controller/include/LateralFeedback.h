#ifndef LATERALFEEDBACK_H
#define LATERALFEEDBACK_H

#include "Point.h"

// Forward declare LateralFeedforward class
class LateralFeedforward;

class LateralFeedback {

public:
    // Final variable calculated
    float feedback_angle;

    // Errors that need to be analyzed after testing
    float lateral_error;
    float heading_error;
    float angular_velocity_error;
    
    // Constructor to do all calculations at the same time
    LateralFeedback(LateralFeedforward *ff, Point &car_position, float car_heading, float angular_velocity, float velocity, float k_lat, float k_head, float k_o);

    // Calculate all of the corrections
    float calculate_lateral_error(LateralFeedforward *ff, Point &car_position, float car_heading, float k_lat);
    float calculate_heading_error(LateralFeedforward *ff, Point &car_position, float car_heading, float k_head);
    float calculate_omega_error(LateralFeedforward *ff, float angular_velocity, float velocity, float k_o);

    // get the roadway feedback angle required
    float get_feedback_angle() { return feedback_angle; }
};

#endif
