//# =========================================================================
//$                          Code Information                           
//- Name:               LateralFeedback.cpp
//- Description:        Identifies the Required Errors and the Feedback Angle
//- Language:           C++
//- Custom Header:      LateralFeedback.h | LateralFeedforward.h
//- Created by:         Samir
//- Last Modified:      Aug 2 2019
//- Modified by:        Rangeesh V
//# =========================================================================
//$                        Parameters Calculated
//- Feedback Angle
//- The Errors
//? - Lateral Error
//? - Heading Error
//? - Yaw Rate Error
//# =========================================================================

//@ Header Files
#include <iostream>
#include <math.h>
#define _USE_MATH_DEFINES
#include <eigen3/Eigen/Dense>
#include "LateralFeedback.h"
#include "LateralFeedforward.h"
using namespace Eigen;
using namespace std;

//@ Default Constructor : Directly Calls the Required functions and finds the Feedback Angle

LateralFeedback::LateralFeedback(LateralFeedforward *ff, Point &car_position, float car_heading, float angular_velocity, float speed, float k_lat, float k_head, float k_o) {
    if (ff->waypoint.size() < 2)
        feedback_angle = 0;
    
    else 
        feedback_angle = 
            calculate_lateral_error(ff, car_position, car_heading, k_lat) + 
            calculate_heading_error(ff, car_position, car_heading, k_head) +
            calculate_omega_error(ff, angular_velocity, speed, k_o);
}

//@ Calculating the Lateral Error


//@ Right now, I'm assuming the Heading Angle has a value 0 at x=0 : The East Direction

float LateralFeedback::calculate_lateral_error(LateralFeedforward *ff, Point &car_position, float car_heading, float k_lat) {
    float distance;
    Matrix2f direction;
    float direction_sign;
    int straight_direction;

    // special case where the waypoints are on a vertical line
    if (ff->vertical_line) {
        // find distance
        distance = abs(car_position.x - ff->circle_points[0].x);
        direction << cos(car_heading), sin(car_heading), (car_position.x - ff->circle_points[0].x), 0; // may need to switch cos and sin to be the last two terms
        direction_sign = direction.determinant();

        // determine if a vaertical line ever happened
        cout << "HEY VERTICAL LINE HAPPENED FOR LATERAL" << endl;
        //= If it is offset on the Left Side (Car should take Right Turn) - direction_sign becomes -ve => straight_direction is -ve => lateral error is +ve(This should Change) - Correction is +ve -- Right turn
        //determine sign of the direction
        if (direction_sign >= 0) {
            straight_direction = 1;
        }
        else if(direction_sign <0) {
            straight_direction = -1;
        }
        //FIXME: Need to verify this based on how it is used in CombinedROSNode.cpp
        lateral_error = -straight_direction * distance;
    }
    
    // case where the waypoints are on a straight line
    else if (ff->straight_line) {
        float determinant = abs(((ff->circle_points[1].x - ff->circle_points[0].x) * (ff->circle_points[0].y - car_position.y)) - ((ff->circle_points[0].x - car_position.x) * (ff->circle_points[1].y - ff->circle_points[0].y)));
        Vector2f line(ff->circle_points[1].x - ff->circle_points[0].x, ff->circle_points[1].y - ff->circle_points[0].y);
        float denominator = line.norm();
        distance = determinant / denominator;
        direction << cos(car_heading), sin(car_heading), (car_position.x - ff->circle_points[0].x), (car_position.y - ff->circle_points[0].y);
        direction_sign = direction.determinant(); //~ WTF?? Was this missing?? 
        // Not convinced with direction




        float x1 = ff->circle_points[0].x - car_position.x;
        float y1 = ff->circle_points[0].y - car_position.y;
        float x2 = ff->circle_points[1].x - ff->circle_points[0].x;
        float y2 = ff->circle_points[1].y - ff->circle_points[0].y;

        float determnt = x1*y2 - x2*y1; //FIXME: I want to use this instead - -ve means left turn - Isn't it Right turn???

        if (determnt >= 0) {
            straight_direction = 1;
        }
        else if(determnt <0) {
            straight_direction = -1;
        }
        //= If car should take Right Turn - determnt is -ve, straight_direction is -1 => Lateral Error is -ve, correction is -ve
        //FIXME: There's a definite error!!!!

        // determine if a straight lines ever happened
        cout << "HEY STRAIGHT LINE HAPPENED FOR LATERAL" << endl;

        // if (direction_sign >= 0) {
        //     straight_direction = 1;
        // }
        // else if(direction_sign <0) {
        //     straight_direction = -1;
        // }
        lateral_error = straight_direction * distance;
        // positive error = left turn
        // negative error = right turn
    }

    else {
        //Calculate distance from the vehicle's position to the center of the feedforward circle - Seems to be right!
        Vector2f distance_vector(car_position.x - ff->center.x, car_position.y - ff->center.y);
        distance = distance_vector.norm();

        // positive error = left turn
        // negative error = right turn
        //= Right Turn, outside the Circle - The Car must take a Right Turn - lateral Error is -ve, since Right direction is -ve. Correction is -ve
        lateral_error = ff->circle_direction * (distance - abs(ff->radius)); //FIXME: FIGURE THIS OUT
    }
    // now calculates correction
    float correction = k_lat * lateral_error;
    cout << "lateral error: " << lateral_error << " meters" << endl;
    //FIXME: Now, you need to find out if it's ff+fb or ff-fb ultimately...
    return correction;
}

float LateralFeedback::calculate_heading_error(LateralFeedforward *ff, Point &car_position, float car_heading, float k_head) {
    float road_heading;
    Point road_vector;
    // if no circle is formed, it must be straight
    if (ff->vertical_line || ff->straight_line) { 
         road_vector = Point((ff->circle_points[1].x - ff->circle_points[0].x), (ff->circle_points[1].y - ff->circle_points[0].y));
         cout << "HOWDY STRAIGHT LINE THING JUST HAPPPENED FOR HEADING" << endl;
    }
    // if circle is formed
    else {
        // Calculate vector from circle center to car position
        Point circle_vector(car_position.x - ff->center.x, car_position.y - ff->center.y);

        // get tangent vector to the road vector (depends on the circle's direction)
        if (ff->circle_direction == 1)
            road_vector = Point(-circle_vector.y, circle_vector.x);
        else
            road_vector = Point(circle_vector.y, -circle_vector.x);
    }

    // get the road's heading in world space
    road_heading = atan2(road_vector.y, road_vector.x); //~ This also uses the fact that the Car Heading is 0 along the x axis
    if (road_heading < 0)
        road_heading += M_PI * 2;

    //= For the car to take a Right Turn, Car_Heading > Road_Heading => Heading Error is +ve, and correction is +ve. 
    car_heading = fmod(car_heading, M_PI * 2);
    // Calculate the angle difference between the car's and the road's headings
    heading_error = road_heading - car_heading;
    if (heading_error >= M_PI)
        heading_error -= M_PI * 2;
    else if (heading_error <= -M_PI)
        heading_error += M_PI * 2;

    // apply gain to the heading error

    //@ Why the 15.4 over there?? This is the relation between the Steering Angle and the Tire Turning Angle - You calculate the Tire Angles
    //@ I'm not convinced with the Heading Angle Error - Need to verify the Outputs


    float correction = k_head * heading_error;
    cout << "road heading : " << road_heading * 57.2958 * 15.4 << " steering degrees" << endl; //= What's the use of sending it in Steering Degrees??
    cout << "car heading: " << car_heading * 57.2958 * 15.4 << " steering degrees" << endl;
    cout << "heading error: " << heading_error * 57.2958 * 15.4 << " steering degrees" << endl;
    cout << "heading error correction: " << correction << endl;
    return correction;
}

float LateralFeedback::calculate_omega_error(LateralFeedforward *ff, float angular_velocity, float speed, float k_o) {
    if (ff->straight_line || ff->vertical_line) {
        cout << "STRAIGHT LINE CASE FOR ANGULAR VELOCITY" << endl;
        angular_velocity_error = angular_velocity;
        //= If the car should take a Right turn - it means right now it is rotating to the left  => angular_velocity is +ve => correction is -ve
    }
    else
        angular_velocity_error = (angular_velocity - ff->circle_direction * speed / ff->radius);
        //= If the Car should take a right turn - error is +ve - correction is -ve
    
        
    float correction = -k_o * angular_velocity_error; // speed / radius
    cout << "angular velocity error: " << angular_velocity_error * 57.2958 * 15.4 << " steering degrees per second" << endl;
    return correction;
}
