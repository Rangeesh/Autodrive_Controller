#ifndef LATERALFEEDFORWARD_H
#define LATERALFEEDFORWARD_H

#include <vector>
#include "Point.h"
#include "ros/ros.h"

class Point;

class LateralFeedforward {

    ros::NodeHandle nh;
    ros::Publisher pub_radius;
    

    
    void EstablishWaypoints(const Point& vehicle_position, std::vector<Point> &waypoints);
    void CalculateCenterAndRadiusUsingPreview(const float& speed, const Point& vehicle_position);
    void Calculate_Turn_Direction(float& Xc, float&Yc, int n);
    float CalculateAnglebetweenThreePoints(float x1, float y1, float x2, float y2, float xc, float yc);

    // THE FOLLOWING FUNCTIONS AREN'T BEING USED
    void find_circle_points(const Point& vehicle_position, std::vector<Point> &waypoints);
    void circle_fit();
    void calculate_feedforward_angle(const float &speed); // This uses the radius
    
public:
    // establish waypoints
    std::vector<Point> waypoint; //The points are the points ahead of the car

    // The circle's center
    // Changing its value
    Point center;

    // Radius of curvature
    // Changing its value
    float radius;

    // Check for circle points that make a straight line
    bool straight_line;

    // Check for circle points that make specifically straight line
    bool vertical_line;

    // curving left = 1, curving right = -1
    int circle_direction;

    float feedforward_angle;
    float preview = 1.0; //$ The amount of time that we're looking ahead - We analyse those Waypoints alone
    float straight_line_classification_parameter=1.0; //$ If all waypoints on an average are within this distance from the Straight Line Drawn
    Point circle_points[3];

    LateralFeedforward(const Point& vehicle_position, const float &speed, std::vector<Point> &waypoints);
    const float get_feedforward_angle() { return feedforward_angle; }
};

#endif