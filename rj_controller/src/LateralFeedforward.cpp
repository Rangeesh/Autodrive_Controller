//# ========================================================================================
//$                          Code Information                           
//- Name:               LateralFeedforward.cpp
//- Description:        Publishes the Radius of Curvature | Calculates the Feedforward Angle
//- Language:           C++
//- Custom Header:      Lateralfeedforward.h | CombinedNodeException.h | CarConstants.h
//- Created by:         Samir
//- Last Modified:      Aug 2 2019
//- Modified by:        Rangeesh V
//# =========================================================================================
//$                        Parameters Calculated
//- Radius Of Curvature
//- Center of Curvature
//- Feedforward Angle 
//? - Turn Direction
//- The First three Waypoints Ahead of the car
//- Ahead Waypoints
//# ==========================================================================================

//@ Header Files
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <math.h>
#include<stdlib.h>
#include<bits/stdc++.h> 
#include "std_msgs/Float32.h"
#include "CombinedNodeException.h"
#include "CarConstants.h"
#include "LateralFeedforward.h"
#define PI 3.14159265
using namespace Eigen;
using namespace std;

//@ Default Constructor : Initialises all Variables and calls all required Functions

LateralFeedforward::LateralFeedforward(const Point& vehicle_position, const float &speed, vector<Point> &waypoints) :
        radius(0), straight_line(false), vertical_line(false), circle_direction(1) {

    this->waypoint = waypoints;

    if (waypoints.size() < 3)
        feedforward_angle = 0;
        //throw CombinedNodeException("Less than 3 waypoints given");

    EstablishWaypoints(vehicle_position,waypoints);
    CalculateCenterAndRadiusUsingPreview(speed, vehicle_position);
    calculate_feedforward_angle(speed);
    
}


//@ Establishes the Waypoints that are ahead of the car - A repeat of a function in velocity.cpp

void LateralFeedforward::EstablishWaypoints(const Point& vehicle_position, std::vector<Point> &waypoints)
{
    //! Using previous code with small modifications 
    int index = 0;
    while (index < waypoints.size()) 
    {
        if (index > waypoints.size() - 3)
            throw CombinedNodeException("Car moved beyond available waypoints");

        Point point1 = waypoints[index];
        Point point2 = waypoints[index + 1];
        Point point3 = waypoints[index + 2];

        // Get the perpendicular dividing line between points 1 and 2
        Vector2f line_vector(-(point2.y - point1.y), point2.x - point1.x);

        // Get the direction from point1 to the car
        Vector2f car_vector(vehicle_position.x - point1.x, vehicle_position.y - point1.y);

        Matrix2f A;
        A << line_vector, car_vector;
        float determinant = A.determinant();

        if (determinant <= 0) {
            // Ahead of line
            index++;
        }
        else {
            // Behind line
            break;
        }
    }
    //! End
    //= These are the first 3 points ahead of the Car
    circle_points[0] = waypoints[index];
    circle_points[1] = waypoints[index + 1];
    circle_points[2] = waypoints[index + 2];

    //~ This temp is unneccesary here!!!
    std::vector<Point>::const_iterator first = waypoints.begin() + index;
    std::vector<Point>::const_iterator last=waypoints.end();
    std::vector<Point> temp(first,last);

    //= This is the Ahead Waypoints
    this->waypoint=temp;

}

//@ This functions calculates the Center and Radius of Curvature
void LateralFeedforward::CalculateCenterAndRadiusUsingPreview(const float& speed, const Point& vehicle_position)
{
    // Keeping a small preview and checking if it's a straight line or not - a simple check
    
    //TODO: This could be improved - Keep a preview with like 6 - 8 points in waypoint 

    //= Getting the Preview Points - The number of waypoints to be considered - n 
    float Distance_sum =0;
    float x1=vehicle_position.x;
    float y1=vehicle_position.y;
    int index=0;

    for(std::vector<Point>::iterator i=waypoint.begin(); i!=waypoint.end();i++)
    {     
        Point t = *i;
        Distance_sum+= sqrt(pow(waypoint[index+1].x-waypoint[index].x,2)+pow(waypoint[index+1].y-waypoint[index].y,2));
        if (Distance_sum>preview*speed)
            break;
    }

    int n=index+1; //$ This is the number of Points in the Preview - 1 : Number of Line Segments

    float x2=waypoint[n].x;
    float y2=waypoint[n].y;
    Distance_sum=0;
    index=1;

    //$ The Line drawn is from the Car Positon to the Last Waypoint in the Preview

    //= Checking if it is a Straight Line

    for(std::vector<Point>::iterator i=waypoint.begin(); i!=waypoint.end();i++)
    {   
        if (index>n)
            break;
        index++;  //- If it breaks because of this, it is the number of waypoints... But what if it is limited by the For Loop Constraint??
        Point t = *i;
        Distance_sum+= abs((y1-y2)*t.x + (x2-x1)*t.y + x1*y2 - x2*y1)/sqrt(pow((y2-y1),2)+pow((x2-x1),2));
    }
    //$ This is the average Distance of a Waypoint from the Line

    //FIXME: What if there are no waypoints?? Division of zero will occur

    if (Distance_sum/(index-1)<= straight_line_classification_parameter)
    {
        radius=999; // Huge Value
        straight_line=true;
        return;
    }


    //= If it is a Curved Path 

    float Sx, Sy, Sx2, Sy2, Sx3, Sy3, Sxy, Sxy2, Sx2y, E = 0;

    index=1;
    n++; // - Now n is the total number of points!!

    //$ Finding all the individual terms required for the Calculation of the Center

    for(std::vector<Point>::iterator i=waypoint.begin(); i!=waypoint.end();i++)
    {
        if (index>n)
            break;
        index++;

        Point t = *i;
        Sx+=t.x;
        Sy+=t.y;
        Sx2+=t.x*t.x;
        Sy2+=t.y*t.y;
        Sxy+=t.x*t.y;
        Sx3+=t.x*t.x*t.x;
        Sy3+=t.y*t.y*t.y;
        Sxy2+=t.x*t.y*t.y;
        Sx2y+=t.x*t.x*t.y;
    }

    float N = n;
    float a = 2*(Sx*Sx-N*Sx2);
    float b = 2*(Sx*Sy-N*Sxy);
    float c = 2*(Sy*Sy - N*Sy2);
    float P = Sx*Sx2 + Sx*Sy2 - (N*Sx3+N*Sxy2);
    float Q = Sy*Sy2 + Sy*Sx2 - N*Sy3 - N*Sx2y;
    float Yc = (b*P - a*Q)/(b*b-a*c);
    float Xc = (b*Q - c*P)/(b*b-a*c);

    index=1;


    for(std::vector<Point>::iterator i=waypoint.begin(); i!=waypoint.end();i++)
    {
        // - Same question above applies here ... 
        if (index>n)
            break;
        index++;
        Point t = *i;
        E+=(t.x-Xc)*(t.x-Xc) + (t.y-Yc)*(t.y-Yc);
    }

    float R = sqrt(E/N);

    center.x=Xc;
    center.y=Yc;
    radius=R;
    cout<<"Radius: "<<radius<<endl;
    cout<<"Center :"<<Xc<<Yc<<endl;
    std_msgs::Float32 msg;
    msg.data = radius;
    pub_radius.publish(msg); //~ Publishing Data
    Calculate_Turn_Direction(Xc,Yc,n);

}
//! END

//@ Finding the Direction of the Turn : Calculate Net Angle and Take a Call

void LateralFeedforward::Calculate_Turn_Direction(float& xc, float& yc, int n)
{
    int index = 1;
    float angle =0.0;
    for(std::vector<Point>::iterator i=waypoint.begin(); i!=waypoint.end();i++)
    {
        float x1=waypoint[index-1].x;
        float y1=waypoint[index-1].y;
        float x2=waypoint[index].x;
        float y2=waypoint[index].y;
        angle+=CalculateAnglebetweenThreePoints(x1,y1,x2,y2,xc,yc);
        // - Same question above applies here ... 
        if (index>=n-1)
            break;
        index++;
        
    }
    
    if (angle<=0)
        {
            // The car must take a RIGHT Turn
            circle_direction = -1;
        }
    else
    {
        // The car is taking a LEFT Turn
        circle_direction = 1;
    }


}

//@ Calculating Angle Between three Points
//= Convention: Clockwise is Positive - Left Turn
float LateralFeedforward::CalculateAnglebetweenThreePoints(float x1, float y1, float x2, float y2, float xc, float yc)
{
    float a1 = x1-xc;
    float a2 = y1-yc;
    float b1 = x2-xc;
    float b2 = y2-yc;
    float dot = a1*a2 + b1*b2;
    float det = a1*b2 - a2*b1;
    return atan2(det,dot)*180/PI;

}

//@ This function identifies the final Feedforward Angle

void LateralFeedforward::calculate_feedforward_angle(const float &speed) {
    if (straight_line || vertical_line) {
        feedforward_angle = 0;
        return;
    }

    float length_between_axles = CarConstants::front_length + CarConstants::rear_length;
    float angle = length_between_axles / radius + ((CarConstants::steering_gradient * speed * speed) /
        (CarConstants::gravity * radius));
    feedforward_angle =
        circle_direction * angle;
}


// THE FOLLOWING FUNCTIONS ARE NOT BEING USED 

//! Not Being Used Right Now
//@ This function identifies the first three Waypoints that are ahead of the car 

void LateralFeedforward::find_circle_points(const Point& vehicle_position, vector<Point> &waypoints) {

    int index = 0;
    while (index < waypoints.size()) {
        if (index > waypoints.size() - 3)
            throw CombinedNodeException("Car moved beyond available waypoints");

        Point point1 = waypoints[index];
        Point point2 = waypoints[index + 1];
        Point point3 = waypoints[index + 2];

        // Get the perpendicular dividing line between points 1 and 3
        Vector2f line_vector(-(point3.y - point1.y), point3.x - point1.x);

        // Get the direction from point2 to the car
        Vector2f car_vector(vehicle_position.x - point2.x, vehicle_position.y - point2.y);

        Matrix2f A;
        A << line_vector, car_vector;
        float determinant = A.determinant();

        if (determinant <= 0) {
            // Ahead of line
            index++;
        }
        else {
            // Behind line
            break;
        }
    }
    circle_points[0] = waypoints[index];
    circle_points[1] = waypoints[index + 1];
    circle_points[2] = waypoints[index + 2];
}

//! Not Being Used Right Now
//@ This function identifies the Circle Radius and Center of the 3 ahead Waypoints 

void LateralFeedforward::circle_fit() {
    Point point1 = circle_points[0];
    Point point2 = circle_points[1];
    Point point3 = circle_points[2];

    //in the case we get all three points to be vertical
    if (point2.x - point1.x == 0 && point3.x - point2.x == 0) {
        vertical_line = true;
        return;
    }

    // check if points need to be reordered to not get a vertical slope
    // (which would cause us to divide by 0)
    // http://paulbourke.net/geometry/circlesphere/
    if (point2.x - point1.x == 0 || point3.x - point2.x == 0) {
        point1 = circle_points[2];
        point2 = circle_points[0];
        point3 = circle_points[1];
        cout << "reordered points" << endl;
    }

    // Get slopes of lines between points 1 and 2 and points 2 and 3
    float m1 = (point2.y - point1.y) / (point2.x - point1.x);
    float m2 = (point3.y - point2.y) / (point3.x - point2.x);
    // if (abs(m1 - m2) < 0.01) {
    if (m1 <= m2 + 0.000005 && m1 >= m2 - 0.0000005) {
        cout << "Parallel lines\n";
        straight_line = true;
        return;
    }

    // Calculate the center of the circle made by points 1, 2, and 3
    float x = ((m1 * m2 * (point1.y - point3.y)) + (m2 * (point1.x + point2.x)) - (m1 * (point2.x + point3.x))) / (2 * (m2 - m1));
    float y = ((-1 / m1) * (x - ((point2.x + point1.x) / 2))) + ((point2.y + point1.y) / 2);




    // center = Point(x, y);


    //cout << "center: " << center << "\n";

    // Calculate the radius of the circle
    Vector2f radius_vector(point1.x - center.x, point1.y - center.y);




    // radius = radius_vector.norm();





    //cout << "radius: " << radius << endl;

    Vector2f v1(circle_points[1].x -  circle_points[0].x,  circle_points[1].y -  circle_points[0].y);
    Vector2f v2(circle_points[2].x -  circle_points[1].x,  circle_points[2].y -  circle_points[1].y);

    // Use the determinant of the vectors v1 and v2
    // to calculate the direction of circle
    // (left turn = positive determinant)
    // (right turn = negative determinant)
    Matrix2f A;
    A << v1, v2;



// ***************************************************


    // float determinant = A.determinant();
    // if (determinant < 0)
    //     circle_direction = -1;

    // if (circle_direction == 1)
    //     cout << "road direction = left\n";
    // else if (circle_direction == -1)
    //     cout << "road direction = right\n";
    // else
    //     throw CombinedNodeException("Bad circle direction");


// *******************************************************

}
