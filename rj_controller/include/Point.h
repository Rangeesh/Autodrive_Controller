#ifndef POINT_H
#define POINT_H
// Commented by RJ
#include <iostream>

class Point {
public:
    float x;
    float y;

    Point() : x(0), y(0) {} // default constructor
    Point(float x1, float y1) : x(x1) , y(y1) {}
    
    friend std::ostream& operator<<(std::ostream &out, const Point &point); // std::ostream is the type that provides stream(sequential) based output
};

inline std::ostream& operator<<(std::ostream &out, const Point &point) {
    out << "(" << point.x << ", " << point.y << ")";
    return out;
}

#endif
