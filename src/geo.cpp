#include <cmath>
#include "../include/geo.hpp"
#define fltNull 0x7fc00000



coordinate::coordinate(): x(0), y(0){};

coordinate::coordinate(float x, float y): x(x), y(y){};

float coordinate::getLength(coordinate a,coordinate b){
    return sqrt(pow(a.x - b.x,2) + pow(a.y - b.y, 2));
};

void coordinate::set(float x, float y){
    this->x = x;
    this->y = y;
}

pose2D::pose2D(): coordinate(), heading(fltNull){};

pose2D::pose2D(float x, float y): coordinate(x, y), heading(fltNull){};

pose2D::pose2D(float x, float y, float heading): coordinate(x, y), heading(heading){};

void pose2D::set(float x, float y, float heading){
    this->x = x;
    this->y = y;
    this->heading = heading;
}

float pose2D::clampAngle(float angle){
    while(angle > 180 || angle < -180){
        angle = angle > 180 ? angle -= 360 : angle += 360;
    }
    return angle;
}

