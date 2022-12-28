#pragma once
#include "main.h"

class coordinate{
    protected:
        float x, y;
    public:
        coordinate();
        coordinate(float x, float y);
        static float getLength(coordinate a,coordinate b);
        void set(float x, float y);
        float getX()const{return this->x;}
        float getY()const{return this->y;}
};

class pose2D : public coordinate{
    protected:
            float heading;
    public:
        pose2D();
        pose2D(float x, float y);
        pose2D(float x, float y, float heading);
        void set(float x, float y, float heading);
        float getHeading()const{return this->heading;}
        static float clampAngle(float angle);
};
