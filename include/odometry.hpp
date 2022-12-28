#pragma once
#include "main.h"
#include <memory>
#include "geo.hpp"


#define DEBUG true
#define TRACK_WHEEL_DIA 2.75

class odometry{
    private:
        pose2D STARTPOSE;
        float offset[3];
        float prev_encoder_val[3] = {0, 0, 0};
        static constexpr float tick_to_length = (M_PI / 180) * (TRACK_WHEEL_DIA/2);
        pose2D current_pose;
        pose2D prev_pose;
    public:

        std::shared_ptr<okapi::ADIEncoder>encoder[3];
        enum encoderID{
            RIGHT = 0,
            MIDDLE = 1,
            LEFT = 2
        };
        odometry(std::shared_ptr<okapi::ADIEncoder> right_odom, std::shared_ptr<okapi::ADIEncoder> middle_odom, std::shared_ptr<okapi::ADIEncoder> left_odom, float right_offset, float middle_offset, float left_offset);
        static void start_odom(void* iparam);
        void tare_sensor();
        void setStartPose(float, float, float);
        pose2D step();
};
