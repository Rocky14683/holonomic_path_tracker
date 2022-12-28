#include "../include/odometry.hpp"
#include "okapi/impl/device/rotarysensor/adiEncoder.hpp"
#include "pros/rtos.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>

#define DEBUG true


odometry::odometry(std::shared_ptr<okapi::ADIEncoder> right_odom, std::shared_ptr<okapi::ADIEncoder> middle_odom, std::shared_ptr<okapi::ADIEncoder> left_odom, float right_offset, float middle_offset, float left_offset){
    offset[RIGHT] = right_offset;
    offset[MIDDLE] = middle_offset;
    offset[LEFT] = left_offset;

    encoder[RIGHT] = right_odom;
    encoder[MIDDLE] = middle_odom;
    encoder[LEFT] = left_odom;

    this->tare_sensor();

    pros::Task loop(this->start_odom, this, "Odom");
}

void odometry::start_odom(void* iparam){
    if(iparam){
        odometry* that = static_cast<odometry*>(iparam);
        that->step();
        pros::delay(10);
    }
}

void odometry::tare_sensor(){
    for(std::shared_ptr<okapi::ADIEncoder> curr_encoder : encoder){
        curr_encoder->reset();
    }
}

void odometry::setStartPose(float x, float y, float heading){
    this->STARTPOSE = pose2D(x, y, heading);
}

pose2D odometry::step(){

    float cur_R = encoder[RIGHT]->get();
    float cur_L = encoder[LEFT]->get();
    float cur_M = encoder[MIDDLE]->get();

    float delta_R_length = (cur_R - prev_encoder_val[RIGHT]) * tick_to_length;
    float delta_L_length = (cur_L - prev_encoder_val[LEFT]) * tick_to_length;
    float delta_M_length = (cur_M - prev_encoder_val[MIDDLE]) * tick_to_length;

    float arc_R = cur_R * tick_to_length, arc_L = cur_L * tick_to_length;

    float delta_theta = ((delta_R_length - delta_L_length) / (offset[RIGHT] - offset[LEFT])) - current_pose.getHeading();

    float delta_heading = prev_pose.getHeading() - offset[MIDDLE] * delta_theta;

    float delta_X_local = delta_theta == 0 ? delta_heading : (2 * sin( delta_theta / 2 )) * (delta_M_length / delta_theta + offset[MIDDLE]);
    float delta_Y_local = delta_theta == 0 ? delta_R_length : (2 * sin( delta_theta / 2 )) * (delta_R_length / delta_theta + offset[RIGHT]);

    float theta_new = (current_pose.getHeading() + delta_heading / 2);
    float theta = atan2f(delta_Y_local, delta_X_local) - theta_new;
    float radius = sqrt(pow(delta_X_local,2) + pow(delta_Y_local,2));

    float delta_X_glob = radius * cos(theta);
    float delta_Y_glob = radius * sin(theta);

    current_pose.set(current_pose.getX() - delta_X_glob, current_pose.getY() - delta_Y_glob, pose2D::clampAngle(theta_new));
    prev_pose = current_pose;

    prev_encoder_val[RIGHT] = cur_R;
    prev_encoder_val[MIDDLE] = cur_M;
    prev_encoder_val[LEFT] = cur_L;

    if(DEBUG){
        std::cout<<this->current_pose.getX()<<", "<<this->current_pose.getY()<<", "<<this->current_pose.getHeading()<<std::endl;
    }

    return current_pose;
}
