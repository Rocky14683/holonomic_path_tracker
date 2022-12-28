#pragma once

#include "../include/odometry.hpp"
#include "geo.hpp"
#include "main.h"
#include <cstddef>
#include <memory>

class chassis {
protected:
  static std::shared_ptr<okapi::Controller> controller;
  std::shared_ptr<okapi::Motor> motors[4];
  odometry *odom;

public:
  struct motors_value {
    float rf, rb, lf, lb;
  };
  enum chassis_motor_ID { RF = 0, RB = 1, LF = 2, LB = 3 };
  chassis() = delete;
  chassis(std::shared_ptr<okapi::Motor> rf, std::shared_ptr<okapi::Motor> rb,
          std::shared_ptr<okapi::Motor> lf, std::shared_ptr<okapi::Motor> lb);
  void withOdom(odometry *iodom);

  void moveVelocity(float vel);
  void moveVelocity(float leftVel, float rightVel);
  float toRPM(float velocity);
  pose2D getPos();
};

class x_drive : public chassis {
public:
  struct motion_vector {
    float v_x, v_y, w; // x-axis velocity, y-axis velocity, rotational velocity
  };
  x_drive() = delete;
  x_drive(std::shared_ptr<okapi::Motor> rf, std::shared_ptr<okapi::Motor> rb,
          std::shared_ptr<okapi::Motor> lf, std::shared_ptr<okapi::Motor> lb);

  void moveVelocity(float rf_vel, float rb_vel, float lf_vel, float lb_vel);

  static motion_vector foward_kinematic(motors_value);
  static motors_value inverse_kinematic(motion_vector);
};
