#include "../include/chassis.hpp"
#include "okapi/api/chassis/controller/chassisController.hpp"
#include "okapi/api/chassis/model/chassisModel.hpp"
#include "okapi/impl/device/motor/motor.hpp"
#include <memory>

chassis::chassis(std::shared_ptr<okapi::Motor> rf,
                 std::shared_ptr<okapi::Motor> rb,
                 std::shared_ptr<okapi::Motor> lf,
                 std::shared_ptr<okapi::Motor> lb) {
  motors[RF] = rf;
  motors[RB] = rb;
  motors[LF] = lf;
  motors[LB] = lb;

  // controller = global::control::controller;
}

void chassis::withOdom(odometry *iodom) { this->odom = iodom; }

void chassis::moveVelocity(float vel) {
  motors[RF]->moveVelocity(vel);
  motors[RB]->moveVelocity(vel);
  motors[LF]->moveVelocity(vel);
  motors[LB]->moveVelocity(vel);
}

void chassis::moveVelocity(float leftVel, float rightVel) {
  motors[RF]->moveVelocity(rightVel);
  motors[RB]->moveVelocity(rightVel);
  motors[LF]->moveVelocity(leftVel);
  motors[LB]->moveVelocity(leftVel);
}

float chassis::toRPM(float velocity) {
  return velocity / (2 * M_PI) / (TRACK_WHEEL_DIA / 2) * 60;
}

pose2D chassis::getPos(){
  return odom->step();
}

x_drive::x_drive(std::shared_ptr<okapi::Motor> rf,
                 std::shared_ptr<okapi::Motor> rb,
                 std::shared_ptr<okapi::Motor> lf,
                 std::shared_ptr<okapi::Motor> lb)
    : chassis(rf, rb, lf, lb){};

void x_drive::moveVelocity(float rf_vel, float rb_vel, float lf_vel,
                           float lb_vel) {
  motors[RF]->moveVelocity(rf_vel);
  motors[RB]->moveVelocity(rb_vel);
  motors[LF]->moveVelocity(lf_vel);
  motors[LB]->moveVelocity(lb_vel);
}

x_drive::motion_vector x_drive::foward_kinematic(motors_value motors_out) {

  return x_drive::motion_vector();
}

x_drive::motors_value x_drive::inverse_kinematic(motion_vector motion) {

  return x_drive::motors_value();
}
