#include "../include/PID.hpp"
#include "math.hpp"
#include "okapi/api/util/mathUtil.hpp"

PIDcontroller::PIDcontroller(float kp, float ki, float kd, okapi::QTime dt)
    : kp(kp), ki(ki), kd(kd), dt(dt){};

void PIDcontroller::setPID(float kp, float ki, float kd) {
  this->kp = kp;
  this->ki = ki;
  this->kd = kd;
}

void PIDcontroller::setP(float kp) { this->kp = kp; }

void PIDcontroller::setI(float ki) { this->ki = ki; }

void PIDcontroller::setD(float kd) { this->kd = kd; }

void PIDcontroller::set_position_tolerance(float position_tolerance) {
  this->position_tolerance = position_tolerance;
}

void PIDcontroller::set_velocity_tolerance(float velocity_tolerance) {
  this->velocity_tolerance = velocity_tolerance;
}

void PIDcontroller::setGoal(float goal) {
  this->goal = goal;
  if (this->forced_continuous) {
    float error_bound = (max_input - min_input) / 2.0;
    this->position_error = math::inputModulus(this->goal - this->measurement,
                                               -error_bound, error_bound);
  } else
    this->position_error = this->goal - this->measurement;

  this->velocity_error = (position_error - prev_position_error) / dt.getValue();
}

float PIDcontroller::getGoal() { return this->goal; }

bool PIDcontroller::atGoal() {
  return std::abs(position_error) < position_tolerance &&
         std::abs(velocity_error) < velocity_tolerance;
}

void PIDcontroller::make_continous(float min_input, float max_input) {
  this->forced_continuous = true;
  this->min_input = min_input;
  this->max_input = max_input;
}

float PIDcontroller::get_position_error() { return this->position_error; }

float PIDcontroller::get_velocity_error() { return this->velocity_error; }

void PIDcontroller::reset() {
  this->velocity_error = 0;
  this->prev_velocity_error = 0;
  this->position_error = 0;
  this->prev_position_error = 0;
}

float PIDcontroller::calculate(float input) {
  this->measurement = input;
  this->prev_position_error = this->position_error;

  if (this->forced_continuous) {
    double error_bound = (this->max_input - this->min_input) / 2.0;
    this->position_error =
        math::inputModulus((this->goal - this->measurement), -error_bound, error_bound);
  } else {
    this->position_error = this->goal - this->measurement;
  }

  this->velocity_error = (this->position_error - this->prev_position_error) / dt.getValue();

  if (this->ki != 0) {
    this->total_error =
        math::clamp<float>(this->total_error + this->position_error * dt.getValue(),
                   this->min_integral / this->ki, this->max_integral / this->ki);
  }

  return this->kp * this->position_error + this->ki * this->total_error + this->kd * this->velocity_error;
}
