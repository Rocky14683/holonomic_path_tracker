#pragma once

#include "../include/math.hpp"
#include "main.h"
#include "okapi/api/units/QTime.hpp"
#include "okapi/api/util/abstractRate.hpp"
#include "okapi/api/util/abstractTimer.hpp"
#include "okapi/impl/util/timer.hpp"
#include <cmath>

class PIDcontroller {
private:
  float kp, ki, kd;
  okapi::QTime dt;
  float position_tolerance;
  float velocity_tolerance;

  float measurement;
  float velocity_error, prev_velocity_error;
  float position_error, prev_position_error;
  float total_error;

  float min_input, max_input;
  float min_integral, max_integral;

  float goal;

  bool forced_continuous = false;

public:
  PIDcontroller(float kp, float ki, float kd, okapi::QTime dt);
  void setPID(float kp, float ki, float kd);
  void setP(float kp);
  void setI(float ki);
  void setD(float kd);
  void set_position_tolerance(float position_tolerance);
  void set_velocity_tolerance(float velocity_tolerance);

  void setGoal(float goal);
  float getGoal();
  bool atGoal();

  void make_continous(float min_input, float max_input);

  float get_position_error();
  float get_velocity_error();

  void reset();

  float calculate(float input);
};
