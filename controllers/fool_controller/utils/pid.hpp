/************************************************************************
 * Copyright (c) 2024 Alvaro Cabrera Barrio
 * This code is licensed under MIT license (see LICENSE.txt for details)
 ************************************************************************/
/**
 * @file pid.hpp
 * @version 1.0
 * @date 16/11/2024
 * @brief Short description
 *
 * Longer description
 */

#pragma once

class Pid {
public:
  struct Params {
    double max;
    double min;
    double kp;
    double kd;
    double ki;
    double pre_error {0};
    double integral {0};
  };

  Pid() = default;

  explicit constexpr Pid(Params const& params) : p_ {params} { }

  constexpr double calculate(double const set_point, double const pv) {
    // Calculate error
    double const error = set_point - pv;

    // Proportional term
    double const p_out = p_.kp * error;

    // Integral term
    p_.integral += error * utils::delta_time;
    double const i_out = p_.ki * p_.integral;

    // Derivative term
    double const derivative = (error - p_.pre_error) / utils::delta_time;
    double const d_out      = p_.kd * derivative;

    // Calculate total output
    double output = p_out + i_out + d_out;

    // Restrict to max/min
    if (output > p_.max)
      output = p_.max;
    else if (output < p_.min)
      output = p_.min;

    // Save error to previous error
    p_.pre_error = error;
    return output;
  }

private:
  Params p_;
};
