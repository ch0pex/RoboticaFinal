#pragma once

#include "MyRobot.hpp"

struct Orientation {
  void update(MyRobot& robot) { }
  void enter(MyRobot& robot) { }
};
/*
  [[nodiscard]] bool isFacingDesiredAngle() const { ///  This code is thrash change this for a PID
    double const compass_angle = facingAngle();

    // print sensor values to console
    std::cout << "Desired angle (degrees): " << desired_angle_ << "\n";
    std::cout << "Compass angle (degrees): " << compass_angle << "\n";

    if (compass_angle * desired_angle_ < 0 && abs(compass_angle - desired_angle_) > 180)
    // Going through angle discontinuity
    {
      if (compass_angle < (desired_angle_ - 2)) {
        // turn left
        return false;
      }
      if (compass_angle > (desired_angle_ + 2)) {
        // turn right
        return false;
      }
      // move straight forward
      return true;
    }

    if (compass_angle < (desired_angle_ - 2)) {
      // turn right
      return false;
    }
    if (compass_angle > (desired_angle_ + 2)) {
      // turn left
      return false;
    }
    // move straight forward
    return true;
  }
*/
