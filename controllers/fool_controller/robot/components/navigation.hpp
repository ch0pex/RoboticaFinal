#pragma once

#include "utils/common.hpp"
#include "utils/math.hpp"

#include <webots/Compass.hpp>
#include <webots/Robot.hpp>

using namespace webots;

class Actuators {
public:
  // *** Constructors ***
  explicit Actuators(Robot& robot) : compass_(robot.getCompass("compass")) { }

  void computeOdometry() {
    pos_.x += ((sr_ + sl_) / 2 * sin(theta_ + ((sr_ - sl_) / (2 * utils::wheel_distance))));
    pos_.y += ((sr_ + sl_) / 2 * cos(theta_ + ((sr_ - sl_) / (2 * utils::wheel_distance))));
    theta_ = theta_ + ((sr_ - sl_) / utils::wheel_distance);
  }

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

  void setDesiredAngle(double const angle) { desired_angle_ = angle; }

  [[nodiscard]] double facingAngle() const { return math::convert_bearing_to_degrees(compass_->getValues()); }

private:
  // *** GPS ***

  // *** Compass ***
  Compass* compass_ {nullptr};
  double desired_angle_ {-90.0};

  // *** Odometry ***
  math::vec2<double> pos_;
  double theta_ {}, sl_ {}, sr_ {};
};
