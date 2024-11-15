#pragma once

#include "utils/common.hpp"
#include "utils/math.hpp"

#include <webots/Camera.hpp>
#include <webots/Compass.hpp>
#include <webots/GPS.hpp>
#include <webots/Robot.hpp>

using namespace webots;

namespace navigation {

class Compass {
public:
  // *** Constructors ***
  explicit Compass(Robot& robot) : compass_(robot.getCompass("compass")) { compass_->enable(utils::time_step); }

  ~Compass()() { compass_->disable(); }

  void setDesiredAngle(double const angle) { desired_angle_ = angle; }

  [[nodiscard]] double facingAngle() const { return math::convert_bearing_to_degrees(compass_->getValues()); }

private:
  // *** Compass ***
  webots::Compass* compass_ {nullptr};
  double desired_angle_ {-90.0};
};

class Odometry {
public:
  Odometry() = default;

  void compute() {
    pos_.x += ((sr_ + sl_) / 2 * sin(theta_ + ((sr_ - sl_) / (2 * utils::wheel_distance))));
    pos_.y += ((sr_ + sl_) / 2 * cos(theta_ + ((sr_ - sl_) / (2 * utils::wheel_distance))));
    theta_ = theta_ + ((sr_ - sl_) / utils::wheel_distance);
  }

  void reset() {
    pos_.x = 0.0;
    pos_.y = 0.0;
    theta_ = 0.0;
  }

  [[nodiscard]] math::vec2<double> getPos() const { return pos_; }

private:
  math::vec2<double> pos_;
  double theta_ {}, sl_ {}, sr_ {};
};

class Gps {
public:
  explicit Gps(Robot& robot) : gps_(robot.getGPS("gps")) { gps_->enable(utils::time_step); }

  Gps() = default;

  ~Gps() { gps_->disable(); }

  [[nodiscard]] math::vec2<double> position() const {
    auto const* const values = gps_->getValues();
    return {values[0], values[2]}; // maybe 0, 2?
  };

  [[nodiscard]] double speed() const { return gps_->getSpeed(); }

private:
  GPS* gps_;
};


} // namespace navigation
