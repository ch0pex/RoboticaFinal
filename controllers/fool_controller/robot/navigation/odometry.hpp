#pragma once

#include "utils/common.hpp"
#include "utils/logger.hpp"
#include "utils/math.hpp"

#include <webots/camera.hpp>
#include <webots/compass.hpp>
#include <webots/gps.hpp>
#include <webots/robot.hpp>

using namespace webots;

namespace navigation {


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


} // namespace navigation
