#pragma once

#include "utils/common.hpp"
#include "utils/logger.hpp"
#include "utils/math.hpp"

#include <webots/Camera.hpp>
#include <webots/Compass.hpp>
#include <webots/GPS.hpp>
#include <webots/Robot.hpp>

using namespace webots;

namespace navigation {


class Odometry {
public:
  Odometry() : pos_(0.0) { }

  void compute(double const radians, math::vec2<double> const wheel_sensors) {

    double const sr_meters = utils::wheel_radius * (wheel_sensors.y);
    double const sl_meters = utils::wheel_radius * (wheel_sensors.x);

    double const delta_sr = sr_meters - sr_;
    double const delta_sl = sl_meters - sl_;

    if (std::isnan(delta_sr) || std::isnan(delta_sl) || std::isnan(radians)) {
      logger(Log::debug) << "Valor inválido detectado antes del cálculo.\n";
    }

    pos_.x +=
        ((delta_sr + delta_sl) / 2.0) * (std::cos(radians + ((delta_sr - delta_sl) / (2.0 * utils::wheel_distance))));
    pos_.y +=
        ((delta_sr + delta_sl) / 2.0) * (std::sin(radians + ((delta_sr - delta_sl) / (2.0 * utils::wheel_distance))));

    theta_ = radians;

    sr_ = sr_meters;
    sl_ = sl_meters;
  }

  void reset() {
    pos_.x = 0.0;
    pos_.y = 0.0;
    theta_ = 0.0;
  }

  [[nodiscard]] math::vec2<double> getPos() const { return pos_; }

private:
  math::vec2<double> pos_;
  double theta_ {0.0}, sl_ {0.0}, sr_ {0.0};
};


} // namespace navigation
