/************************************************************************
 * Copyright (c) 2024 Alvaro Cabrera Barrio
 * This code is licensed under MIT license (see LICENSE.txt for details)
 ************************************************************************/
/**
 * @file compass.h
 * @version 1.0
 * @date 26/11/2024
 * @brief Short description
 *
 * Longer description
 */

#pragma once

#pragma once

#include "utils/common.hpp"
#include "utils/logger.hpp"
#include "utils/math.hpp"

#include <webots/camera.hpp>
#include <webots/compass.hpp>
#include <webots/gps.hpp>
#include <webots/robot.hpp>

using namespace webots;

namespace sensors {

class Compass {
public:
  // *** Constructors ***
  explicit Compass(Robot& robot) : compass_(robot.getCompass("compass")) {
    logger(Log::robot) << "Compass initialized";
    compass_->enable(utils::time_step);
  }

  ~Compass() { compass_->disable(); }

  constexpr void desiredAngle(double const angle) { desired_angle_ = math::Angle {angle}; }

  constexpr void desiredAngle(math::Angle const angle) { desired_angle_ = angle; }

  [[nodiscard]] constexpr math::Angle desiredAngle() const { return desired_angle_; }

  [[nodiscard]] math::Angle facingAngle() const { return math::Angle {compass_->getValues()}; }

  [[nodiscard]] double facingRadians() const { return atan2(compass_->getValues()[0], compass_->getValues()[2]); }

  [[nodiscard]] bool isFacingDesired(double const error = 0.1) const {
    return facingAngle().diff(desiredAngle()) < error;
  }

  [[nodiscard]] double distanceToDesiredAngle() const { return facingAngle().signedDiff(desiredAngle()); }

private:
  // *** Compass ***
  webots::Compass* compass_ {nullptr};
  math::Angle desired_angle_ {90.0};
};

} // namespace sensors
