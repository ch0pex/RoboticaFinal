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

namespace navigation {

class Gps {
public:
  explicit Gps(Robot& robot) : gps_(robot.getGPS("gps")) {
    logger(Log::robot) << "Gps initialized";
    gps_->enable(utils::time_step);
  }

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
