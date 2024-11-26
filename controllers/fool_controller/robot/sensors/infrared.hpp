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

#include "utils/logger.hpp"
#include "utils/math.hpp"

#include <webots/DistanceSensor.hpp>
#include <webots/Robot.hpp>

#include <array>
#include <execution>

namespace sensors {

class Infrared {
public:
  explicit Infrared(Robot& robot) {
    for (size_t ind = 0; ind < distance_sensors_.size(); ++ind) {
      std::string sensor_name = std::string("ds") + std::to_string(ind);
      logger(Log::robot) << "Distance sensor " << sensor_name << " initialized";
      distance_sensors_.at(ind) = robot.getDistanceSensor(sensor_name);
      distance_sensors_.at(ind)->enable(utils::time_step);
    }
  }

  enum Sensor {
    front_left  = 0,
    left        = 3,
    right       = 5,
    front_right = 15,

  };

  ~Infrared() {
    for (auto* sensor: distance_sensors_) {
      sensor->disable();
    }
  }

  [[nodiscard]] double distance(uint8_t const idx) const {
    auto ir_val = distance_sensors_.at(idx)->getValue();
    return (1024 - ir_val) * 0.04;
  }

  [[nodiscard]] auto sensorsDetectingWall() const {
    std::vector<std::uint8_t> sensor_indices;
    for (std::size_t i = 0; i < distance_sensors_.size(); ++i) {
      auto const dis = distance(i);
      if (dis < sensor_range) sensor_indices.push_back(i);
    }
    return sensor_indices;
  }

  [[nodiscard]] bool wallDetected() const {
    for (std::size_t i = 0; i < distance_sensors_.size(); ++i) {
      auto dis = distance(i);
      if (dis < sensor_range) return true;
    }
    return false;
  }

  double minDistance() const {
    double min = std::numeric_limits<double>::max();
    for (std::size_t i = 0; i < distance_sensors_.size(); ++i) {
      auto const dis = distance(i);
      min            = min > dis ? dis : min;
    }
    return min;
  }

  friend std::ostream& operator<<(std::ostream& os, Infrared& infrared);

private:
  static constexpr double sensor_range = 40.0;
  std::array<DistanceSensor*, 16> distance_sensors_ {};
};

inline std::ostream& operator<<(std::ostream& os, Infrared& infrared) {
  for (size_t i = 0; i < infrared.distance_sensors_.size(); ++i) {
    os << "Sensor(" << i << "): " << infrared.distance(i) << "\n";
  }
}

} // namespace sensors
