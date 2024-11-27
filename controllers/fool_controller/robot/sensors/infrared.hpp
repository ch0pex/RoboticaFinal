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
    ds0 = 0, // front left
    ds1,
    ds2,
    ds3, // left
    ds4,
    ds5, // right
    ds6,
    ds7,
    ds8,
    ds9,
    ds10,
    ds11,
    ds12,
    ds13,
    ds14,
    ds15, // front right

    none
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
      if (auto const dis = distance(i); dis < sensor_range) return true;
    }
    return false;
  }

  [[nodiscard]] bool detecting(uint8_t const idx, double const error = 0) const {
    return distance(idx) < utils::min_distance - error;
  }

  [[nodiscard]] Sensor minDistanceSensor() const {
    double min      = std::numeric_limits<double>::max();
    std::size_t idx = Sensor::none;
    for (std::size_t i = 0; i < distance_sensors_.size(); ++i) {
      if (auto const dis = distance(i); dis < min) {
        min = dis;
        idx = i;
      }
    }
    return static_cast<Sensor>(idx);
  }

  static Sensor mirror(Sensor const sensor) { return static_cast<Sensor>(15 - sensor); }

  [[nodiscard]] bool frontDetection() const {
    return detecting(ds0) or detecting(ds15) or detecting(ds1, 10) or detecting(ds14, 10) or detecting(ds2, 20) or
           detecting(ds13, 20);
  };

  friend std::ostream& operator<<(std::ostream& os, Infrared const& infrared);

private:
  static constexpr double sensor_range = 40.0;
  std::array<DistanceSensor*, 16> distance_sensors_ {};
};

inline std::ostream& operator<<(std::ostream& os, Infrared const& infrared) {
  os << "Sensor(" << 0 << "): " << infrared.distance(0) << "\n";
  for (size_t i = 1; i < infrared.distance_sensors_.size(); ++i) {
    os << "[Controller]: Sensor(" << i << "): " << infrared.distance(i) << "\n";
  }
  return os;
}

} // namespace sensors
