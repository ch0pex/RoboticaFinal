/************************************************************************
 * @file sensors.hpp
 * @version 1.0
 * @date 14/11/2024
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

class Cameras {
public:
  explicit Cameras(Robot& robot) : front_cam_(robot.getCamera("camera_f")), sphere_cam_(robot.getCamera("camera_s")) {
    logger(Log::robot) << "Cameras initialized";
    front_cam_->enable(utils::time_step);
    sphere_cam_->enable(utils::time_step);
  }

  ~Cameras() {
    front_cam_->disable();
    sphere_cam_->disable();
  }

private:
  Camera* front_cam_;
  Camera* sphere_cam_;
};

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
    return (1024 - ir_val) * 0.2;
  }

  bool wallDetected() {
    for (size_t i = 0; i < distance_sensors_.size(); ++i) {
      if (distance(i) < 1024)
        return true;
    }
    return false;
  }

private:
  std::array<DistanceSensor*, 16> distance_sensors_ {};
};

} // namespace sensors
