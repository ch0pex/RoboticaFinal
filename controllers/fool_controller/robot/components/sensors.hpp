/************************************************************************
 * Copyright (c) 2024 Alvaro Cabrera Barrio
 * This code is licensed under MIT license (see LICENSE.txt for details)
 ************************************************************************/
/**
 * @file sensors.hpp
 * @version 1.0
 * @date 14/11/2024
 * @brief Short description
 *
 * Longer description
 */

#pragma once

#include "utils/math.hpp"

#include <webots/DistanceSensor.hpp>
#include <webots/Robot.hpp>

#include <array>
#include <execution>

namespace sensors {

class Cameras {
public:
  explicit Cameras(webots::Robot& robot) :
    front_cam_(robot.getCamera("camera_f")), sphere_cam_(robot.getCamera("camera_s")) {
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
  explicit Infrared(webots::Robot& robot) {
    for (size_t ind = 0; ind < distance_sensor_.size(); ++ind) {
      std::string sensor_name = std::string("ds") + std::to_string(ind);
      std::cout << "Initializing distance sensor: " << sensor_name << "\n";
      distance_sensor_.at(ind) = robot.getDistanceSensor(sensor_name);
      distance_sensor_.at(ind)->enable(utils::time_step);
    }
  }

  ~Infrared() {
    for (auto* sensor: distance_sensor_) {
      sensor->disable();
    }
  }

  template<uint8_t idx>
  constexpr double distance() {
    return math::ir_to_distance(distance_sensor_.at(idx)->getValue());
  }

  template<uint8_t idx>
  constexpr double distance2() {
    return math::ir_to_distance(distance_sensor_[idx]->getValue());
  }

  double distance(uint8_t idx) { return math::ir_to_distance(distance_sensor_.at(idx)->getValue()); }

  double distance2(uint8_t idx) { return math::ir_to_distance(distance_sensor_[idx]->getValue()); }

private:
  std::array<DistanceSensor*, 16> distance_sensor_ {};
};

} // namespace sensors
