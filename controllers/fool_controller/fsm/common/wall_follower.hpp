/************************************************************************
 * Copyright (c) 2024 Alvaro Cabrera Barrio
 * This code is licensed under MIT license (see LICENSE.txt for details)
 ************************************************************************/
/**
 * @file wall_follower.h
 * @version 1.0
 * @date 03/12/2024
 * @brief Short description
 *
 * Longer description
 */

#pragma once

#include "MyRobot.hpp"

#include <bits/random.h>

static inline std::random_device rd;
static inline std::mt19937 gen(rd());

enum class FollowDir : std::uint8_t { left = 0, right };

template<bool random = false>
FollowDir setSideToFollow(MyRobot const& robot, FollowDir const current_dir) {
  using ir                  = sensors::Infrared::Sensor;
  auto const nearest_sensor = robot.ir_sensors.minDistanceSensor();

  if (robot.ir_sensors.detecting(ir::ds3)) {
    return FollowDir::left;
  }

  if (robot.ir_sensors.detecting(ir::ds12)) {
    return FollowDir::right;
  }

  if (auto const mirror = sensors::Infrared::mirror(nearest_sensor); not robot.ir_sensors.detecting(mirror)) {
    return mirror < 8 ? FollowDir::right : FollowDir::left;
  }

  if constexpr (random) {
    return static_cast<FollowDir>(std::uniform_int_distribution<std::uint8_t>(0, 1)(gen));
  }
  else {
    if (robot.compass.desiredAngle() == math::Angle {0}) {
      return FollowDir::left;
    }
    return current_dir == FollowDir::right ? FollowDir::left : FollowDir::right;
  }
}

inline void wallFollowerLeft(MyRobot const& robot) {
  using ir           = sensors::Infrared::Sensor;
  double left_speed  = 0;
  double right_speed = 0;

  if (auto const& ir = robot.ir_sensors; ir.distance(ir::ds3) < utils::min_distance) {
    if (ir.frontDetection()) {
      left_speed += 1.5F;
      right_speed -= 1.5F;
    }
    else {
      left_speed += 5.0F;
      right_speed += 5.0F;
    }
  }
  else {
    if (ir.frontDetection()) {

      left_speed += 1.5F;
      right_speed -= 1.5F;
    }
    else {
      right_speed += 2.0F;
    }
  }

  if (robot.ir_sensors.distance(2) > utils::min_distance) {
    right_speed += 1;
  }
  else if (robot.ir_sensors.distance(2) <= utils::min_distance) {
    right_speed -= 1.5;
  }

  robot.motors.setVelocity({left_speed, right_speed});
}

inline void wallFollowerRight(MyRobot const& robot) {
  double left_speed  = 0;
  double right_speed = 0;
  auto const& ir     = robot.ir_sensors;
  using sensor       = sensors::Infrared::Sensor;

  if (ir.distance(sensor::ds12) < utils::min_distance) {
    if (ir.frontDetection()) {
      left_speed -= 1.5F;
      right_speed += 1.5F;
    }
    else {
      left_speed += 5.0F;
      right_speed += 5.0F;
    }
  }
  else {
    if (ir.frontDetection()) {
      left_speed -= 1.5F;
      right_speed += 1.5F;
    }
    else {
      left_speed += 2.0F;
    }
  }

  if (robot.ir_sensors.distance(sensor::ds13) > utils::min_distance) {
    left_speed += 1;
  }
  else if (robot.ir_sensors.distance(sensor::ds13) <= utils::min_distance) {
    left_speed -= 1.5;
  }

  robot.motors.setVelocity({left_speed, right_speed});
}
