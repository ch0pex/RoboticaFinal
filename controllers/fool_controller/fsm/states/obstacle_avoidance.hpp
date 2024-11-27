#pragma once

#include "MyRobot.hpp"
#include "utils/pid.hpp"

#include <random>

enum class FollowDir { left = 0, right };

inline auto follow_side = FollowDir::right;

class ObstacleAvoidance {
public:
  using ir = sensors::Infrared::Sensor;

  void enter(MyRobot const& robot) {
    Pid::Params constexpr params {
      .max = 10,
      .min = 0,
      .kp  = 0.9,
      .kd  = 0.2,
      .ki  = 0.08,
    };
    // pid_ = Pid(params);
    robot.motors.setVelocity(0);

    follow_side = setSideToFollow(robot);
  }

  void update(MyRobot const& robot) {
    auto const side_str = (follow_side == FollowDir::right) ? "right" : "left";
    logger(Log::controller) << side_str;
    logger(Log::controller) << robot.ir_sensors;
    if (follow_side == FollowDir::left) {
      wallFollowerLeft(robot);
    }
    else {
      wallFollowerRight(robot);
    }
  }

private:
  FollowDir setSideToFollow(MyRobot const& robot) {

    // if (not robot.ir_sensors.detecting(ir::ds2)) {
    // return FollowDir::right;
    // }

    // if (not robot.ir_sensors.detecting(ir::ds13)) {
    // return FollowDir::left;
    // }

    // if (robot.ir_sensors.detecting(ir::ds3) and not robot.ir_sensors.detecting(ir::ds12)) {
    // return follow_side;
    // }

    // if (robot.ir_sensors.detecting(ir::ds12)) {
    // return follow_side;
    // }
    auto const nearest_sensor = robot.ir_sensors.minDistanceSensor();
    auto const mirror         = robot.ir_sensors.mirror(nearest_sensor);

    if (not robot.ir_sensors.detecting(mirror)) {
      return mirror < 8 ? FollowDir::right : FollowDir::left;
    }

    return follow_side == FollowDir::right ? FollowDir::left : FollowDir::right;
  }

  void wallFollowerLeft(MyRobot const& robot) {
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
        left_speed += 2.0F;
        right_speed -= 2.0F;
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

  void wallFollowerRight(MyRobot const& robot) {
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
        left_speed -= 2.0F;
        right_speed += 2.0F;
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

  // Pid pid_;
};

inline std::ostream& operator<<(std::ostream& os, ObstacleAvoidance const&) {
  os << "Obstacle Avoidance";
  return os;
}
