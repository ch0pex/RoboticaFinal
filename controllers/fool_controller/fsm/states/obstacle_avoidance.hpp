#pragma once

#include "MyRobot.hpp"

struct ObstacleAvoidance {
  void enter(MyRobot& robot) { }

  void update(MyRobot& robot) {
    double left_speed  = 0;
    double right_speed = 0;
    auto const& ir     = robot.ir_sensors;
    using sensor       = sensors::Infrared::Sensor;

    auto front_detection = [&ir]() {
      return ir.distance(sensor::front_left) < utils::min_distance or
             ir.distance(sensor::front_right) < utils::min_distance;
    };

    if (ir.distance(sensor::left) < utils::min_distance) {
      if (front_detection()) {
        left_speed += 6.0f;
        right_speed -= 8.0f;
      }
      else {
        left_speed += 5.0f;
        right_speed += 5.0f;
      }
    }
    else {
      if (front_detection()) {
        left_speed += 6.0f;
        right_speed -= 6.0f;
      }
      else {
        //                left_speed -= 0.0f;
        right_speed += 6.0f;
      }
    }

    //    if (distance_sensors_[2]->getValue() > utils::min_distance) {
    //      right_speed -= 0.5;
    //    }
    //    else if (distance_sensors_[2]->getValue() <= utils::min_distance) {
    //      right_speed += 0.5;
    //    }

    robot.motors.setVelocity({left_speed, right_speed});
  }
};

inline std::ostream& operator<<(std::ostream& os, ObstacleAvoidance const&) {
  os << "Obstacle Avoidance";
  return os;
}
