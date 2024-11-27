#pragma once

#include "MyRobot.hpp"
#include "utils/pid.hpp"

class ObstacleAvoidance {
public:
  void enter(MyRobot& robot) {
    Pid::Params constexpr params {
      .max = 10,
      .min = 0,
      .kp  = 0.9,
      .kd  = 0.2,
      .ki  = 0.08,
    };
    pid_ = Pid(params);
    robot.motors.setVelocity(0);
  }

  void update(MyRobot& robot) {
    double left_speed  = 0;
    double right_speed = 0;
    auto const& ir     = robot.ir_sensors;
    using sensor       = sensors::Infrared::Sensor;


    //    logger(Log::controller) << ir.distance(sensor::front_left) << ", " << ir.distance(sensor::front_right);
    //    logger(Log::controller) << ir.distance(sensor::left);
    //
    //        auto out = pid_.calculate(utils::min_distance, ir.minDistance());
    //
    if (ir.distance(sensor::left) < utils::min_distance) {
      if (ir.frontDetection()) {
        left_speed += 2.0f;
        right_speed -= 2.0f;
      }
      else {
        left_speed += 5.0f;
        right_speed += 5.0f;
      }
    }
    else {
      if (ir.frontDetection()) {
        left_speed += 2.0f;
        right_speed -= 2.0f;
      }
      else {
        right_speed += 5.0f;
      }
    }

    if (robot.ir_sensors.distance(2) > utils::min_distance) {
      right_speed += 1;
    }
    else if (robot.ir_sensors.distance(2) <= utils::min_distance) {
      right_speed -= 1.5;
    }

    robot.motors.setVelocity({left_speed, right_speed});
    //        logger(Log::controller) << "Pid output: " << out;
  }

private:
  Pid pid_;
};

inline std::ostream& operator<<(std::ostream& os, ObstacleAvoidance const&) {
  os << "Obstacle Avoidance";
  return os;
}
