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

    auto front_detection = [&ir]() {
      return ir.distance(sensor::front_left) < utils::min_distance or
             ir.distance(sensor::front_right) < utils::min_distance;
    };

    //    logger(Log::controller) << ir.distance(sensor::front_left) << ", " << ir.distance(sensor::front_right);
    //    logger(Log::controller) << ir.distance(sensor::left);
    //
    //        auto out = pid_.calculate(utils::min_distance, ir.minDistance());
    //
    if (ir.distance(sensor::left) < utils::min_distance) {
      if (front_detection()) {
        left_speed += 2.0f;
        right_speed -= 2.0f;
      }
      else {
        left_speed += 2.0f;
        right_speed += 2.0f;
      }
    }
    else {
      if (front_detection()) {
        left_speed += 2.0f;
        right_speed -= 2.0f;
      }
      else {
        right_speed += 2.0f;
      }
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
