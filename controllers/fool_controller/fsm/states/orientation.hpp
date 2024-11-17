#pragma once

#include "MyRobot.hpp"
#include "utils/pid.hpp"

class Orientation {
public:
  Orientation() = default;

  void enter(MyRobot const& robot) {
    Pid::Params constexpr params {
      .max = 180,
      .min = -180,
      .kp  = 0.9,
      .kd  = 0.2,
      .ki  = 0.08,
    };

    logger(Log::controller) << "Desired angle: " << robot.compass.desiredAngle();
    pid_ = Pid(params);
  };

  void update(MyRobot const& robot) {
    double const out = pid_.calculate(0, robot.compass.distanceToDesiredAngle());
    double const vel = -out * utils::delta_time * 2;
    logger(Log::controller) << "Pid out: " << out;
    logger(Log::controller) << "Velocity: " << vel;
    robot.motors.rotate(vel);
  };

private:
  Pid pid_;
};


inline std::ostream& operator<<(std::ostream& os, Orientation const&) {
  os << "Orientation";
  return os;
}
