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
    pid_ = Pid(params);
  };

  void update(MyRobot const& robot) {
    double const dis = robot.compass.distanceToDesiredAngle();

    if (dis > 160 or dis < -160) {
      robot.motors.rotate(5);
      return;
    }

    double const out = pid_.calculate(0, dis);
    double const vel = -out * utils::delta_time * 5;
    logger(Log::controller) << "Rotation velocity: " << vel;
    logger(Log::controller) << "Facing angle: " << robot.compass.facingAngle();
    logger(Log::controller) << "Distance to angle: " << robot.compass.distanceToDesiredAngle();
    robot.motors.setVelocity({vel + 4, -vel + 4});
  };

private:
  Pid pid_;
};


inline std::ostream& operator<<(std::ostream& os, Orientation const&) {
  os << "Orientation";
  return os;
}
