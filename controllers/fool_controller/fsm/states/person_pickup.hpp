#pragma once

#include "MyRobot.hpp"

class PersonPickUp {
public:
  void enter(MyRobot& robot) {
    Pid::Params params {
      .max = 180,
      .min = -180,
      .kp  = 0.9,
      .kd  = 0.2,
      .ki  = 0.08,
    };
    pid_ = Pid(params);

    robot.compass.desiredAngle(robot.compass.facingAngle());
    robot.motors.rotate(-5);
  };

  void update(MyRobot& robot) {
    auto const diff = robot.compass.facingAngle() - robot.compass.desiredAngle();

    if (diff > math::Angle {180}) {
      robot.motors.rotate(-5);
      return;
    }

    double const out = pid_.calculate(5, diff.degrees());
    double const vel = out * utils::delta_time * 5;

    logger(Log::controller) << "Desired angle: " << robot.compass.desiredAngle();
    logger(Log::controller) << "Rotation velocity: " << vel;
    logger(Log::controller) << "Facing angle: " << robot.compass.facingAngle();
    logger(Log::controller) << "Distance to angle: " << diff;
    logger(Log::controller) << "People found: " << robot.people_positions.size();
    robot.motors.rotate(vel);
  }

private:
  Pid pid_;
};

inline std::ostream& operator<<(std::ostream& os, PersonPickUp const&) {
  os << "Person Pick Up";
  return os;
}
