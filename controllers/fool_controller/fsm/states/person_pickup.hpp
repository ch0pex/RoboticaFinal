#pragma once

#include "MyRobot.hpp"

class PersonPickUp {
public:
  void update(MyRobot& robot) {
    auto diff = robot.compass.facingAngle() - robot.compass.desiredAngle();

    if (diff > math::Angle {180}) {
      robot.motors.rotate(5);
      return;
    }

    double const out = pid_.calculate(0, diff.degrees());
    double const vel = -out * utils::delta_time * 5;
    logger(Log::controller) << "Rotation velocity: " << vel;
    logger(Log::controller) << "Facing angle: " << robot.compass.facingAngle();
    logger(Log::controller) << "Distance to angle: " << robot.compass.distanceToDesiredAngle();
    // logger(Log::controller) << "Front wall percentage: " << robot.cameras.wallInFront();
    robot.motors.setVelocity({vel, -vel});
  }

  static constexpr auto enter = [](MyRobot& robot) {
    robot.compass.desiredAngle(robot.compass.facingAngle().degrees());
    robot.motors.rotate(5);
  };

private:
  Pid pid_;
};

inline std::ostream& operator<<(std::ostream& os, PersonPickUp const&) {
  os << "Person Pick Up";
  return os;
}
