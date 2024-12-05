#pragma once

#include "MyRobot.hpp"

class PersonSearch {
public:
  static void enter(MyRobot const& robot) {
    //    Pid::Params constexpr params {
    //      .max = 10,
    //      .min = 0,
    //      .kp  = 0.9,
    //      .kd  = 0.2,
    //      .ki  = 0.08,
    //    };
    //     pid_ = Pid(params);
    robot.motors.setVelocity(0);
    follow_side = setSideToFollow(robot, follow_side);
  }

  static void update(MyRobot& robot) {
    auto const* side_str = (follow_side == FollowDir::right) ? "right" : "left";
    logger(Log::controller) << side_str;
    logger(Log::controller) << robot.ir_sensors;

    // If nothing is in front and we see the person go for him
    if (not robot.ir_sensors.frontDetection() and robot.cameras.personInSight()) {
      if (utils::Direction const dir = robot.cameras.personInSide(); dir == utils::Direction::right) {
        robot.motors.setVelocity({3, 2});
      }
      else if (dir == utils::Direction::left) {
        robot.motors.setVelocity({2, 3});
      }
      return;
    }

    if (follow_side == FollowDir::left) {
      robot.compass.desiredAngle(0);
      wallFollowerLeft(robot);
    }
    else {
      wallFollowerRight(robot);
      robot.compass.desiredAngle(180);
    }
  }

private:
  static inline auto follow_side = FollowDir::left;
};

inline std::ostream& operator<<(std::ostream& os, PersonSearch const&) {
  os << "Person Search";
  return os;
}
