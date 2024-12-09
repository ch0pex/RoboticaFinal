#pragma once

#include "MyRobot.hpp"

class PersonSearch {
public:
  static void enter(MyRobot& robot) {
    //    Pid::Params constexpr params {
    //      .max = 10,
    //      .min = 0,
    //      .kp  = 0.9,
    //      .kd  = 0.2,
    //      .ki  = 0.08,
    //    };
    //     pid_ = Pid(params);
    robot.motors.setVelocity(0);
    robot.super_person_search = true;
    follow_side               = setSideToFollow(robot, follow_side);
  }

  static void update(MyRobot& robot) {
    auto const* side_str = (follow_side == FollowDir::right) ? "right" : "left";
    logger(Log::controller) << side_str;
    logger(Log::controller) << robot.ir_sensors;

    if (follow_side == FollowDir::left) {
      robot.compass.desiredAngle(0);
      wallFollowerLeft(robot);
    }
    else {
      robot.compass.desiredAngle(180);
      wallFollowerRight(robot);
    }
  }

private:
  static inline auto follow_side = FollowDir::right;
};

inline std::ostream& operator<<(std::ostream& os, PersonSearch const&) {
  os << "Person Search";
  return os;
}
