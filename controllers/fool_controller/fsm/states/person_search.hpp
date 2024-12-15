#pragma once

#include "MyRobot.hpp"

#include "fsm/common/wall_follower.hpp"

class PersonSearch {
public:
  static void enter(MyRobot& robot) {

    robot.motors.setVelocity(0);
    robot.super_person_search = true;
    follow_side               = setSideToFollow<false>(robot, follow_side);
  }

  static void update(MyRobot& robot) {
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
