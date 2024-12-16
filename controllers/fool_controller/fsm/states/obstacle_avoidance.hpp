#pragma once

#include "fsm/common/wall_follower.hpp"
#include "utils/pid.hpp"

#include <random>

class ObstacleAvoidance {
public:
  static void enter(MyRobot& robot) {
    robot.motors.setVelocity(0);
    if (robot.people_positions.size() == 2 or robot.time_searching > utils::max_time_searching) {
      robot.compass.desiredAngle(270);
    }
    follow_side = setSideToFollow<true>(robot, follow_side);
  }

  static void update(MyRobot const& robot) {
    if (follow_side == FollowDir::left) {
      wallFollowerLeft(robot);
    }
    else {
      wallFollowerRight(robot);
    }
  }

private:
  static inline auto follow_side = FollowDir::right;
};

inline std::ostream& operator<<(std::ostream& os, ObstacleAvoidance const&) {
  os << "Obstacle Avoidance";
  return os;
}
