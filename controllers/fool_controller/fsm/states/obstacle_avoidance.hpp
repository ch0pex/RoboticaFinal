#pragma once

#include "fsm/common/wall_follower.hpp"
#include "utils/pid.hpp"

#include <random>

class ObstacleAvoidance {
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

  static void update(MyRobot const& robot) {
    auto const* side_str = (follow_side == FollowDir::right) ? "right" : "left";
    logger(Log::controller) << side_str;
    logger(Log::controller) << robot.ir_sensors;
    if (follow_side == FollowDir::left) {
      wallFollowerLeft(robot);
    }
    else {
      wallFollowerRight(robot);
    }
  }

private:
  static inline auto follow_side = FollowDir::right;
  // Pid pid_;
};

inline std::ostream& operator<<(std::ostream& os, ObstacleAvoidance const&) {
  os << "Obstacle Avoidance";
  return os;
}
