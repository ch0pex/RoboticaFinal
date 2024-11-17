#pragma once

#include "MyRobot.hpp"

struct MoveForward {
  static constexpr auto update = [](MyRobot& robot) {};
  static constexpr auto enter  = [](MyRobot& robot) { robot.motors.setVelocity(utils::max_velocity); };
};

inline std::ostream& operator<<(std::ostream& os, MoveForward const& move) {
  os << "Moving Forward";
  return os;
}
