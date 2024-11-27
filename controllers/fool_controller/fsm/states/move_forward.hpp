#pragma once

#include "MyRobot.hpp"

struct MoveForward {
  static constexpr auto update = [](MyRobot& robot) {
    // logger(Log::controller) << "Front wall percentage: " << robot.cameras.wallInFront();
  };
  static constexpr auto enter = [](MyRobot& robot) { robot.motors.setVelocity(5); };
};

inline std::ostream& operator<<(std::ostream& os, MoveForward const& move) {
  os << "Moving Forward";
  return os;
}
