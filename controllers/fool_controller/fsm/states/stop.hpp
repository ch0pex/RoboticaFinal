#pragma once

#include "MyRobot.hpp"

struct Stop {
  static constexpr auto enter = [](MyRobot& robot) { robot.motors.setVelocity(0); };

  static constexpr auto update = [](MyRobot& robot) { logger(Log::controller) << "Mission completed"; };
};

inline std::ostream& operator<<(std::ostream& os, Stop const&) {
  os << "Stop";
  return os;
}
