#pragma once

#include "MyRobot.hpp"

struct Localization {
  constexpr static auto update = [](MyRobot& robot) { logger(Log::debug) << robot.compass.facingAngle(); };

  constexpr static auto enter = [](MyRobot& robot) {
    auto const gpu_pos = robot.gps.position();
    robot.compass.desiredAngle(gpu_pos.y < 0 ? 90 : 270);
  };
};

inline std::ostream& operator<<(std::ostream& os, Localization const& localization) {
  os << "Localization";
  return os;
}
