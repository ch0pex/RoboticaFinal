#pragma once

#include "MyRobot.hpp"

struct Localization {
  constexpr static auto enter = [](MyRobot& robot) {
    //    logger(Log::controller) << robot.ir_sensors;
    auto const gps_pos = robot.gps.position();
    robot.compass.desiredAngle(gps_pos.y < 0 ? 90 : 270);
  };

  constexpr static auto update = [](MyRobot& robot) { logger(Log::debug) << robot.compass.facingAngle(); };
};

inline std::ostream& operator<<(std::ostream& os, Localization const& localization) {
  os << "Localization";
  return os;
}
