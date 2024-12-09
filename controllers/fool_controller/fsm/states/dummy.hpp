#pragma once

#include "MyRobot.hpp"

struct Dummy {
  constexpr static auto enter = [](MyRobot& robot) {};

  constexpr static auto update = [](MyRobot const& robot) {
    logger(Log::controller) << "Dummy: " << robot.gps.position();
  };
};

inline std::ostream& operator<<(std::ostream& os, Dummy const& Dummy) {
  os << "Dummy";
  return os;
}
