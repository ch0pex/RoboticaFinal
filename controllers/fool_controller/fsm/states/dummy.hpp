#pragma once

#include "MyRobot.hpp"

struct Dummy {
  constexpr static auto enter = [](MyRobot& robot) {};

  constexpr static auto update = [](MyRobot& robot) {
    logger(Log::controller) << "Is infront: " << robot.cameras.personInFront();
  };
};

inline std::ostream& operator<<(std::ostream& os, Dummy const& Dummy) {
  os << "Dummy";
  return os;
}
