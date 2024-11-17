#pragma once

#include "MyRobot.hpp"

struct Stop {
  void update(MyRobot& robot) { }
  void enter(MyRobot& robot) { }
};

inline std::ostream& operator<<(std::ostream& os, Stop const&) {
  os << "Stop";
  return os;
}
