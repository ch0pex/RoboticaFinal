#pragma once

#include "MyRobot.hpp"

struct PersonSearch {
  void enter(MyRobot& robot) { }
  void update(MyRobot& robot) { }
};

inline std::ostream& operator<<(std::ostream& os, PersonSearch const&) {
  os << "Person Search";
  return os;
}
