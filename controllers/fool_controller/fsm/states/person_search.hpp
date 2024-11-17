#pragma once

#include "MyRobot.hpp"

struct PersonSearch {
  void update(MyRobot& robot) { }
  void enter(MyRobot& robot) { }
};

inline std::ostream& operator<<(std::ostream& os, PersonSearch const&) {
  os << "Person Search";
  return os;
}
