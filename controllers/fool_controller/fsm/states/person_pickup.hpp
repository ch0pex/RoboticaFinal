#pragma once

#include "MyRobot.hpp"

struct PersonPickUp {
  void update(MyRobot& robot) { }
  void enter(MyRobot& robot) { }
};

inline std::ostream& operator<<(std::ostream& os, PersonPickUp const&) {
  os << "Person Pick Up";
  return os;
}
