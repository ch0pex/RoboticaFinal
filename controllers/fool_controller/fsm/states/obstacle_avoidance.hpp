#pragma once

#include "MyRobot.hpp"

struct ObstacleAvoidance {
  void update(MyRobot& robot) { }
  void enter(MyRobot& robot) { }
};

inline std::ostream& operator<<(std::ostream& os, ObstacleAvoidance const&) {
  os << "Obstacle Avoidance";
  return os;
}
