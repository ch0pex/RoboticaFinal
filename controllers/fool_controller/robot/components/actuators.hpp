#pragma once

#include "utils/common.hpp"

#include <cassert>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>

using namespace webots;

class Actuators {
public:
  explicit Actuators(Robot* robot) :
    left(robot->getMotor("left wheel motor")), right(robot->getMotor("right wheel motor")) {
    left->setPosition(INFINITY);
    right->setPosition(INFINITY);
  }

  template<utils::Direction motor>
  void setVelocity(double const speed) const {
    assert(speed < utils::max_velocity);
    if constexpr (motor == utils::Direction::left) {
      left->setVelocity(speed);
    }
    else {
      right->setVelocity(speed);
    }
  }

  void setVelocity(double const speed) const {
    assert(speed < utils::max_velocity);
    left->setVelocity(speed);
    right->setVelocity(speed);
  }

  template<utils::Direction Dir>
  void rotate(double const speed) const {
    assert(speed < utils::max_velocity);
    if constexpr (Dir == utils::Direction::left) {
      left->setVelocity(-speed);
      right->setVelocity(speed);
    }
    else {
      left->setVelocity(speed);
      right->setVelocity(-speed);
    }
  }

private:
  // static constexpr auto clip_velocity = [](double const speed) { return speed > max_velocity ? max_velocity : speed;
  // };

  Motor* left;
  Motor* right;
};
