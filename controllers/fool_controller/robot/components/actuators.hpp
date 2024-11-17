#pragma once

#include "utils/common.hpp"
#include "utils/logger.hpp"

#include <cassert>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>


namespace actuators {

class Motors {
public:
  explicit Motors(webots::Robot& robot) :
    left(robot.getMotor("left wheel motor")), right(robot.getMotor("right wheel motor")) {
    logger(Log::robot) << "Motors initialized";
    left->setPosition(INFINITY);
    right->setPosition(INFINITY);
  }

  ~Motors() { setVelocity(0); }

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

  void rotate(double const speed = utils::max_velocity) const {
    assert(speed < utils::max_velocity);
    left->setVelocity(speed);
    right->setVelocity(-speed);
  }

private:
  // static constexpr auto clip_velocity = [](double const speed) { return speed > max_velocity ? max_velocity :
  // speed;};

  webots::Motor* left;
  webots::Motor* right;
};

} // namespace actuators
