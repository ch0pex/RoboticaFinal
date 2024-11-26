#pragma once

#include "utils/common.hpp"
#include "utils/logger.hpp"
#include "utils/math.hpp"

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
    setVelocity(0);
  }

  ~Motors() { setVelocity(0); }

  void setVelocity(math::vec2<double> const vel) const {
    left->setVelocity(clamp(vel.x));
    right->setVelocity(clamp(vel.y));
  }

  void setVelocity(double const speed) const {
    auto vel = clamp(speed);
    left->setVelocity(vel);
    right->setVelocity(vel);
  }

  void rotate(double const speed = utils::max_velocity) const {
    auto vel = clamp(speed);
    left->setVelocity(vel);
    right->setVelocity(-vel);
  }

  math::vec2<double> velocity() { return {left->getVelocity(), right->getVelocity()}; }

private:
  static constexpr auto clamp = [](double const speed) {
    if (speed < -utils::max_velocity) return -utils::max_velocity;
    return speed > utils::max_velocity ? utils::max_velocity : speed;
  };

  webots::Motor* left;
  webots::Motor* right;
};

} // namespace actuators
