#pragma once

#include "utils/common.hpp"
#include "utils/logger.hpp"
#include "utils/math.hpp"

#include <cassert>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Robot.hpp>

namespace actuators {

class Motors {
public:
  explicit Motors(webots::Robot& robot) :
    left_(robot.getMotor("left wheel motor")), right_(robot.getMotor("right wheel motor")),
    left_sensor_(robot.getPositionSensor("left wheel sensor")),
    right_sensor_(robot.getPositionSensor("right wheel sensor")) {

    right_sensor_->enable(utils::time_step);
    left_sensor_->enable(utils::time_step);
    left_->setPosition(INFINITY);
    right_->setPosition(INFINITY);
    setVelocity(0);
    logger(Log::robot) << "Motors initialized";
  }

  ~Motors() {
    right_sensor_->disable();
    left_sensor_->disable();
    setVelocity(0);
  }

  void setVelocity(math::vec2<double> const vel) const {
    left_->setVelocity(clamp(vel.x));
    right_->setVelocity(clamp(vel.y));
  }

  void setVelocity(double const speed) const {
    auto const vel = clamp(speed);
    left_->setVelocity(vel);
    right_->setVelocity(vel);
  }

  void rotate(double const speed = utils::max_velocity) const {
    auto const vel = clamp(speed);
    left_->setVelocity(vel);
    right_->setVelocity(-vel);
  }

  [[nodiscard]] math::vec2<double> velocity() const { return {left_->getVelocity(), right_->getVelocity()}; }

  [[nodiscard]] math::vec2<double> positionSensors() const {
    return {left_sensor_->getValue(), right_sensor_->getValue()};
  }


private:
  static constexpr auto clamp = [](double const speed) {
    if (speed < -utils::max_velocity) return -utils::max_velocity;
    return speed > utils::max_velocity ? utils::max_velocity : speed;
  };

  webots::Motor* left_;
  webots::Motor* right_;

  webots::PositionSensor* left_sensor_;
  webots::PositionSensor* right_sensor_;
};

} // namespace actuators
