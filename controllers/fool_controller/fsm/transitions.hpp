#pragma once

#include "states/dummy.hpp"
#include "states/localization.hpp"
#include "states/move_forward.hpp"
#include "states/obstacle_avoidance.hpp"
#include "states/orientation.hpp"
#include "states/person_pickup.hpp"
#include "states/person_search.hpp"
#include "states/stop.hpp"

#include <optional>
#include <tuple>
#include <variant>

using state_variant =
    std::variant<Localization, MoveForward, ObstacleAvoidance, Orientation, PersonPickUp, PersonSearch, Stop, Dummy>;

inline std::optional<state_variant> transition([[maybe_unused]] Localization& state, MyRobot& robot) {
  if (robot.compass.isFacingDesired()) return MoveForward {};
  return Orientation {};
}

inline std::optional<state_variant> transition([[maybe_unused]] Orientation& state, MyRobot& robot) {
  if (robot.ir_sensors.frontDetection()) {
    if (robot.people_positions.size() > 0) {
      return PersonSearch {}; // TODO this is a fix if we have time implement hfsm
    }
    return ObstacleAvoidance {};
  }
  if (robot.compass.isFacingDesired()) {
    if (robot.people_positions.size() > 0) {
      return PersonSearch {}; // TODO this is a fix if we have time implement hfsm
    }
    return MoveForward {};
  }
  return std::nullopt;
}

inline std::optional<state_variant> transition([[maybe_unused]] MoveForward& state, MyRobot& robot) {
  if (robot.ir_sensors.frontDetection()) {
    if (robot.people_positions.size() > 0) {
      return PersonSearch {}; // TODO this is a fix if we have time implement hfsm
    }
    return ObstacleAvoidance {};
  }
  return std::nullopt;
}

inline std::optional<state_variant> transition([[maybe_unused]] ObstacleAvoidance& state, MyRobot& robot) {
  if (robot.ir_sensors.frontDetection() and robot.cameras.personInFront() and not robot.personAlreadySaved()) {
    return PersonPickUp {};
  }
  if (robot.compass.isFacingDesired(10) and not robot.ir_sensors.frontDetection()) return Orientation {};
  return std::nullopt;
}

inline std::optional<state_variant> transition([[maybe_unused]] PersonPickUp& state, MyRobot& robot) {
  if (abs(robot.motors.velocity().x) < 5 and robot.compass.isFacingDesired(10)) {
    robot.people_positions.push_back(robot.odometry.getPos());
    return PersonSearch {};
  }
  return std::nullopt;
}

inline std::optional<state_variant> transition([[maybe_unused]] PersonSearch& state, MyRobot& robot) {
  if (robot.ir_sensors.frontDetection() and robot.cameras.personInFront() and not robot.personAlreadySaved()) {
    return PersonPickUp {};
  }
  if (robot.compass.isFacingDesired(10) and not robot.ir_sensors.frontDetection()) return Orientation {};
  return std::nullopt;
}

inline std::optional<state_variant> transition([[maybe_unused]] Stop& state, MyRobot& robot) { return std::nullopt; }

inline std::optional<state_variant> transition([[maybe_unused]] Dummy& state, MyRobot& robot) { return std::nullopt; }
