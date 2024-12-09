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
  if (robot.people_positions.size() == 2 and robot.gps.position().y < 0 and robot.cameras.overpassedYellowLine()) {
    return Stop {};
  }
  if (robot.people_positions.empty() and robot.cameras.overpassedYellowLine() and robot.gps.position().y > 6 and
      not robot.super_person_search) {
    return PersonSearch {};
  }

  if (robot.ir_sensors.frontDetection()) {
    if (robot.super_person_search) {
      return PersonSearch {}; // TODO this is a fix if we have time implement hfsm
    }
    return ObstacleAvoidance {};
  }
  if (robot.compass.isFacingDesired()) {
    return MoveForward {};
  }
  return std::nullopt;
}

inline std::optional<state_variant> transition([[maybe_unused]] MoveForward& state, MyRobot& robot) {
  auto const people_found = robot.people_positions.size();
  if (robot.people_positions.size() == 2 and robot.gps.position().y < 0 and robot.cameras.overpassedYellowLine()) {
    return Stop {};
  }
  if (robot.people_positions.empty() and robot.cameras.overpassedYellowLine() and robot.gps.position().y > 6 and
      not robot.super_person_search) {
    return PersonSearch {};
  }

  if (robot.ir_sensors.frontDetection()) {
    if (robot.cameras.personInFront() and not robot.personAlreadySaved()) {
      return PersonPickUp {};
    }
    if (robot.super_person_search) {
      return PersonSearch {}; // TODO this is a fix if we have time implement hfsm
    }
    return ObstacleAvoidance {};
  }
  auto const person_dir = robot.cameras.personInSide();

  if (person_dir == utils::Direction::left) {
    robot.compass.desiredAngle(robot.compass.desiredAngle() - 10);
    return Orientation {};
  }

  if (person_dir == utils::Direction::right) {
    robot.compass.desiredAngle(robot.compass.desiredAngle() + 10);
    return Orientation {};
  }

  return std::nullopt;
}

inline std::optional<state_variant> transition([[maybe_unused]] ObstacleAvoidance& state, MyRobot& robot) {
  if (robot.people_positions.size() == 2 and robot.gps.position().y < 0 and robot.cameras.overpassedYellowLine()) {
    return Stop {};
  }
  if (robot.people_positions.empty() and robot.cameras.overpassedYellowLine() and robot.gps.position().y > 6 and
      not robot.super_person_search) {
    return PersonSearch {};
  }
  if (robot.ir_sensors.frontDetection() and robot.cameras.personInFront() and not robot.personAlreadySaved() and
      robot.people_positions.size() < 2) {
    return PersonPickUp {};
  }
  if (robot.compass.isFacingDesired(10) and not robot.ir_sensors.frontDetection()) return Orientation {};
  return std::nullopt;
}

inline std::optional<state_variant> transition([[maybe_unused]] PersonPickUp& state, MyRobot& robot) {
  if (abs(robot.motors.velocity().x) < 5 and robot.compass.isFacingDesired(10)) {
    robot.people_positions.push_back(robot.odometry.getPos());

    if (robot.super_person_search and robot.people_positions.size() == 2) {
      robot.super_person_search = false;
      robot.compass.desiredAngle(270); // Back home buddy!
      return ObstacleAvoidance {};
    }

    return PersonSearch {};
  }
  return std::nullopt;
}

inline std::optional<state_variant> transition([[maybe_unused]] PersonSearch& state, MyRobot& robot) {
  if (robot.ir_sensors.frontDetection()) {
    if (robot.cameras.personInFront() and not robot.personAlreadySaved()) return PersonPickUp {};
  }
  else {
    auto const person_dir = robot.cameras.personInSide();

    // if (person_dir == utils::Direction::left) {
    //   logger(Log::controller) << "Person to the left";
    //   robot.compass.desiredAngle(robot.compass.desiredAngle() - 5);
    //   return Orientation {};
    // }
    //
    // if (person_dir == utils::Direction::right) {
    //   logger(Log::controller) << "Person to the right";
    //   robot.compass.desiredAngle(robot.compass.desiredAngle() + 5);
    //   return Orientation {};
    // }

    if (robot.compass.isFacingDesired(20)) return Orientation {};
  }

  return std::nullopt;
}

inline std::optional<state_variant> transition([[maybe_unused]] Stop& state, MyRobot& robot) { return std::nullopt; }

inline std::optional<state_variant> transition([[maybe_unused]] Dummy& state, MyRobot& robot) { return std::nullopt; }
