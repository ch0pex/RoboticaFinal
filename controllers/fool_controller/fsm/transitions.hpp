#pragma once

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
    std::variant<Localization, MoveForward, ObstacleAvoidance, Orientation, PersonPickUp, PersonSearch, Stop>;


inline std::optional<state_variant> transition(Localization& state, MyRobot const& robot) {
  if (robot.compass.isFacingDesired()) {
    return MoveForward {};
  }
  return Orientation {};
}

inline std::optional<state_variant> transition(Orientation& state, MyRobot& robot) {
  if (robot.compass.isFacingDesired())
    return MoveForward {};
  return std::nullopt;
}

inline std::optional<state_variant> transition(MoveForward& state, MyRobot& robot) {
  if (robot.ir_sensors.wallDetected()) {
    //    return ObstacleAvoidance {};
  }
  return std::nullopt;
}

inline std::optional<state_variant> transition(ObstacleAvoidance& state, MyRobot& robot) { return std::nullopt; }

inline std::optional<state_variant> transition(PersonPickUp& state, MyRobot& robot) { return std::nullopt; }

inline std::optional<state_variant> transition(PersonSearch& state, MyRobot& robot) { return std::nullopt; }

inline std::optional<state_variant> transition(Stop& state, MyRobot& robot) { return std::nullopt; }
