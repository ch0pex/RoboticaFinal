#ifndef TRANSITIONS_HPP_
#define TRANSITIONS_HPP_

#include "states/localization.hpp"
#include "states/move_forward.hpp"
#include "states/obstacle_avoidance.hpp"
#include "states/orientation.hpp"
#include "states/person_pickup.hpp"
#include "states/person_search.hpp"
#include "states/stop.hpp"

#include <tuple>
#include <variant>
#include <optional>

using state_variant = std::variant<Localization, MoveForward, ObstacleAvoidance, Orientation, PersonPickUp, PersonSearch, Stop>;


inline std::optional<state_variant> transition(MoveForward& state, MyRobot const& robot) {

}

inline std::optional<state_variant> transition(ObstacleAvoidance& state, MyRobot const& robot) {

}

inline std::optional<state_variant> transition(Orientation& state, MyRobot const& robot) {

}

inline std::optional<state_variant> transition(PersonPickUp& state, MyRobot const& robot) {

}

inline std::optional<state_variant> transition(PersonSearch& state, MyRobot const& robot) {

}

inline std::optional<state_variant> transition(Stop& state, MyRobot const& robot) {

}

#endif