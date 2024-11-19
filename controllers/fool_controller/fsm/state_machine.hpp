#pragma once

#include "utils/logger.hpp"

#include <functional>
#include <iostream>
#include <memory>
#include <optional>
#include <ostream>
#include <variant>

template<typename Robot, typename state_variant>
class Controller {
public:
  Controller(std::unique_ptr<Robot> robot, state_variant&& initial_state) :
    current_state_ {std::move(initial_state)}, robot_(std::move(robot)) {
    robot_->time_step();
    std::visit(
        [&context = robot_](auto& state) {
          logger(Log::controller) << "Initial state - " << state << '\n';
          state.enter(*context);
        },
        current_state_
    );
  }

  void run() {
    logger(Log::controller) << "Init...";
    while (robot_->time_step()) {
      logger(Log::separator);
      std::visit([&context = robot_](auto& state) { state.update(*context); }, current_state_);
      auto newState = std::visit(
          [&context = robot_](auto& state) -> std::optional<state_variant> { return transition(state, *context); },
          current_state_
      );

      if (newState) {
        current_state_ = std::move(newState.value());
        std::visit(
            [&context = robot_](auto& state) {
              logger(Log::separator);
              logger(Log::controller) << "New State - " << state;
              state.enter(*context);
            },
            current_state_
        );
      }
    }
  }

private:
  state_variant current_state_;
  std::unique_ptr<Robot> robot_;
};
