#ifndef FSM_HPP_
#define FSM_HPP_

#include <tuple>
#include <variant>
#include <functional>
#include <optional>
#include <memory>

template<typename Robot, typename state_variant> 
class Controller { 
public: 
    Controller(std::unique_ptr<Robot> robot, state_variant && initial_state) 
    : robot(std::move(robot)), current_state_ {std::move(initial_state)} { }

    void run() {
        while(robot_.time_step()) { 
            std::visit([&context = robot_](auto &state) { state.robot_(context); }, current_state_);
            auto newState = std::visit(
                [&context = robot_](auto &state) -> std::optional<state_variant> {
                    return transition(state, context);
                }, current_state_
            );

            if (newState) {
                current_state_ = std::move(newState.value());
            }
        }
  }

private: 
    state_variant current_state_;
    std::unique_ptr<Robot> robot_;
};


#endif