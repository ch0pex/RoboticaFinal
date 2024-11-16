/**
 * @file    MyRobot.cpp
 * @brief   A simple controller for wall following -> Clockwise preference
 * @author  Javier Pastor Fernandez <javpasto@ing.uc3m.es>
 * @date    2021
 */


#include "MyRobot.hpp"

//////////////////////////////////////////////

// MyRobot::MyRobot() : motors(this) {


//  for (int ind = 0; ind < 16; ind++) {
//    std::string sensor_name = std::string("ds") + std::to_string(ind);
//    std::cout << "Initializing distance sensor: " << sensor_name << "\n";
//    distance_sensor_[ind] = getDistanceSensor(sensor_name);
//    distance_sensor_[ind]->enable(utils::time_step);
//  }
//
//   set position to infinity, to allow velocity control
//}


// void MyRobot::ReadSensors() {
//   front_left  = math::ir_to_distance(distance_sensor_[0]->getValue());
//   front_right = math::ir_to_distance(distance_sensor_[15]->getValue());
//   left_ir     = math::ir_to_distance(distance_sensor_[3]->getValue());
//   right_ir    = math::ir_to_distance(distance_sensor_[5]->getValue());
//   std::cout << "Front left: " << front_left << "\n";
//   std::cout << "Front right: " << front_left << "\n";
//   std::cout << "Left: " << left_ir << "\n";
//   std::cout << "Right: " << right_ir << "\n";
// }

// void MyRobot::AddStopState() {
// utils::State stop_state {
// [this] () { Idle(); },
// [this] () {},
// [this] () {},
// };
// stateMachine_.AddState(std::move(stop_state));
// }

// void MyRobot::AddAvoidState() {
// utils::State avoid_state {
// [this] () {
// std::cout << "Enter avoid state\n"; },
// [this] () {
// ReadSensors();
// if (RightOrLeft() == utils::right) {
// WallFollower<utils::right>();
// } else {
// WallFollower<utils::left>();
// }
// if (IsFacingDesiredAngle()) {
// stateMachine_.ChangeState(State::FORWARD);
// }
//
// },
// [this] () {},
// };
// stateMachine_.AddState(std::move(avoid_state));
// }

// void MyRobot::AddMoveState() {
// utils::State left {
// [this] () { std::cout << "Enter left state\n"; },
// [this] () {
// ReadSensors();
// Move<utils::Direction::left>();
// },
// [this] () {},
// };
//
// utils::State right {
// [this] () { std::cout << "Enter right state\n"; },
// [this] () {
// ReadSensors();
// Move<utils::Direction::right>();
// },
// [this] () {},
// };

// utils::State forward{
// [this]() { std::cout << "Enter forward state\n"; },
// [this]() {
// ReadSensors();
// Move<utils::Direction::front>();
// if (left_ir < min_distance) {
// follower_dir = utils::left;
// stateMachine_.ChangeState(State::OBSTACLE_AVOID);
// }
// if (right_ir < min_distance) {
// follower_dir = utils::right;
// stateMachine_.ChangeState(State::OBSTACLE_AVOID);
// }
// if (front_left < min_distance || front_right < min_distance) {
// stateMachine_.ChangeState(State::OBSTACLE_AVOID);
// }
// },
// [this]() {}
// };

// stateMachine_.AddState(std::move(forward));
// stateMachine_.AddState(std::move(left));
// stateMachine_.AddState(std::move(right));
// }

// utils::Direction MyRobot::RightOrLeft() {
// return (follower_dir = follower_dir == utils::right ? utils::left : utils::right);
// }
