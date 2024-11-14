/**
 * @file    MyRobot.cpp
 * @brief   A simple controller for wall following -> Clockwise preference
 * @author  Javier Pastor Fernandez <javpasto@ing.uc3m.es>
 * @date    2021
 */


#include "MyRobot.hpp"

//////////////////////////////////////////////

MyRobot::MyRobot() : Robot(), motors(this), pos_(0, -8.5), theta_(0.14) {

  my_compass_ = getCompass("compass");
  my_compass_->enable(utils::time_step);

  for (int ind = 0; ind < 16; ind++) {
    std::string sensor_name = std::string("ds") + std::to_string(ind);
    std::cout << "Initializing distance sensor: " << sensor_name << "\n";
    distance_sensor_[ind] = getDistanceSensor(sensor_name);
    distance_sensor_[ind]->enable(utils::time_step);
  }

  // set position to infinity, to allow velocity control
}

MyRobot::~MyRobot() {
  // disable devices --> distance sensor
  for (int ind = 0; ind < 2; ind++) {
    std::cout << "Disabling distance sensor: " << ds_name_[ind] << "\n";
    distance_sensor_[ind]->disable();
  }
}


void MyRobot::ReadSensors() {
  front_left  = math::ir_to_distance(distance_sensor_[0]->getValue());
  front_right = math::ir_to_distance(distance_sensor_[15]->getValue());
  left_ir     = math::ir_to_distance(distance_sensor_[3]->getValue());
  right_ir    = math::ir_to_distance(distance_sensor_[5]->getValue());
  std::cout << "Front left: " << front_left << "\n";
  std::cout << "Front right: " << front_left << "\n";
  std::cout << "Left: " << left_ir << "\n";
  std::cout << "Right: " << right_ir << "\n";
}

bool MyRobot::IsFacingDesiredAngle() {
  double const* compass_val = my_compass_->getValues();
  // convert compass bearing vector to angle, in degrees
  compass_angle = utils::convert_bearing_to_degrees(compass_val);

  // print sensor values to console
  std::cout << "Desired angle (degrees): " << desired_angle << "\n";
  std::cout << "Compass angle (degrees): " << compass_angle << "\n";

  if (compass_angle * desired_angle < 0 && abs(compass_angle - desired_angle) > 180)
  // Going through angle discontinuity
  {
    if (compass_angle < (desired_angle - 2)) {
      // turn left
      return false;
    }
    else if (compass_angle > (desired_angle + 2)) {
      // turn right
      return false;
    }
    // move straight forward
    return true;
  }

  else {
    if (compass_angle < (desired_angle - 2)) {
      // turn right
      return false;
    }
    else if (compass_angle > (desired_angle + 2)) {
      // turn left
      return false;
    }
    // move straight forward
    return true;
  }
}

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

void MyRobot::ComputeOdometry() {
  pos_.x += ((sr_ + sl_) / 2 * sin(theta_ + ((sr_ - sl_) / (2 * utils::wheel_distance))));
  pos_.y += ((sr_ + sl_) / 2 * cos(theta_ + ((sr_ - sl_) / (2 * utils::wheel_distance))));
  theta_ = theta_ + ((sr_ - sl_) / utils::wheel_distance);
}
