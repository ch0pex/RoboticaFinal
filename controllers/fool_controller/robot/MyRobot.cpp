/**
 * @file    MyRobot.cpp
 * @brief   A simple controller for wall following -> Clockwise preference
 * @author  Javier Pastor Fernandez <javpasto@ing.uc3m.es>
 * @date    2021
 */


#include "MyRobot.hpp"

//////////////////////////////////////////////

MyRobot::MyRobot()
    :   Robot(), pos_(0, -8.5), goal_pos_(0, 8), displacement_(), theta_(0.14),
        theta_goal_(atan2((goal_pos_.x - pos_.x), (goal_pos_.y - pos_.y)))
{
    _time_step = 64; // control time step

    my_compass_= getCompass("compass");
    my_compass_->enable(_time_step);

    left_motor_ = getMotor("left wheel motor");
    right_motor_ = getMotor("right wheel motor");


    for (int ind = 0; ind < 16; ind++){
      std::string sensor_name = std::string("ds") + std::to_string(ind);
      cout << "Initializing distance sensor: " << sensor_name << endl;
      distance_sensor_[ind] = getDistanceSensor(sensor_name.c_str());
      distance_sensor_[ind]->enable(_time_step);
    }
    
    // set position to infinity, to allow velocity control
    left_motor_->setPosition(INFINITY);
    right_motor_->setPosition(INFINITY);

    // AddStopState();
    // AddMoveState();
    // AddAvoidState();
    // stateMachine_.ChangeState(State::FORWARD);
}

MyRobot::~MyRobot()
{
    // disable devices --> distance sensor
    for (int ind = 0; ind < 2; ind++) {
      cout << "Disabling distance sensor: " << ds_name_[ind] << endl;
      distance_sensor_[ind]->disable();
    }   
}

void MyRobot::Idle() {
    left_motor_->setVelocity(0);
    right_motor_->setVelocity(0);
}

void MyRobot::ReadSensors() {
    front_left = utils::ir_to_distance(distance_sensor_[0]->getValue() );
    front_right = utils::ir_to_distance(distance_sensor_[15]->getValue());
    left_ir = utils::ir_to_distance(distance_sensor_[3]->getValue());
    right_ir = utils::ir_to_distance(distance_sensor_[5]->getValue());
    std::cout << "Front left: " << front_left << "\n";
    std::cout << "Front right: " << front_left << "\n";
    std::cout << "Left: " << left_ir << "\n";
    std::cout << "Right: " << right_ir << "\n";
}

bool MyRobot::IsFacingDesiredAngle() {
  const double *compass_val = my_compass_->getValues();
  // convert compass bearing vector to angle, in degrees
  compass_angle = utils::convert_bearing_to_degrees(compass_val);

  // print sensor values to console
  cout << "Desired angle (degres): " << desired_angle << endl;
  cout << "Compass angle (degrees): " << compass_angle << endl;

  if (((compass_angle * desired_angle) < 0) && ((abs(compass_angle - desired_angle)) > 180))
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

utils::Direction MyRobot::RightOrLeft() {
    return (follower_dir = follower_dir == utils::right ? utils::left : utils::right);
}

void MyRobot::ComputeOdometry()
{
  pos_.x += ((sr_ + sl_) / 2 * sin(theta_ + ((sr_ - sl_) / (2 * wheel_distance))));
  pos_.y += ((sr_ + sl_) / 2 * cos(theta_ + ((sr_ - sl_) / (2 * wheel_distance))));
  theta_ = theta_ + ((sr_ - sl_) / wheel_distance);
  
}        