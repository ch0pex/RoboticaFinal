#pragma once


#include "../fsm/state_machine.hpp"
#include "../utils/math.hpp"
#include "components/motors.hpp"
#include "components/navigation.hpp"


#include <array>
#include <webots/DistanceSensor.hpp>
#include <webots/Robot.hpp>

using namespace webots;

struct MyRobot final : private Robot {

  MyRobot();

  ~MyRobot() override;

  bool time_step() { return step(utils::time_step) != -1; }

  void ReadSensors();


  // Motors
  Motors motors;
  Navigation navigation;

  std::array<DistanceSensor*, 16> distance_sensor_;
  std::array<char const*, 16> ds_name_;
  double front_left, front_right, left_ir, right_ir;
  std::array<double, 4> lasers_;
};


/*
template <utils::Direction direction>
inline void MyRobot::WallFollower()
{

        left_motor_->setPosition(INFINITY);
        right_motor_->setPosition(INFINITY);

        double left_speed = 0, right_speed = 0, side_sensor = 0;
        constexpr utils::Direction opposite_dir = direction == utils::left ? utils::left : utils::right;

        if (direction == utils::left) {
            side_sensor = left_ir;
        } else {
            side_sensor = right_ir;
        }

        if (side_sensor > 0) {
            if (front_ir > 0) {
                Move<direction>();
            } else {
                Move<utils::Direction::front>();
            }
        } else {
            if (front_ir > 0) {
                Move<direction>();
            } else {
                Move<opposite_dir>();
            }
        }

        if (distance_sensor_[2]->getValue() > 0) {
           right_speed -= 0.5;
        } else if (distance_sensor_[2]->getValue() <= 0){
            right_speed += 0.5;
        }

        left_motor_->setVelocity(left_speed);
        right_motor_->setVelocity(right_speed);
        double left_speed = 0;
        double right_speed = 0;

        if (left_ir < min_distance) {
            if (front_ir < min_distance) {
                left_speed += 6.0f;
                right_speed -= 8.0f;
            } else {
                left_speed += 5.0f;
                right_speed += 5.0f;
            }
        } else {
            if (front_ir < min_distance) {
                left_speed += 6.0f;
                right_speed -= 6.0f;
            } else {
//                left_speed -= 0.0f;
                right_speed += 6.0f;
            }
        }

        if (distance_sensor_[2]->getValue() > min_distance) {
           right_speed -= 0.5;
        } else if (distance_sensor_[2]->getValue() <= min_distance){
            right_speed += 0.5;
        }

        left_motor_->setVelocity(left_speed);
        right_motor_->setVelocity(right_speed);


}
*/
