#pragma once


#include "../utils/math.hpp"
#include "../fsm/state_machine.hpp"
#include "components/motors.hpp"

// include dependencies
#include <iostream>
#include <limits>
#include <math.h>

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Compass.hpp>
#include <array>

using namespace webots;

class MyRobot : public Robot
{
public:
    /**
     * @brief Empty constructor of the class.
     */
    MyRobot();

    /**
     * @brief Destructor of the class.
     */
    ~MyRobot();

    bool time_step() { return step(65) != -1; } 

    template<utils::Direction direction> void Move();

    template<utils::Direction direction> void Rotate();

    template<utils::Direction motor> 
    void setMotorVelocity(double speed) { 
        if constexpr (motor == left) { 
            left_motor_->setVelocity(speed);
        } else { 
            right_motor_->setVelocity(speed);
        }
    }

    void stop() { 
        // motors.left->setVelocity(0);
        // motors.right->setVelocity(0);
    }

    void ReadSensors();

    bool IsFacingDesiredAngle();

    void ComputeOdometry();

private:
    static constexpr double min_distance    { 15 };
    static constexpr double desired_angle   { -90.0 };

    // Motors
    Motors motors;

    Compass *my_compass_;

    std::array<DistanceSensor*, 16> distance_sensor_;
    std::array<const char *, 16> ds_name_;
    double front_left, front_right, left_ir, right_ir;
    std::array<double, 4> lasers_;
    double compass_angle;

    //Odometry
    utils::vec2<double> pos_;
    double theta_, sl_, sr_;
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