#ifndef MY_ROBOT_HPP_
#define MY_ROBOT_HPP_


#include "../utils/math.hpp"
#include "../fsm/state_machine.hpp"

// include dependencies
#include <iostream>
#include <limits>
#include <math.h>

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Compass.hpp>
#include <array>

using namespace std;
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


    enum State {
        STOP = 0,
        FORWARD,
        TURN_LEFT,
        TURN_RIGHT,
        OBSTACLE_AVOID
    };

    bool time_step() { return step(time_step_) != -1; } 

private:
    utils::Direction RightOrLeft();

    template<utils::Direction direction>
    void WallFollower();

    template<utils::Direction direction>
    void Move();
    void Idle();

    void ReadSensors();
    bool IsFacingDesiredAngle();
    void ComputeOdometry();

    static constexpr double max_velocity_   { 5.0 };
    static constexpr double min_distance    { 15 };
    static constexpr double desired_angle   { -90.0 };
    static constexpr double wheel_distance  { 0.32 };
    static constexpr double wheel_radius    { 0.0825 };

    // The time step
    int time_step_;

    // velocities
    double left_speed, right_speed;

    // Motors
    Motor *left_motor_;
    Motor *right_motor_;

    Compass *my_compass_;

    std::array<DistanceSensor*, 16> distance_sensor_;
    std::array<const char *, 16> ds_name_;
    double front_left, front_right, left_ir, right_ir;
    utils::Direction follower_dir { utils::Direction::left };
    double compass_angle;

    //Odometry
    utils::vec2<double> pos_, goal_pos_;
    double theta_, theta_goal_, sl_, sr_;
};

template <>
inline void MyRobot::Move<utils::Direction::right>()
{
    left_motor_->setVelocity(max_velocity_);
    right_motor_->setVelocity(max_velocity_ / 2);
}

template <>
inline void MyRobot::Move<utils::Direction::left>()
{
    left_motor_->setVelocity(max_velocity_ / 2);
    right_motor_->setVelocity(max_velocity_);
}

template <>
inline void MyRobot::Move<utils::Direction::front>()
{
    left_motor_->setVelocity(max_velocity_);
    right_motor_->setVelocity(max_velocity_);
}

template <>
inline void MyRobot::Move<utils::Direction::back>()
{
    left_motor_->setVelocity(-max_velocity_);
    right_motor_->setVelocity(-max_velocity_);
}

template <utils::Direction direction>
inline void MyRobot::WallFollower()
{

        left_motor_->setPosition(INFINITY);
        right_motor_->setPosition(INFINITY);

/*
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
*/

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

#endif