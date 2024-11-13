#pragma once

#include <webots/Motor.hpp>
#include <webots/Robot.hpp>

using namespace webots;

class Motors { 
public:
    Motors(Robot* robot) 
    : left(robot->getMotor("left wheel motor"))
    , right(robot->getMotor("right wheel motor")) 
    {
        left->setPosition(INFINITY);
        right->setPosition(INFINITY);
    }

private:
    static constexpr double max_velocity { 5.0 };
    static constexpr double distance     { 0.32 };
    static constexpr double radius       { 0.0825 };

    Motor* left;
    Motor* right;
};

template <>
inline void MyRobot::Rotate<utils::Direction::right>()
{
    motors.left->setVelocity(max_velocity);
    motors.right->setVelocity(-max_velocity);
}

template <>
inline void MyRobot::Rotate<utils::Direction::left>()
{
    motors.left->setVelocity(-max_velocity);
    motors.right->setVelocity(max_velocity);
}

template <>
inline void MyRobot::Move<utils::Direction::front>()
{
    motors.left->setVelocity(max_velocity);
    motors.right->setVelocity(max_velocity);
}

template <>
inline void MyRobot::Move<utils::Direction::back>()
{
    motors.left->setVelocity(-max_velocity);
    motors.right->setVelocity(-max_velocity);
}