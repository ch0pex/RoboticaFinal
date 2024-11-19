#pragma once


#include "components/actuators.hpp"
#include "components/navigation.hpp"
#include "components/sensors.hpp"


#include <array>
#include <webots/DistanceSensor.hpp>
#include <webots/Robot.hpp>

using namespace webots;

struct MyRobot final : private Robot {

  MyRobot() : motors(*this), gps(*this), compass(*this), cameras(*this), ir_sensors(*this) { motors.setVelocity(0); }

  bool time_step() { return step(utils::time_step) != -1; }

  // Motors
  actuators::Motors motors;
  navigation::Gps gps;
  navigation::Compass compass;
  navigation::Odometry odometry;
  sensors::Cameras cameras;
  sensors::Infrared ir_sensors;
};


/*
template <utils::Direction direction>
inline void MyRobot::WallFollower()
{
}
*/
