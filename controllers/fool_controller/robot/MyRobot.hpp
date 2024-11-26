#pragma once


#include "actuators/motor.hpp"

#include "sensors/camera.hpp"
#include "sensors/compass.hpp"
#include "sensors/infrared.hpp"

#include "navigation/gps.hpp"
#include "navigation/odometry.hpp"

#include <array>
#include <webots/DistanceSensor.hpp>
#include <webots/Robot.hpp>

using namespace webots;

struct MyRobot final : private Robot {

  MyRobot() : motors(*this), compass(*this), cameras(*this), ir_sensors(*this), gps(*this) { }

  bool time_step() { return step(utils::time_step) != -1; }

  // *** Actuators ***
  actuators::Motors motors;

  // *** Sensors ***
  sensors::Compass compass;
  sensors::Cameras cameras;
  sensors::Infrared ir_sensors;

  // *** Navigation ***
  navigation::Gps gps;
  navigation::Odometry odometry;
};
