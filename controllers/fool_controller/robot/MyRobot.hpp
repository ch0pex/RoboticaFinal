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

  bool time_step() {
    auto const val = (step(utils::time_step) != -1);
    odometry.compute(compass.facingRadians(), motors.positionSensors());
    logger(Log::robot) << "Position: " << odometry.getPos();
    return val;
  }


  bool personAlreadySaved() {
    return std::any_of(people_positions.begin(), people_positions.end(), [this](auto const& pos) {
      auto const distance = pos.distance(odometry.getPos());
      logger(Log::robot) << "Distance to person" << distance;
      return distance < 1.0;
    });
  }

  // *** Found people positions ***/
  std::vector<math::vec2<double>> people_positions;
  bool super_person_search = false;

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
