#pragma once


#include "actuators/motor.hpp"
#include "sensors/camera.hpp"
#include "sensors/compass.hpp"
#include "sensors/infrared.hpp"

#include "navigation/gps.hpp"
#include "navigation/odometry.hpp"

#include "utils/math.hpp"

#include <webots/Robot.hpp>

using namespace webots;

struct MyRobot final : private Robot {

  MyRobot() : motors(*this), compass(*this), cameras(*this), ir_sensors(*this), gps(*this) { }

  bool time_step() {
    auto const val = (step(utils::time_step) != -1);
    odometry.compute(compass.facingRadians(), motors.positionSensors());
    if (super_person_search) {
      time_searching += utils::time_step;
      if (time_searching > utils::max_time_searching) { // Time limit for super state 2 mins
        logger(Log::robot) << "TimeLimit reached, returning home";
        super_person_search = false;
        compass.desiredAngle(270); // Back home buddy!
        for (std::size_t i = people_positions.size(); i < 2; i++) {
          people_positions.emplace_back();
        }
      }
    }
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
  bool super_person_search     = false;
  std::uint64_t time_searching = 0;

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
