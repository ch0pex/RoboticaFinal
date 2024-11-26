#pragma once

#include "utils/logger.hpp"
#include "utils/math.hpp"

#include <webots/Camera.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Robot.hpp>

#include <array>
#include <execution>

using namespace webots;

namespace sensors {

class Cameras {
public:
  explicit Cameras(Robot& robot) : front_cam_(robot.getCamera("camera_f")), sphere_cam_(robot.getCamera("camera_s")) {
    logger(Log::robot) << "Cameras initialized";
    front_cam_->enable(utils::time_step);
    sphere_cam_->enable(utils::time_step);
  }

  ~Cameras() {
    front_cam_->disable();
    sphere_cam_->disable();
  }

private:
  Camera* front_cam_;
  Camera* sphere_cam_;
};

} // namespace sensors
