/************************************************************************
 * Copyright (c) 2024 Alvaro Cabrera Barrio
 * This code is licensed under MIT license (see LICENSE.txt for details)
 ************************************************************************/
/**
 * @file sensors.hpp
 * @version 1.0
 * @date 14/11/2024
 * @brief Short description
 *
 * Longer description
 */

#pragma once

namespace sensors {

class Cameras {
public:
  explicit Cameras(Robot& robot) : front_cam_(robot.getCamera("camera_f")), sphere_cam_(robot.getCamera("sphere_cam")) {
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

class DistanceSensors {
public:
private:
};

} // namespace sensors
