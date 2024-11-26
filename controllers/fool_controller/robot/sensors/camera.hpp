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

  bool isPersonInFront () {
    int sum = 0;

    // get current image from forward camera
    const unsigned char *image_f = _forward_camera->getImage();

    // count number of pixels that are white
    // (here assumed to have pixel value > 245 out of 255 for all color components)
    for (int x = 0; x < image_width_f; x++) {
        for (int y = 0; y < image_height_f; y++) {
            green = _forward_camera->imageGetGreen(image_f, image_width_f, x, y);
            red = _forward_camera->imageGetRed(image_f, image_width_f, x, y);
            blue = _forward_camera->imageGetBlue(image_f, image_width_f, x, y);

            if (green > THRESHOLD)  {
                sum = sum + 1;
            }
        }
    }

    percentage_green = (sum / (float) (image_width_f * image_height_f)) ;
    return percentage_green > 0.99;  

  }

private:
  Camera* front_cam_;
  Camera* sphere_cam_;
};

} // namespace sensors
