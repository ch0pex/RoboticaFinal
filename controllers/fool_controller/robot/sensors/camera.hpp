#pragma once

#include "utils/logger.hpp"
#include "utils/math.hpp"

#include <webots/Camera.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Robot.hpp>

#include <array>
#include <execution>

namespace sensors {

class Camera {
public:
  struct Resolution {
    std::int32_t width;
    std::int32_t height;
  };

  struct Pixel {
    std::int32_t red;
    std::int32_t green;
    std::int32_t blue;
  };

  Camera(webots::Robot& robot, std::string_view name) :
    cam_(robot.getCamera(std::string(name))), resolution_({cam_->getWidth(), cam_->getHeight()}) {
    cam_->enable(utils::time_step);
  }

  ~Camera() { cam_->disable(); }

  [[nodiscard]] Resolution res() const { return resolution_; }

  Pixel pixel(math::vec2<std::int32_t> const position) {
    assert(position.x > 0 and position.x < resolution_.x);
    assert(position.y > 0 and position.y < resolution_.y);
  }


private:
  using pixel_pos = math::vec2<std::int32_t>;

  std::int32_t red(pixel_pos const pos) {return webots::Camera::

  }

  std::int32_t green(pixel_pos const pos) { }

  std::int32_t blue(pixel_pos const pos) { }

  webots::Camera* cam_;
  Resolution resolution_;
};

class Cameras {
public:
  explicit Cameras(webots::Robot& robot) : front_cam_(robot, "camera_f"), sphere_cam_(robot, "camera_s") {
    logger(Log::robot) << "Cameras initialized";
  }


  bool isPersonInFront() {
    std::int32_t sum = 0;
    std::float_t percentage_green, green, {0.0f};
    std::uint8_t const* image_f = front_cam_->getImage();

    // count number of pixels that are white
    // (here assumed to have pixel value > 245 out of 255 for all color components)
    for (std::int32_t x = 0; x < image_width_f; x++) {
      for (std::int32_t y = 0; y < image_height_f; y++) {
        green = webots::Camera::imageGetGreen(image_f, image_width_f, x, y);
        //        red   = webots::Camera::imageGetRed(image_f, image_width_f, x, y);
        //        blue  = webots::Camera::imageGetBlue(image_f, image_width_f, x, y);

        if (green > 245) {
          sum = sum + 1;
        }
      }
    }

    percentage_green = (sum / (float)(image_width_f * image_height_f));
    return percentage_green > 0.99;
  }

private:
  Camera front_cam_;
  Camera sphere_cam_;
};

} // namespace sensors
