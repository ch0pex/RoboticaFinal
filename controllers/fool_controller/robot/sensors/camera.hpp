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
    std::uint8_t red;
    std::uint8_t green;
    std::uint8_t blue;
  };

  Camera(webots::Robot& robot, std::string_view name) :
    cam_(robot.getCamera(std::string(name))), resolution_({cam_->getWidth(), cam_->getHeight()}) {
    cam_->enable(utils::time_step);
  }

  ~Camera() { cam_->disable(); }

  [[nodiscard]] Resolution res() const { return resolution_; }

  [[nodiscard]] Pixel pixel(math::vec2<std::int32_t> const position) const {
    return {red(position), green(position), blue(position)};
  }

  [[nodiscard]] std::uint8_t red(math::vec2<std::int32_t> const pos) const {
    return webots::Camera::imageGetRed(cam_->getImage(), resolution_.width, pos.x, pos.y);
  };

  [[nodiscard]] std::uint8_t green(math::vec2<std::int32_t> const pos) const {
    return webots::Camera::imageGetGreen(cam_->getImage(), resolution_.width, pos.x, pos.y);
  }

  [[nodiscard]] std::uint8_t blue(math::vec2<std::int32_t> const pos) const {
    return webots::Camera::imageGetBlue(cam_->getImage(), resolution_.width, pos.x, pos.y);
  }


private:
  webots::Camera* cam_;
  Resolution resolution_;
};

class Cameras {
public:
  explicit Cameras(webots::Robot& robot) : front_cam_(robot, "camera_f"), sphere_cam_(robot, "camera_s") {
    logger(Log::robot) << "Cameras initialized";
  }


  bool isPersonInFront() {
    std::int32_t sum              = 0;
    std::uint32_t green           = 0;
    std::float_t percentage_green = 0;

    auto [width, height]     = front_cam_.res();
    float const aspect_ratio = static_cast<float>(width) * static_cast<float>(height);

    // count number of pixels that are white
    // (here assumed to have pixel value > 245 out of 255 for all color components)
    for (std::int32_t x = 0; x < width; x++) {
      for (std::int32_t y = 0; y < height; y++) {
        green = front_cam_.green({x, y});
        //        red   = webots::Camera::imageGetRed(image_f, image_width_f, x, y);
        //        blue  = webots::Camera::imageGetBlue(image_f, image_width_f, x, y);

        if (green > 10) {
          ++sum;
        }
      }
    }

    percentage_green = (sum / aspect_ratio);
    return percentage_green > 0.99;
  }

  [[nodiscard]] std::int32_t wallInFront() const {
    std::int32_t sum              = 0;
    std::uint32_t blue            = 0;
    std::float_t percentage_green = 0;

    auto [width, height] = front_cam_.res();
    logger(Log::robot) << "Res: " << width << " " << height << " ";

    // count number of pixels that are white
    // (here assumed to have pixel value > 245 out of 255 for all color components)
    for (std::int32_t x = 0; x < width; x++) {
      for (std::int32_t y = 0; y < height; y++) {
        blue = front_cam_.green({x, y});
        //        red   = webots::Camera::imageGetRed(image_f, image_width_f, x, y);
        // blue  = webots::Camera::imageGetBlue(image_f, image_width_f, x, y);

        if (blue > 245) {
          ++sum;
        }
      }
    }

    return sum;
  }

private:
  Camera front_cam_;
  Camera sphere_cam_;
};

} // namespace sensors
