#pragma once

#include "utils/logger.hpp"
#include "utils/math.hpp"

#include <webots/Camera.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Robot.hpp>

#include <array>
#include <chrono>
#include <execution>

namespace sensors {

class Camera {
public:
  struct Resolution {
    Resolution(std::int32_t const width, std::int32_t const height) :
      width(width), height(height),
      aspect_ratio {static_cast<std::float_t>(width) / static_cast<std::float_t>(height)} { }
    std::int32_t width;
    std::int32_t height;
    std::float_t aspect_ratio;
  };

  struct Pixel {
    std::uint8_t red;
    std::uint8_t green;
    std::uint8_t blue;
  };

  Camera(webots::Robot& robot, std::string_view const name) :
    cam_(robot.getCamera(std::string(name))), resolution_({cam_->getWidth(), cam_->getHeight()}) {
    cam_->enable(utils::time_step);
  }

  ~Camera() { cam_->disable(); }

  [[nodiscard]] Resolution res() const { return resolution_; }

  [[nodiscard]] bool personInSight() const {
    return checkPersonInRegion(0.1, {0, 0}, {resolution_.width, resolution_.height});
  }

  [[nodiscard]] utils::Direction personInSide() const {
    if (not personInSight()) return utils::Direction::front;

    if (checkPersonInRegion(0.2, {0, 0}, {resolution_.width / 2, resolution_.height})) {
      return utils::Direction::left;
    }
    return utils::Direction::right;
  }

  [[nodiscard]] bool personInFront() const {
    return checkPersonInRegion(0.5, {0, 0}, {resolution_.width, resolution_.height});
  }

  [[nodiscard]] bool
  checkPersonInRegion(double const percentage_needed, math::vec2<int> start, math::vec2<int> end) const {
    unsigned char const* image_f = cam_->getImage();
    Pixel pixel {};
    int sum                 = 0;
    double percentage_green = 0.0;

    for (int x = 0; x < resolution_.width; x++) {
      for (int y = 0; y < resolution_.height; y++) {
        pixel.green = webots::Camera::imageGetGreen(image_f, resolution_.width, x, y);
        pixel.red   = webots::Camera::imageGetRed(image_f, resolution_.width, x, y);
        pixel.blue  = webots::Camera::imageGetBlue(image_f, resolution_.width, x, y);

        if (pixel.red < 30 and pixel.green > 100 and pixel.blue < 30) {
          ++sum;
        }
      }
    }

    percentage_green = sum / static_cast<std::float_t>(resolution_.width * resolution_.height);

    logger(Log::robot) << "Green percentage: " << percentage_green;

    return percentage_green > percentage_needed;
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

  [[nodiscard]] bool personInFront() const { return front_cam_.personInFront(); }

  [[nodiscard]] bool personInSight() const { return front_cam_.personInSight(); }

  [[nodiscard]] utils::Direction personInSide() const { return front_cam_.personInSide(); }

private:
  Camera front_cam_;
  Camera sphere_cam_;
};

} // namespace sensors
