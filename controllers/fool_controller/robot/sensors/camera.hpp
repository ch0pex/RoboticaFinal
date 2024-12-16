#pragma once

#include "utils/logger.hpp"
#include "utils/math.hpp"

#include <webots/Camera.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Robot.hpp>

#include <array>
#include <chrono>

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
    cam_(robot.getCamera(std::string(name))), resolution_(cam_->getWidth(), cam_->getHeight()) {
    cam_->enable(utils::time_step);
  }

  ~Camera() { cam_->disable(); }

  [[nodiscard]] Resolution res() const { return resolution_; }

  [[nodiscard]] bool personInSight() const {
    return checkPersonInRegion({0, 0}, {resolution_.width, resolution_.height}) > 0.05;
  }

  [[nodiscard]] utils::Direction personInSide() const {
    if (not personInSight()) return utils::Direction::front;

    if (checkPersonInRegion({0, 0}, {resolution_.width / 2, resolution_.height}) >
        checkPersonInRegion({resolution_.width / 2, 0}, {resolution_.width, resolution_.height})) {
      logger(Log::controller) << "Person to the left";
      return utils::Direction::left;
    }
    logger(Log::controller) << "Person to the right";
    return utils::Direction::right;
  }

  [[nodiscard]] bool personInFront() const {
    return checkPersonInRegion({0, 0}, {resolution_.width, resolution_.height}) > 0.4;
  }

  [[nodiscard]] double checkPersonInRegion(math::vec2<int> start, math::vec2<int> end) const {
    return checkColorInRegion(start, end, green_threshold);
  }

  [[nodiscard]] double checkYellowLine() const {
    auto const height = resolution_.height >> 2;
    return checkColorInRegion<decltype(yellow_threshold), true>({44, 120}, {128, 135}, yellow_threshold);
  }

private:
  template<typename Threshold, bool relative = false>
  [[nodiscard]] double
  checkColorInRegion(math::vec2<int> const start, math::vec2<int> const end, Threshold& threshold) const {
    unsigned char const* image_f = cam_->getImage();
    Pixel pixel {};
    int sum           = 0;
    double percentage = 0.0;

    for (int x = start.x; x < end.x; x++) {
      for (int y = start.y; y < end.y; y++) {
        pixel.green = webots::Camera::imageGetGreen(image_f, resolution_.width, x, y);
        pixel.red   = webots::Camera::imageGetRed(image_f, resolution_.width, x, y);
        pixel.blue  = webots::Camera::imageGetBlue(image_f, resolution_.width, x, y);

        if (threshold(pixel)) {
          ++sum;
        }
      }
    }

    if constexpr (not relative) {
      percentage = sum / static_cast<std::float_t>(resolution_.width * resolution_.height);
    }
    else {
      percentage = sum / static_cast<std::float_t>((end.x - start.x) * (end.y - start.y));
    }

    logger(Log::robot) << "Color percentage: " << percentage;

    return percentage;
  }

  static constexpr auto green_threshold = [](Pixel const& pixel) {
    return pixel.red < 60 and pixel.green > 60 and pixel.blue < 60;
  };

  static constexpr auto yellow_threshold = [](Pixel const& pixel) {
    return pixel.red > 88 and pixel.green > 88 and pixel.blue < 88;
  };

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

  [[nodiscard]] bool overpassedYellowLine() const { return sphere_cam_.checkYellowLine() > 0.025; }

private:
  Camera front_cam_;
  Camera sphere_cam_;
};

} // namespace sensors
