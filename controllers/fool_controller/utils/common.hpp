#pragma once

#include "math.hpp"

#include <cstdint>

namespace utils {

using namespace math::literals;

constexpr int time_step {64};
constexpr double delta_time {1.0 / time_step};
constexpr double wheel_distance {0.32};
constexpr double wheel_radius {0.0825};
constexpr double max_velocity {10.0};
constexpr double min_distance {30};
constexpr std::uint64_t max_time_searching {4_min};

enum class Direction : uint8_t { left = 0, right, back, front };

enum class Color : uint8_t {
  white = 0,
  yellow,
};

} // namespace utils
