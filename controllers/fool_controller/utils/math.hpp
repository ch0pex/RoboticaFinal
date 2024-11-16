#ifndef MATH_HPP_
#define MATH_HPP_

#include <cmath>
#include <cstdint>
#include <iostream>
#include <vector>


namespace math {

constexpr double pi_v = 3.14159265358979323846264338327950288;

template<typename T>
struct vec2 {
  vec2() : x {}, y {} { }
  explicit vec2(T num) : x(num), y(num) { }
  vec2(T num, T num2) : x(num), y(num2) { }

  vec2 operator+(vec2 const& other) const { return {other.x + x, other.y + y}; }
  vec2 operator-(vec2 const& other) const { return {other.x - x, other.y - y}; }
  void operator+=(vec2 const& other) const {
    x += other.x;
    y += other.y;
  }
  void operator-=(vec2 const& other) const {
    x -= other.x;
    y -= other.y;
  }
  bool operator==(vec2 const& other) const { return x == other.x && y == other.y; }

  T x, y;
};

template<typename T>
struct vec3 {
  vec3() : x {}, y {}, z {} { }
  explicit vec3(T num) : x(num), y(num), z(num) { }
  vec3(T num, T num2, T num3) : x(num), y(num2), z(num3) { }

  vec3 operator+(vec3 const& other) const { return {other.x + x, other.y + y, other.z + z}; }
  vec3 operator-(vec3 const& other) const { return {other.x - x, other.y - y, other.z - z}; }
  void operator+=(vec3 const& other) const {
    x += other.x;
    y += other.y;
    z += other.z;
  }
  void operator-=(vec3 const& other) const {
    x -= other.x;
    y -= other.y;
    z -= other.z;
  }
  bool operator==(vec3 const& other) const { return x == other.x && y == other.y && z == other.z; }
  // bool operator<=(const auto& other) const { return x <= other && y <= other && z <= other; }
  // bool operator>=(const auto& other) const { return x >= other && y >= other && z >= other; }

  T x, y, z;
};

constexpr inline double convert_bearing_to_degrees(double const* in_vector) {
  double const rad = atan2(in_vector[0], in_vector[2]);
  double const deg = rad * (180.0 / pi_v);

  return deg;
}

constexpr inline double ir_to_distance(double const ir_value) { return (1024 - ir_value) * 0.2; }

} // namespace math

#endif
