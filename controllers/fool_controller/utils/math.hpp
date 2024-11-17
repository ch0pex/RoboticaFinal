#ifndef MATH_HPP_
#define MATH_HPP_

#include <cmath>
#include <iostream>


namespace math {

constexpr double pi_v = 3.14159265358979323846264338327950288;

template<typename T>
struct vec2 {
  constexpr vec2() : x {}, y {} { }

  constexpr explicit vec2(T num) : x(num), y(num) { }

  constexpr vec2(T num, T num2) : x(num), y(num2) { }

  constexpr vec2 operator+(vec2 const& other) const { return {other.x + x, other.y + y}; }

  constexpr vec2 operator-(vec2 const& other) const { return {other.x - x, other.y - y}; }

  constexpr void operator+=(vec2 const& other) const {
    x += other.x;
    y += other.y;
  }

  constexpr void operator-=(vec2 const& other) const {
    x -= other.x;
    y -= other.y;
  }

  constexpr bool operator==(vec2 const& other) const { return x == other.x && y == other.y; }

  T x, y;
};

template<typename T>
struct vec3 {
  constexpr vec3() : x {}, y {}, z {} { }

  constexpr explicit vec3(T num) : x(num), y(num), z(num) { }

  constexpr vec3(T num, T num2, T num3) : x(num), y(num2), z(num3) { }

  constexpr vec3 operator+(vec3 const& other) const { return {other.x + x, other.y + y, other.z + z}; }

  constexpr vec3 operator-(vec3 const& other) const { return {other.x - x, other.y - y, other.z - z}; }

  constexpr void operator+=(vec3 const& other) const {
    x += other.x;
    y += other.y;
    z += other.z;
  }
  constexpr void operator-=(vec3 const& other) const {
    x -= other.x;
    y -= other.y;
    z -= other.z;
  }
  constexpr bool operator==(vec3 const& other) const { return x == other.x && y == other.y && z == other.z; }

  // bool operator<=(const auto& other) const { return x <= other && y <= other && z <= other; }
  // bool operator>=(const auto& other) const { return x >= other && y >= other && z >= other; }

  T x, y, z;
};

template<typename T>
std::ostream& operator<<(std::ostream& os, vec2<T> const& v) {
  os << "(" << v.x << ", " << v.y << ")";
  return os;
}

template<typename T>
std::ostream& operator<<(std::ostream& os, vec3<T> const& v) {
  os << "(" << v.x << ", " << v.y << ", " << v.z << ")";
  return os;
}

constexpr inline double convert_bearing_to_degrees(double const* in_vector) {
  double const rad = atan2(in_vector[0], in_vector[2]);
  double const deg = rad * (180.0 / pi_v) + 180;

  return deg;
}

class Angle {
public:
  constexpr explicit Angle(double const* in_vector) : deg_(convert_bearing_to_degrees(in_vector)) { }

  template<typename T>
  constexpr explicit Angle(T const angle) :
    deg_(angle > 0 ? std::fmod(angle, 360) : 360 - std::abs(std::fmod(angle, 360))) { }

  explicit operator double() const { return deg_; }

  [[nodiscard]] double degrees() const { return deg_; }

  [[nodiscard]] double rad() const { return (deg_ - 180) / (180.0 / pi_v); }

  void degrees(double const degrees) {
    assert(angle > 0 and angle < 360);
    deg_ = degrees;
  }

  void rad(double const rad) { deg_ = (rad * (180.0 / pi_v)) + 180; }

  [[nodiscard]] double diff(Angle const other) const {
    double const diff = std::abs(deg_ - other.deg_);
    return std::min(diff, 360 - diff);
  }

  [[nodiscard]] double signedDiff(Angle const other) const {
    double const d = diff(other);
    if (*this + d == other)
      return d;
    return -d;
  }

  Angle operator+(Angle const& other) const { return Angle(deg_ + other.deg_); }

  Angle operator-(Angle const& other) const { return Angle(deg_ - other.deg_); }

  template<typename T>
  Angle operator+(T const& other) const {
    return *this + Angle(other);
  }

  template<typename T>
  Angle operator-(T const& other) const {
    return *this - Angle(other);
  }

  bool operator==(Angle const& other) const { return deg_ == other.deg_; }

  bool operator!=(Angle const& other) const { return deg_ != other.deg_; }

  bool operator<(Angle const& other) const { return deg_ < other.deg_; }

  bool operator>(Angle const& other) const { return deg_ > other.deg_; }

  bool operator<=(Angle const& other) const { return deg_ <= other.deg_; }

  bool operator>=(Angle const& other) const { return deg_ >= other.deg_; }

private:
  double deg_;
};

inline std::ostream& operator<<(std::ostream& os, Angle const& angle) {
  os << angle.degrees();
  return os;
}

constexpr double ir_to_distance(double const ir_value) { return (1024 - ir_value) * 0.2; }


} // namespace math

#endif
