#ifndef MATH_HPP_
#define MATH_HPP_

#include <vector>
#include <cmath>
#include <cstdint>
#include <iostream>


namespace utils {

constexpr double pi = 3.14159265358979323846264338327950288;

enum Direction { 
    left = 0,
    right, 
    back, 
    front
};


template<typename T>
struct vec2  { 
    vec2() : x{}, y{} {}
    explicit vec2(T num) : x(num), y (num) {}
    vec2(T num, T num2) : x(num), y (num2) {}

    vec2 operator+(const vec2& other) const { return {other.x + x, other.y + y}; }
    vec2 operator-(const vec2& other) const { return {other.x - x, other.y - y}; }
    void operator+=(const vec2& other) const { x += other.x; y += other.y; }
    void operator-=(const vec2& other) const { x -= other.x; y -= other.y; }
    bool operator==(const vec2& other) const { return x == other.x && y == other.y; }

    T x;
    T y;
};

template<typename T>
struct vec3  { 
    vec3() : x{}, y{}, z{} {}
    explicit vec3(T num) : x(num), y (num), z(num) {}
    vec3(T num, T num2, T num3) : x(num), y(num2), z(num3) {}

    vec3 operator+(const vec3& other) const { return {other.x + x, other.y + y, other.z + z}; }
    vec3 operator-(const vec3& other) const { return {other.x - x, other.y - y, other.z - z}; }
    void operator+=(const vec3& other) const { x += other.x; y += other.y; z += other.z; }
    void operator-=(const vec3& other) const { x -= other.x; y -= other.y; z -= other.z; }
    bool operator==(const vec3& other) const { return x == other.x && y == other.y && z == other.z; }
    // bool operator<=(const auto& other) const { return x <= other && y <= other && z <= other; }
    // bool operator>=(const auto& other) const { return x >= other && y >= other && z >= other; }

    T x; 
    T y; 
    T z;
};


enum Color { 
    white = 0,
    yellow,
};


inline double convert_bearing_to_degrees(const double* in_vector)
{
    double rad = atan2(in_vector[0], in_vector[2]);
    double deg = rad * (180.0 / pi);

    return deg;
}

inline double ir_to_distance(const double ir_value) { 
    return (1024 - ir_value) * 0.2;
}

}

#endif