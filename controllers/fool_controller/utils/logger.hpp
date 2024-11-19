/************************************************************************
 * Copyright (c) 2024 Alvaro Cabrera Barrio
 * This code is licensed under MIT license (see LICENSE.txt for details)
 ************************************************************************/
/**
 * @file logger.hpp
 * @version 1.0
 * @date 16/11/2024
 * @brief Short description
 *
 * Longer description
 */

#pragma once

#include <cstdint>
#include <iostream>
#include <sstream>

enum Log : uint8_t { controller = 0, robot, debug, separator };

inline static Log last_level = Log::robot;

template<Log lvl>
class Logger {
public:
  explicit Logger() {
    if constexpr (lvl == Log::robot) {
      buffer_ << "[Robot]: ";
      last_level = Log::robot;
    }
    else if constexpr (lvl == Log::controller) {
      buffer_ << "[Controller]: ";
      last_level = Log::controller;
    }
    else if constexpr (lvl == Log::separator) {
      if (last_level != Log::separator)
        buffer_ << "----------------------------------------------";
      last_level = Log::separator;
    }
    else {
      buffer_ << "[Debug]: ";
      last_level = Log::debug;
    }
  }

  template<typename T>
  Logger& operator<<(T const& value) {
    buffer_ << value;
    return *this;
  }

  ~Logger() {
    buffer_ << '\n';
    std::cout << buffer_.str();
  }

private:
  std::ostringstream buffer_;
};

#define logger(level) Logger<level>()
