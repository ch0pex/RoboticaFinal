#pragma once

#include "utils/logger.hpp"


template<typename T, typename R, typename Comparator>
void test_comparison(T left, R right, Comparator comp, char const* comp_name, char const* file, int line) {
  if (!comp(left, right)) {
    logger(Log::debug) << "Comparison failed: " << left << " " << comp_name << " " << right << " at " << file << ":"
                       << line;
  }
  else {
    logger(Log::debug) << "Comparison success: " << left << " " << comp_name << " " << right << " at " << file << ":"
                       << line;
  }
}

// Definición de macros específicas para diferentes tipos de comparación
#define CHECK_EQUALS(left, right) test_comparison(left, right, std::equal_to<>(), "==", __FILE__, __LINE__)
#define CHECK_NOT_EQUALS(left, right) test_comparison(left, right, std::not_equals_to<>(), "==", __FILE__, __LINE__)
#define CHECK_BIGGER(left, right) test_comparison(left, right, std::greater<>(), ">", __FILE__, __LINE__)
#define CHECK_BIGGER_EQUALS(left, right) test_comparison(left, right, std::greater_equal<>(), ">=", __FILE__, __LINE__)
#define CHECK_LOWER(left, right) test_comparison(left, right, std::less<>(), "<", __FILE__, __LINE__)
#define CHECK_LOWER_EQUALS(left, right) test_comparison(left, right, std::less_equal<>(), "<=", __FILE__, __LINE__)
