#pragma once
#include <cmath>
#include <vector>

namespace math {
double inputModulus(double input, double minimumInput, double maximumInput);
template <typename T> const T &clamp(const T &v, const T &lo, const T &hi);
template <typename T> int sgn(T input);
template <typename T> T sum(std::vector<T> input);
template <typename T> std::vector<T> normalized(std::vector<T> input);
}; // namespace math
