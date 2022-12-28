#include "../include/math.hpp"
#include <vector>

namespace math {

double inputModulus(double input, double minimumInput, double maximumInput) {
  double modulus = maximumInput - minimumInput;

  // Wrap input if it's above the maximum input
  int numMax = (int)((input - minimumInput) / modulus);
  input -= numMax * modulus;

  // Wrap input if it's below the minimum input
  int numMin = (int)((input - maximumInput) / modulus);
  input -= numMin * modulus;

  return input;
}

template <typename T> const T &clamp(const T &v, const T &lo, const T &hi) {
  assert(!(hi < lo));
  return v < lo ? lo : hi < v ? hi : v;
}

template <typename T> int sgn(T input) {
  return (T(0) < input) - (T(0) > input);
}

template <typename T> T sum(std::vector<T> input) {
  T total = 0;
  for (int i = 0; i < input.size(); i++) {
    total += input.at(i);
  }
  return total;
}

template <typename T> std::vector<T> normalized(std::vector<T> input) {
  T max = fmax_vector(input);
  for (int i = 0; i < input.size(); i++) {
    input[i] = input[i] / max;
  }
  return input;
}

template <typename T> T fmax_vector(std::vector<T> input) {
  T max = input.at(0);
  for (int i = 0; i < input.size(); i++) {
    max = fmax(max, input.at(i));
  }
  return max;
}

}; // namespace math
