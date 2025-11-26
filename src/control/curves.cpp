#include "control/curves.h"
#include <cmath>

double curves::linear(int axisPos) {
  return axisPos;
}

double curves::quadratic(int axisPos) {
  constexpr double QUADRATIC_STEEPNESS = 0.01;

  return copysign( QUADRATIC_STEEPNESS * axisPos * axisPos , axisPos);
}

double curves::exponential(int axisPos) {
  constexpr double EXPONENTIAL_STEEPNESS = 1.048;

  return copysign( pow(EXPONENTIAL_STEEPNESS, std::abs(axisPos)) - 1 , axisPos);
}

double curves::sigmoidal(int axisPos) {
  constexpr double K = 0.08;
  constexpr double MIDPOINT_X = 50.0;
  constexpr double SCALE_ADJUSTED = 103.7315; // Sigmoid scaled to hit points (0,0) and (100,100)
  constexpr double HEIGHT_ADJUSTED = -1.8657; // Cannot be calculated compile time due to std::exp not being const

  return copysign( SCALE_ADJUSTED / (1 + exp(-K * (std::abs(axisPos) - MIDPOINT_X))) + HEIGHT_ADJUSTED , axisPos);
}

double curves::sqrtSigmoidal(int axisPos) {
  constexpr double MIDPOINT_X = 50.0;
  constexpr double MIDPOINT_Y = 50.0;
  constexpr double VERTICAL_SCALE = 7.0711; // sqrt(50.0), cannot be calculated compile time due to std::sqrt not being const

  if (std::abs(axisPos) < MIDPOINT_X) {
    return copysign( VERTICAL_SCALE * -sqrt( -std::abs(axisPos - MIDPOINT_X) ) + MIDPOINT_Y , axisPos);
  } else {
    return copysign( VERTICAL_SCALE * sqrt( std::abs(axisPos) - MIDPOINT_X ) + MIDPOINT_Y , axisPos);
  }
}