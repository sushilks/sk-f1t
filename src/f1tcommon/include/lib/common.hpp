#pragma once
#include <math.h>
namespace common {
/**
 * @brief Convert radians to degrees
 * @param r radians
 * @return degrees
 */
float inline rtod(float r) {
  return r * 180 / M_PI;
}

/**
 * @brief Convert degrees to radians
 * @param r degrees
 * @return radians
 */
float inline dtor(float r) {
  return r * M_PI / 180.0;
}
}
