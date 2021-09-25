#ifndef CARLA1S_ROS_CARLA_NAVIGATION_CARLA_NAV_COMMON_INCLUDE_MATH_MATH_H_
#define CARLA1S_ROS_CARLA_NAVIGATION_CARLA_NAV_COMMON_INCLUDE_MATH_MATH_H_

#include <algorithm>
#include <cmath>


namespace carla1s {
namespace math {

template<typename T>
T Clip(const T &n, const T &lower, const T &upper) {
  return std::max(lower, std::min(n, upper));
}

template<typename T>
T NormalizeAngle(const T angle) {
  using value_type = typename std::enable_if<std::is_same<float, T>::value || std::is_same<double, T>::value, T>::type;
  value_type a = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (a < 0.0) {
    a += (2.0 * M_PI);
  }
  return a - M_PI;
}

template<typename T>
T AngleDiff(const T from, const T to) {
  return NormalizeAngle(to - from);
}

}
}

#endif // CARLA1S_ROS_CARLA_NAVIGATION_CARLA_NAV_COMMON_INCLUDE_MATH_MATH_H_