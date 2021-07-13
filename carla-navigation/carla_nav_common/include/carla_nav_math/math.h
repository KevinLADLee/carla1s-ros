#ifndef CARLA1S_ROS_CARLA_NAVIGATION_CARLA_NAV_COMMON_INCLUDE_MATH_MATH_H_
#define CARLA1S_ROS_CARLA_NAVIGATION_CARLA_NAV_COMMON_INCLUDE_MATH_MATH_H_

template <typename T>
T clip(const T& n, const T& lower, const T& upper) {
  return std::max(lower, std::min(n, upper));
}

#endif // CARLA1S_ROS_CARLA_NAVIGATION_CARLA_NAV_COMMON_INCLUDE_MATH_MATH_H_