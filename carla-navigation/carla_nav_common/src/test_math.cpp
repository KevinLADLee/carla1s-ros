
#include "carla_nav_math/math_utils.h"
#include <iostream>

int main(int argc, char** argv){

  double angle_1 = M_PI;
  double angle_2 = 2.0 * M_PI;

  std::cout << carla1s::math::NormalizeAngle(angle_1 + angle_2) << std::endl;
//  std::cout << carla1s::math::NormalizeAngle(static_cast<int>(angle_1 + angle_2)) << std::endl;

  return 0;
}