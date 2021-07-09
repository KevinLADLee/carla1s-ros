
#include "carla_maps.h"

int main(int argc, char** argv){
  ros::init(argc, argv, "traffic_light_node");
  CarlaMaps carla_maps;
  ros::spin();
}