
#include "traffic_light_perception.h"

int main(int argc, char** argv){
  ros::init(argc, argv, "traffic_light_node");
  TrafficLightPerception traffic_light_perception;
  ros::spin();
}