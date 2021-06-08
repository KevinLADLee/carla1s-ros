//
// Created by kevinlad on 2021/5/26.
//

#include <ros/ros.h>
#include "path_tracking.h"


int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "path_tracking_node");

  PathTracking path_tracking;

  ros::AsyncSpinner spinner(4); // Use multi threads
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
