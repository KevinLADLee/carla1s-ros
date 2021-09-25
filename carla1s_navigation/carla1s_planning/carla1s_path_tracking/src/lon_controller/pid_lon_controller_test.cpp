
#include "pid_lon_controller.h"
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Int32.h"
#include "carla_msgs/CarlaEgoVehicleControl.h"

bool reverse = true;
//bool goal_received = false;
bool goal_received = true;
Pose2dPtr goal_pose;
Pose2dPtr vehicle_pose_;
double vehicle_speed_;

void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr &goal_msg){
  goal_pose->x = (*goal_msg).pose.position.x;
  goal_pose->y = (*goal_msg).pose.position.y;
  goal_pose->yaw = tf2::getYaw((*goal_msg).pose.orientation);
  goal_received = true;
}

void OdomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg){
  vehicle_pose_->x = (*odom_msg).pose.pose.position.x;
  vehicle_pose_->y = (*odom_msg).pose.pose.position.y;
  vehicle_pose_->yaw = tf2::getYaw((*odom_msg).pose.pose.orientation);
  vehicle_speed_ = std::sqrt((*odom_msg).twist.twist.linear.x * (*odom_msg).twist.twist.linear.x
                                 + (*odom_msg).twist.twist.linear.y * (*odom_msg).twist.twist.linear.y
                                 + (*odom_msg).twist.twist.linear.z * (*odom_msg).twist.twist.linear.z) * 3.6;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "test_pid_node");
  ros::NodeHandle nh;

  goal_pose = std::make_shared<Pose2d>();
  vehicle_pose_ = std::make_shared<Pose2d>();

  auto pid_controller = std::make_unique<PidLonController>();

  auto goal_sub = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, &GoalCallback);
  auto odom_sub = nh.subscribe<nav_msgs::Odometry>("/carla/ego_vehicle/odometry", 1, &OdomCallback);
  auto cmd_pub = nh.advertise<carla_msgs::CarlaEgoVehicleControl>("/carla/ego_vehicle/vehicle_control_cmd", 10);
  auto station_error_pub = nh.advertise<std_msgs::Int32>("pid_station_error", 1);
  auto speed_error_pub = nh.advertise<std_msgs::Int32>("pid_speed_error", 1);
  std_msgs::Int32 station_err_msg, speed_err_msg;

  carla_msgs::CarlaEgoVehicleControl cmd_msg;

  ros::Rate r(20);
  while (ros::ok()){
    ros::spinOnce();
    if(!goal_received){
      r.sleep();
      continue;
    }else{
//      std::cout << 15.0 - vehicle_speed_ << std::endl;
      double acc = 0.0;
//      if(reverse){
        cmd_msg.reverse = true;
        acc = pid_controller->RunStep(3.0, vehicle_speed_, 1.0 /20.0);
//      }else {
//        acc = pid_controller->RunStep(15.0, vehicle_speed_, 1.0 / 20.0);
//      }
//      acc = pid_controller->RunStep(vehicle_pose_, goal_pose, vehicle_speed_, 1.0 / 20.0);
      if(acc > 0){
        cmd_msg.throttle = acc;
        cmd_msg.brake = 0;
      }else{
        cmd_msg.throttle = 0;
        cmd_msg.brake = acc;
      }
      cmd_pub.publish(cmd_msg);
      station_err_msg.data = 100*(pid_controller->GetStationError());
      speed_err_msg.data = 100*(pid_controller->GetSpeedError());
      speed_error_pub.publish(speed_err_msg);
      station_error_pub.publish(station_err_msg);
    }
  }

}