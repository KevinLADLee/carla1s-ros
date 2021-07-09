
#ifndef CARLA1S_ROS_CARLA_NAVIGATION_CARLA_FAKE_PERCEPTION_SRC_TRAFFIC_LIGHT_PERCEPTION_H_
#define CARLA1S_ROS_CARLA_NAVIGATION_CARLA_FAKE_PERCEPTION_SRC_TRAFFIC_LIGHT_PERCEPTION_H_

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>

#include <carla_msgs/CarlaTrafficLightInfoList.h>
#include <carla_msgs/CarlaTrafficLightStatusList.h>
#include <carla_msgs/CarlaEgoVehicleInfo.h>

#include <tf/tf.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>

template<typename T>
int FindElementById(std::vector<T> t_vec, unsigned int id){
  auto it = std::find_if(t_vec.begin(),
                         t_vec.end(),
                         [&](const T & p) {
                           return p.id == id;
                         });
  if(it != t_vec.end()){
    return it - t_vec.begin();
  } else{
    return -1;
  }
}

struct Box{
  Box(const tf::Transform &box_trans_in_map,
      const geometry_msgs::Vector3 &size) : box_trans(box_trans_in_map),
                                            size(size) {};
  tf::Transform box_trans;
  geometry_msgs::Vector3 size;
  tf::Vector3 cornor_min;
  tf::Vector3 cornor_max;
};

//static void BoxToCornorVector(const Box &box, tf::Vector3 cornor_vec_min, tf::Vector3 cornor_vec_max){
//
//}

struct TrafficLight{
  enum Status{
    RED = 0u,
    YELLOW = 1u,
    GREEN = 2u,
    OFF = 3u,
    UNKNOWN = 4u,
  };
  TrafficLight() = default;
  TrafficLight(unsigned int id,
               tf::Transform transform,
               tf::Transform box_tranform,
               geometry_msgs::Vector3 trigger_volume_size) : id(id), transform(transform), passable(false), box(Box(box_tranform, trigger_volume_size)){};
  unsigned int id = 0;
  tf::Transform transform;
  Box box;
  bool passable = false;
};


class TrafficLightPerception {
 public:
  TrafficLightPerception();

  void OdomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg);

  void TrafficLightInfoCallback(const carla_msgs::CarlaTrafficLightInfoListConstPtr &info_msg);

  void TrafficLightStatusCallback(const carla_msgs::CarlaTrafficLightStatusListConstPtr &status_msg);

 private:

  bool CheckPassable();

  void InitMarkerColors();

  tf::Transform BoxinTrafficToMap(const geometry_msgs::Vector3 &box_pos_in_traffic, const tf::Transform &tl_transform_in_map);

  void CreateMarker(const TrafficLight& tl);

  void UpdateMarker(unsigned int id, unsigned char status);

  static tf::Transform PoseMsgToTfTransform(const geometry_msgs::Pose & pose);


 private:
  ros::NodeHandle nh_;
  std::string role_name_ = "ego_vehicle";
  double vehicle_scale_x = 2.5;
  double vehicle_scale_y = 1.2;

  ros::Subscriber odom_sub_;
  nav_msgs::Odometry odom_;

  ros::Subscriber traffic_lights_info_sub_, traffic_lights_status_sub_;
  bool traffic_lights_info_received_ = false;
  std::vector<TrafficLight> traffic_lights_;

  ros::Publisher tl_passable_pub_;
  std_msgs::Bool tl_passable_msg_;

  bool publish_viz_ = true;
  ros::Publisher tl_viz_pub_;
  visualization_msgs::MarkerArray tl_viz_marker_vec_msgs_;

  std::map<unsigned char, std_msgs::ColorRGBA> color_map_;
};

#endif //CARLA1S_ROS_CARLA_NAVIGATION_CARLA_FAKE_PERCEPTION_SRC_TRAFFIC_LIGHT_PERCEPTION_H_
