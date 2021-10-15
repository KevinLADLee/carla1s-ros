

#include "object_detection.h"

ObjectDetection::ObjectDetection() {

  auto success = LoadParam();
  if(!success){
    return;
  }

  using namespace message_filters;

  odom_sub_ = std::make_shared<message_filters::Subscriber<nav_msgs::Odometry>>(nh_, carla_topic_prefix+"/odometry", 1);
  objects_sub_ = std::make_shared<message_filters::Subscriber<derived_object_msgs::ObjectArray>>(nh_, "/carla/objects", 1);

  msg_sync_ = std::make_shared<Synchronizer<OdomObjectsSyncPolicy>>(OdomObjectsSyncPolicy(10), *odom_sub_, *objects_sub_);
  msg_sync_->registerCallback(boost::bind(&ObjectDetection::OdomObjectsCallback, this, _1, _2));

  speed_pub_ = nh_.advertise<std_msgs::Float64>(carla1s_topic_prefix+"/collision_avoid_speed",1);
  object_pub_ = nh_.advertise<derived_object_msgs::Object>(carla1s_topic_prefix+"/detected_object", 1);
  viz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(carla1s_topic_prefix+"/detected_object_marker", 1);
  InitMarker();

}

bool ObjectDetection::LoadParam() {
  ros::NodeHandle ph("~");
  ph.param<std::string>("role_name", role_name, "ego_vehicle");
  carla_topic_prefix = "/carla/"+role_name;
  carla1s_topic_prefix = "/carla1s/"+role_name+"/fake_perception";

  ph.param<double>("max_distance", max_distance, 50.0);
  max_dist_square = max_distance * max_distance;

  auto vehicle_info = ros::topic::waitForMessage<carla_msgs::CarlaEgoVehicleInfo>("/carla/"+role_name+"/vehicle_info");
  vehicle_id = vehicle_info->id;

  return true;
}

void ObjectDetection::OdomObjectsCallback(const nav_msgs::OdometryConstPtr &odom_ptr,
                                          const derived_object_msgs::ObjectArrayConstPtr &object_array_ptr){

  auto vehicle_twist = odom_ptr->twist;
  tf::Transform vehicle_trans;
  tf::poseMsgToTF(odom_ptr->pose.pose, vehicle_trans);
  auto vehicle_trans_inv = vehicle_trans.inverse();

  auto min_dist_square = max_distance * max_distance;
  auto min_index = -1;
  for(int i = 0; i < object_array_ptr->objects.size(); i++) {
    if(object_array_ptr->objects[i].id == vehicle_id){
      continue;
    }

    auto dist_square = RosPoseDistanceSquare(odom_ptr->pose.pose,
                                             object_array_ptr->objects[i].pose);
    if(dist_square > max_dist_square){
      continue;
    }

    tf::Pose object_pose;
    tf::poseMsgToTF(object_array_ptr->objects[i].pose, object_pose);
    auto object_pose_in_vehicle = vehicle_trans_inv * object_pose;
    auto object_sin = std::sin(object_pose_in_vehicle.getRotation().getAngle());
    if(object_pose_in_vehicle.getOrigin().x() < 25
        && object_pose_in_vehicle.getOrigin().x() > 0
        && std::abs(object_pose_in_vehicle.getOrigin().y()) < 1.5
        && object_sin > 0){
      min_dist_square = min_dist_square;
      min_index = i;
    }
  }

  if(min_index < 0){
    speed_msg_.data = -1.0;
    speed_pub_.publish(speed_msg_);
    return;
  }
  current_object_ = object_array_ptr->objects[min_index];
  object_pub_.publish(current_object_);
  ROS_INFO("ObjectDetection: Found object! Pose (%f, %f, %f), Speed: (%f) km/h",
           current_object_.pose.position.x,
           current_object_.pose.position.y,
           current_object_.pose.position.z,
           TwistToVehicleSpeed(current_object_.twist) * 3.6);


  speed_msg_.data = SimpleCollisionAvoid(odom_ptr, current_object_);
  speed_pub_.publish(speed_msg_);
  PublishMarker(current_object_);
}

double ObjectDetection::SimpleCollisionAvoid(const nav_msgs::OdometryConstPtr &odom_ptr,
                                          const derived_object_msgs::Object &object) {
  auto ego_pose = odom_ptr->pose.pose;
  auto front_pose = object.pose;
  auto dist = std::sqrt(RosPoseDistanceSquare(ego_pose, front_pose));

  auto front_speed = TwistToVehicleSpeed(object.twist);

  if(dist < safe_dist){
    return front_speed;
  }

//  auto ego_speed = TwistToVehicleSpeed(odom_ptr->twist.twist);
//  auto safe_speed = front_speed - ((safe_dist - dist) / dt );
//  safe_speed = std::max(safe_speed / 3.6, 0.0);
//  return safe_speed;
}

double ObjectDetection::RosPoseDistanceSquare(const geometry_msgs::Pose &pose1, const geometry_msgs::Pose &pose2) {
  auto dx = pose1.position.x - pose2.position.x;
  auto dy = pose1.position.y - pose2.position.y;
  auto dz = pose1.position.z - pose2.position.z;
  return (dx * dx + dy * dy + dz * dz);
}

double ObjectDetection::TwistToVehicleSpeed(const geometry_msgs::Twist &twist) {
  return std::sqrt(twist.linear.x * twist.linear.x
                       + twist.linear.y * twist.linear.y
                       + twist.linear.z * twist.linear.z);
}

void ObjectDetection::InitMarker() {
  using namespace visualization_msgs;
  Marker box_marker;
  box_marker.header.frame_id = "map";
  box_marker.ns = role_name+"/object_marker";
  box_marker.action = Marker::ADD;
  box_marker.type = Marker::CUBE;
  box_marker.id = 0;
  box_marker.color.r = 1.0;
  box_marker.color.a = 1.0;
  box_marker.lifetime = ros::Duration(0.5);
  marker_array_.markers.push_back(box_marker);

  Marker text_marker;
  text_marker.header.frame_id = "map";
  text_marker.ns = role_name+"/object_marker";
  text_marker.action = Marker::ADD;
  text_marker.type = Marker::TEXT_VIEW_FACING;
  text_marker.id = 1;
  text_marker.color.r = 0.5;
  text_marker.color.a = 0.9;
  text_marker.lifetime = ros::Duration(0.5);
  marker_array_.markers.push_back(text_marker);
}

void ObjectDetection::PublishMarker(const derived_object_msgs::Object &object) {
  auto box_marker = marker_array_.markers.begin();
  box_marker->pose = object.pose;
  box_marker->scale.x = object.shape.dimensions[0];
  box_marker->scale.y = object.shape.dimensions[1];
  box_marker->scale.z = object.shape.dimensions[2];
  auto text_marker = box_marker+1;
  text_marker->pose = object.pose;
  text_marker->pose.position.z += 2.0;
  text_marker->scale.z = object.shape.dimensions[2];
  text_marker->text = std::to_string((int)(TwistToVehicleSpeed(object.twist) * 3.6)) + " km/h";

  viz_pub_.publish(marker_array_);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "object_detection_node");
  auto object_detection_ptr = std::make_unique<ObjectDetection>();
  ros::spin();
}