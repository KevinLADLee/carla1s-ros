#include <iostream>
#include <algorithm>
#include <cmath>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <visualization_msgs/Marker.h>

#include <thread>
#include <mutex>
#include <condition_variable>

#include <carla_nav_msgs/TrackingPathAction.h>
#include <carla_msgs/CarlaEgoVehicleInfo.h>

/********************/
/* CLASS DEFINITION */
/********************/
class PurePursuit
{
  enum NodeState{
    IDLE,
    RUNNING,
    PAUSE,
    SUCCESS,
    FAILURE
  };
 public:
  PurePursuit(const std::string &name);
  void initMarker();

  void trackingLoop();
  NodeState getNodeState();
  void setNodeState(const NodeState & node_state);
  void ExecuteCallback(const carla_nav_msgs::TrackingPathGoal::ConstPtr & goal_msg);
  bool isGoalReached(const nav_msgs::Odometry &odom);
  double computeDistToGoal(const nav_msgs::Odometry &odom);

  void startTrackingPath();
  void stopTrackingPath();
  void vehicleInfoCB(const carla_msgs::CarlaEgoVehicleInfoConstPtr &vehicle_info_msg);
  void odomCB(const nav_msgs::Odometry::ConstPtr &odom_msg);

 private:
  bool isForwardWayPt(const geometry_msgs::Point &way_pt, const geometry_msgs::Pose &car_pose);
  bool isWayPtAwayFromLfwDist(const geometry_msgs::Point &way_pt, const geometry_msgs::Point &car_pos);
  double getYawFromPose(const geometry_msgs::Pose &car_pose);
  double getEta(const geometry_msgs::Pose &car_pose);
  double getCar2GoalDist();
  double getSteering(double eta) const;
  geometry_msgs::Point get_odom_car2WayPtVec(const geometry_msgs::Pose &car_pose);


 private:
  ros::NodeHandle nh_;
  ros::Subscriber odom_sub_, vehicle_info_sub_;
  ros::Publisher cmd_pub_, marker_pub_;

  actionlib::SimpleActionServer<carla_nav_msgs::TrackingPathAction> as_;
  NodeState node_state_ = NodeState::IDLE;
  std::mutex plan_mutex_, node_state_mutex_, odom_mutex_;
  std::condition_variable plan_condition_;
  std::thread tracking_path_thread_;

  visualization_msgs::Marker points_, line_strip_, goal_circle_;
  geometry_msgs::Point odom_goal_pos_, goal_pos_;
  geometry_msgs::Twist cmd_vel_;
  ackermann_msgs::AckermannDrive ackermann_cmd_;
  nav_msgs::Odometry odom_;
  nav_msgs::Path map_path_, odom_path_;

  bool use_vehicle_info_ = true;

  float L_, Lfw_, Vcmd_, lfw_, steering_, velocity_, Vmin_;
  float brake_dist_;
  float tire_friction_ = 3.0;
  float steering_gain_, base_angle_, goal_radius_, speed_incremental_;
  int controller_freq_;
  bool found_forward_pt_, smooth_accel_;
  bool goal_reached_;


}; // end of class

PurePursuit::PurePursuit(const std::string &name)
    : nh_{ros::NodeHandle()}, cmd_vel_{geometry_msgs::Twist()}, as_(nh_, "tracking_path_action", boost::bind(&PurePursuit::ExecuteCallback, this, _1), false)
{
  ros::NodeHandle pn("~/" + name);
  pn.param<float>("L", L_, 2.77);      // length of car
  pn.param<float>("Vcmd", Vcmd_, 15.0); // reference speed (m/s)
  pn.param<float>("Lfw", Lfw_, 5.0);   // forward look ahead distance (m)
  pn.param<float>("lfw", lfw_, 1.6614);  // distance between front the center of car
  pn.param<float>("Vmin", Vmin_, 2.0); // reference speed (m/s)
  pn.param<float>("BrakeDistance", brake_dist_, 15.0); // reference speed (m/s)

  //Controller parameter
  pn.param<int>("controller_freq", controller_freq_, 50);
  pn.param<float>("steering_gain", steering_gain_, 3.0);
  pn.param<float>("goal_radius", goal_radius_, 2.0);             // goal radius (m)
  pn.param<float>("base_angle", base_angle_, 0.0);               // neutral point of servo (rad)
  pn.param<bool>("smooth_accel", smooth_accel_, true);          // smooth the acceleration of car
  pn.param<float>("speed_incremental", speed_incremental_, 1.5); // speed incremental value (discrete acceleraton), unit: m/s

  //Publishers and Subscribers
  odom_sub_ = nh_.subscribe("/carla/ego_vehicle/odometry", 1, &PurePursuit::odomCB, this);
  vehicle_info_sub_ = nh_.subscribe("/carla/ego_vehicle/vehicle_info", 1, &PurePursuit::vehicleInfoCB, this);

  marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/path_tracking_node/pure_pursuit/path_marker", 5);
  cmd_pub_ = nh_.advertise<ackermann_msgs::AckermannDrive>("/carla/ego_vehicle/ackermann_cmd", 10);

  //Action server
  as_.start();
  node_state_ = NodeState::IDLE;

  //Init variables
  found_forward_pt_ = false;
  goal_reached_ = false;
  velocity_ = 0.0;
  steering_ = base_angle_;

  //Show info
  ROS_INFO("[param] base_angle: %f", base_angle_);
  ROS_INFO("[param] Vcmd: %f", Vcmd_);
  ROS_INFO("[param] Lfw: %f", Lfw_);

  //Visualization Marker Settings
  initMarker();
}

void PurePursuit::initMarker()
{
  points_.header.frame_id = line_strip_.header.frame_id = goal_circle_.header.frame_id = "map";
  points_.ns = line_strip_.ns = goal_circle_.ns = "Markers";
  points_.action = line_strip_.action = goal_circle_.action = visualization_msgs::Marker::ADD;
  points_.pose.orientation.w = line_strip_.pose.orientation.w = goal_circle_.pose.orientation.w = 1.0;
  points_.id = 0;
  line_strip_.id = 1;
  goal_circle_.id = 2;

  points_.type = visualization_msgs::Marker::POINTS;
  line_strip_.type = visualization_msgs::Marker::LINE_STRIP;
  goal_circle_.type = visualization_msgs::Marker::CYLINDER;
  // POINTS markers use x and y scale for width/height respectively
  points_.scale.x = 0.2;
  points_.scale.y = 0.2;

  //LINE_STRIP markers use only the x component of scale, for the line width
  line_strip_.scale.x = 0.1;

  goal_circle_.scale.x = goal_radius_;
  goal_circle_.scale.y = goal_radius_;
  goal_circle_.scale.z = 0.1;

  // Points are green
  points_.color.g = 1.0f;
  points_.color.a = 1.0;

  // Line strip is blue
  line_strip_.color.b = 1.0;
  line_strip_.color.a = 1.0;

  //goal_circle is yellow
  goal_circle_.color.r = 1.0;
  goal_circle_.color.g = 1.0;
  goal_circle_.color.b = 0.0;
  goal_circle_.color.a = 0.5;
}

void PurePursuit::odomCB(const nav_msgs::Odometry::ConstPtr &odom_msg)
{
  std::lock_guard<std::mutex> guard(odom_mutex_);
  this->odom_ = *odom_msg;
  isGoalReached(odom_);
}

double PurePursuit::getYawFromPose(const geometry_msgs::Pose &car_pose)
{
  double x = car_pose.orientation.x;
  double y = car_pose.orientation.y;
  double z = car_pose.orientation.z;
  double w = car_pose.orientation.w;

  double tmp, yaw;
  tf::Quaternion q(x, y, z, w);
  tf::Matrix3x3 quaternion(q);
  quaternion.getRPY(tmp, tmp, yaw);

  return yaw;
}

bool PurePursuit::isForwardWayPt(const geometry_msgs::Point &way_pt, const geometry_msgs::Pose &car_pose)
{
  float car2way_pt_x = way_pt.x - car_pose.position.x;
  float car2way_pt_y = way_pt.y - car_pose.position.y;
  double car_theta = getYawFromPose(car_pose);

  float car_car2wayPt_x = std::cos(car_theta) * car2way_pt_x + std::sin(car_theta) * car2way_pt_y;
  float car_car2wayPt_y = -std::sin(car_theta) * car2way_pt_x + std::cos(car_theta) * car2way_pt_y;

  if (car_car2wayPt_x > 0) /*is Forward WayPt*/
    return true;
  else
    return false;
}

bool PurePursuit::isWayPtAwayFromLfwDist(const geometry_msgs::Point &way_pt, const geometry_msgs::Point &car_pos)
{
  double dist = std::hypot(way_pt.x - car_pos.x, way_pt.y - car_pos.y);

  if (dist >= Lfw_) {
    return true;
  } else{
    return false;
  }
}

geometry_msgs::Point PurePursuit::get_odom_car2WayPtVec(const geometry_msgs::Pose &car_pose)
{
  geometry_msgs::Point carPose_pos = car_pose.position;
  double carPose_yaw = getYawFromPose(car_pose);
  geometry_msgs::Point forwardPt;
  geometry_msgs::Point odom_car2WayPtVec;
  found_forward_pt_ = false;

  if (!goal_reached_)
  {
    for (int i = 0; i < map_path_.poses.size(); i++)
    {
      geometry_msgs::PoseStamped map_path_pose = map_path_.poses[i];

      geometry_msgs::Point odom_path_wayPt = map_path_pose.pose.position;
      bool _isForwardWayPt = isForwardWayPt(odom_path_wayPt, car_pose);

      if (_isForwardWayPt)
      {
        bool _isWayPtAwayFromLfwDist = isWayPtAwayFromLfwDist(odom_path_wayPt, carPose_pos);
        if (_isWayPtAwayFromLfwDist)
        {
          forwardPt = odom_path_wayPt;
          found_forward_pt_ = true;
          break;
        }
      }
    }
  }
  else if (goal_reached_)
  {
    forwardPt = odom_goal_pos_;
    found_forward_pt_ = false;
    //ROS_INFO("goal REACHED!");
  }

  points_.points.clear();
  line_strip_.points.clear();

  if (found_forward_pt_ && !goal_reached_)
  {
    points_.points.push_back(carPose_pos);
    points_.points.push_back(forwardPt);
    line_strip_.points.push_back(carPose_pos);
    line_strip_.points.push_back(forwardPt);
  }

  marker_pub_.publish(points_);
  marker_pub_.publish(line_strip_);

  odom_car2WayPtVec.x = cos(carPose_yaw) * (forwardPt.x - carPose_pos.x) + sin(carPose_yaw) * (forwardPt.y - carPose_pos.y);
  odom_car2WayPtVec.y = -sin(carPose_yaw) * (forwardPt.x - carPose_pos.x) + cos(carPose_yaw) * (forwardPt.y - carPose_pos.y);
  return odom_car2WayPtVec;
}

double PurePursuit::getEta(const geometry_msgs::Pose &carPose)
{
  geometry_msgs::Point odom_car2WayPtVec = get_odom_car2WayPtVec(carPose);
  return atan2(odom_car2WayPtVec.y, odom_car2WayPtVec.x);
}

double PurePursuit::getCar2GoalDist()
{
  geometry_msgs::Point car_pose = odom_.pose.pose.position;
  double car2goal_x = odom_goal_pos_.x - car_pose.x;
  double car2goal_y = odom_goal_pos_.y - car_pose.y;

  return sqrt(car2goal_x * car2goal_x + car2goal_y * car2goal_y);
}

double PurePursuit::getSteering(double eta) const
{
  return atan2((this->L_ * sin(eta)), (this->Lfw_ / 2 + this->lfw_ * cos(eta)));
}

void PurePursuit::vehicleInfoCB(const carla_msgs::CarlaEgoVehicleInfoConstPtr &vehicle_info_msg) {
  if(use_vehicle_info_){
    auto wheels = vehicle_info_msg->wheels;
    L_ = std::abs(wheels.at(0).position.x - wheels.at(2).position.x);
    lfw_ = std::abs(wheels.at(0).position.x);
    tire_friction_ = vehicle_info_msg->wheels.at(0).tire_friction;
    std::cout << "VehicleInfo: wheel_base = " << L_ << " wheel front to center = " << lfw_  << std::endl;
  }
  use_vehicle_info_ = false;
}

PurePursuit::NodeState PurePursuit::getNodeState() {
  std::lock_guard<std::mutex> guard(node_state_mutex_);
  return node_state_;
}

void PurePursuit::setNodeState(const NodeState & node_state) {
  std::lock_guard<std::mutex> guard(node_state_mutex_);
  node_state_ = node_state;
}

void PurePursuit::ExecuteCallback(const carla_nav_msgs::TrackingPathGoal::ConstPtr & goal_msg) {

//  std::cout << "PurePursuit: got " << goal_msg->path.poses.size() << " waypoints" << std::endl;

  if(goal_msg->path.poses.size() <=1 ){
    setNodeState(NodeState::FAILURE);
  } else{
    setNodeState(NodeState::IDLE);
  }

  NodeState node_state = getNodeState();

  if(node_state == NodeState::FAILURE){
    carla_nav_msgs::TrackingPathResult result;
    result.error_code = NodeState::FAILURE;
    as_.setAborted(result, "Tracking path failed!");
    return;
  }

  if(plan_mutex_.try_lock()){
    map_path_ = goal_msg->path;
      goal_pos_ = map_path_.poses.back().pose.position;
      goal_circle_.pose = map_path_.poses.back().pose;
      plan_mutex_.unlock();
      plan_condition_.notify_one();
  }
  marker_pub_.publish(goal_circle_);

  ROS_INFO("Start tracking!");
  if(node_state == NodeState::IDLE){
    startTrackingPath();
  }

  while (ros::ok()) {
    std::this_thread::sleep_for(std::chrono::microseconds(1));

    if (as_.isPreemptRequested()) {
      ROS_INFO("Action Preempted");
      stopTrackingPath();
      setNodeState(NodeState::IDLE);
      carla_nav_msgs::TrackingPathResult result;
      result.error_code = NodeState::SUCCESS;
      goal_reached_ = false;
      as_.setPreempted(result);
      break;
    }
    node_state = getNodeState();

    if (node_state == NodeState::RUNNING|| node_state == NodeState::SUCCESS
        || node_state == NodeState::FAILURE) {
      carla_nav_msgs::TrackingPathFeedback feedback;
      carla_nav_msgs::TrackingPathResult result;

      feedback.error_code = node_state;
      as_.publishFeedback(feedback);

      if(node_state == NodeState::SUCCESS) {
        result.error_code = node_state;
        as_.setSucceeded(result,"Tracking succeed!");
        stopTrackingPath();
        break;
      } else if(node_state == NodeState::FAILURE) {
        result.error_code = node_state;
        as_.setAborted(result, "Error!");
        stopTrackingPath();
        break;
      }
    }
  }

  std::cout << "end execute loop!" << std::endl;
}

void PurePursuit::startTrackingPath() {
  if(tracking_path_thread_.joinable()){
    tracking_path_thread_.join();
  }
  setNodeState(NodeState::RUNNING);
  tracking_path_thread_ = std::thread([this] { trackingLoop(); });

}

void PurePursuit::stopTrackingPath() {
  setNodeState(NodeState::IDLE);
  if(tracking_path_thread_.joinable()){
    tracking_path_thread_.join();
  }
  std::cout << "stop tracking path" << std::endl;
}

bool PurePursuit::isGoalReached(const nav_msgs::Odometry &odom) {
  double dist2goal = computeDistToGoal(odom);
  if (dist2goal < goal_radius_)
  {
    goal_reached_ = true;
    return true;
  } else{
    goal_reached_ = false;
    return false;
  }
}

double PurePursuit::computeDistToGoal(const nav_msgs::Odometry &odom) {
  double dist2goal = std::hypot(goal_pos_.x - odom.pose.pose.position.x,
                                goal_pos_.y - odom.pose.pose.position.y);
  return dist2goal;
}

void PurePursuit::trackingLoop() {

  std::chrono::microseconds sleep_time = std::chrono::microseconds(0);
  while (getNodeState() == NodeState::RUNNING){
//    std::cout << "start tracking loop" << std::endl;
    std::unique_lock<std::mutex> plan_lock(plan_mutex_);
//    std::cout << "wait condition!" << std::endl;
    plan_condition_.wait_for(plan_lock, sleep_time);
    auto begin_time = std::chrono::steady_clock::now();

    geometry_msgs::Pose carPose;
    geometry_msgs::Twist carVel;
    {
      std::lock_guard<std::mutex> guard(odom_mutex_);
      carPose = odom_.pose.pose;
      carVel = odom_.twist.twist;
//      std::cout << "got odom!" << std::endl;
    }

    double eta = getEta(carPose);
    if (found_forward_pt_)
    {
      steering_ = base_angle_ + getSteering(eta) * steering_gain_;
      velocity_ = std::min(velocity_ + speed_incremental_, Vcmd_);
    }

    if(computeDistToGoal(odom_) <= brake_dist_){
      velocity_ = Vmin_;
    }

    auto cost_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - begin_time);
    int need_time = 1000 / controller_freq_;
    sleep_time = std::chrono::milliseconds(need_time) - cost_time;
    if (sleep_time <= std::chrono::milliseconds(0)) {
      sleep_time = std::chrono::milliseconds(0);
    }

    ackermann_cmd_.speed = velocity_;
    ackermann_cmd_.steering_angle = steering_;
    cmd_pub_.publish(ackermann_cmd_);
    if(isGoalReached(odom_)){
      std::cout << "reached goal!" << std::endl;
      setNodeState(NodeState::SUCCESS);
    }

  }
  std::cout << "stop tracking loop" << std::endl;
  ackermann_cmd_.speed = 0.0;
  ackermann_cmd_.steering_angle = base_angle_;
  cmd_pub_.publish(ackermann_cmd_);

}


/*****************/
/* MAIN FUNCTION */
/*****************/
int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "pure_pursuit_node");
  PurePursuit controller("pure_pursuit");
  ros::AsyncSpinner spinner(2); // Use multi threads
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
