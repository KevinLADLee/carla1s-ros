#include <iostream>
#include <algorithm>
#include <cmath>
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <visualization_msgs/Marker.h>

#include <carla_msgs/CarlaEgoVehicleInfo.h>

/********************/
/* CLASS DEFINITION */
/********************/
class PurePursuit
{
public:
    PurePursuit(const std::string &name);
    void initMarker();
    bool isForwardWayPt(const geometry_msgs::Point &way_pt, const geometry_msgs::Pose &car_pose);
    bool isWayPtAwayFromLfwDist(const geometry_msgs::Point &way_pt, const geometry_msgs::Point &car_pos);
    double getYawFromPose(const geometry_msgs::Pose &car_pose);
    double getEta(const geometry_msgs::Pose &car_pose);
    double getCar2GoalDist();
    double getSteering(double eta);
    geometry_msgs::Point get_odom_car2WayPtVec(const geometry_msgs::Pose &car_pose);

private:
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_, path_sub_, vehicle_info_sub_;
    ros::Publisher cmd_pub_, marker_pub_;
    ros::Timer timer1_, timer2_;

    visualization_msgs::Marker points_, line_strip_, goal_circle_;
    geometry_msgs::Point odom_goal_pos_, goal_pos_;
    geometry_msgs::Twist cmd_vel_;
    ackermann_msgs::AckermannDrive ackermann_cmd_;
    nav_msgs::Odometry odom_;
    nav_msgs::Path map_path_, odom_path_;

    bool use_vehicle_info_ = true;

    double L_, Lfw_, Vcmd_, lfw_, steering_, velocity_;
    double steering_gain_, base_angle_, goal_radius_, speed_incremental_;
    int controller_freq_;
    bool found_forward_pt_, smooth_accel_;
    bool goal_received_, goal_reached_;

    void vehicleInfoCB(const carla_msgs::CarlaEgoVehicleInfoConstPtr &vehicle_info_msg);
    void odomCB(const nav_msgs::Odometry::ConstPtr &odom_msg);
    void pathCB(const nav_msgs::Path::ConstPtr &path_msg);
    void controlLoopCB(const ros::TimerEvent &);

}; // end of class

PurePursuit::PurePursuit(const std::string &name)
    : nh_{ros::NodeHandle()}, cmd_vel_{geometry_msgs::Twist()}
{
    //Private parameters handler
    ros::NodeHandle pn("~/" + name);

    // TODO: Optimize parameters setting
    //Car parameter
    pn.param("L", L_, 2.77);      // length of car
    pn.param("Vcmd", Vcmd_, 20.0); // reference speed (m/s)
    pn.param("Lfw", Lfw_, 5.0);   // forward look ahead distance (m)
    pn.param("lfw", lfw_, 1.6614);  // distance between front the center of car

    //Controller parameter
    pn.param("controller_freq", controller_freq_, 50);
    pn.param("steering_gain", steering_gain_, 3.0);
    pn.param("goal_radius", goal_radius_, 2.0);             // goal radius (m)
    pn.param("base_angle", base_angle_, 0.0);               // neutral point of servo (rad)
    pn.param("smooth_accel", smooth_accel_, true);          // smooth the acceleration of car
    pn.param("speed_incremental", speed_incremental_, 1.5); // speed incremental value (discrete acceleraton), unit: m/s

    //Publishers and Subscribers
    odom_sub_ = nh_.subscribe("/carla/ego_vehicle/odometry", 1, &PurePursuit::odomCB, this);
    path_sub_ = nh_.subscribe("/carla/ego_vehicle/waypoints", 1, &PurePursuit::pathCB, this);
    vehicle_info_sub_ = nh_.subscribe("/carla/ego_vehicle/vehicle_info", 1, &PurePursuit::vehicleInfoCB, this);

    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/path_tracking_node/pure_pursuit/path_marker", 5);
    cmd_pub_ = nh_.advertise<ackermann_msgs::AckermannDrive>("/carla/ego_vehicle/ackermann_cmd", 10);

    //Timer
    timer1_ = nh_.createTimer(ros::Duration((1.0) / controller_freq_), &PurePursuit::controlLoopCB, this); // Duration(1/control_Hz)

    //Init variables
    found_forward_pt_ = false;
    goal_received_ = false;
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
    this->odom_ = *odom_msg;
    if (goal_received_)
    {
        double dist2goal = std::hypot(goal_pos_.x - odom_.pose.pose.position.x,
                                      goal_pos_.y - odom_.pose.pose.position.y);
        if (dist2goal < goal_radius_)
        {
            // goal_reached_ = true;
            goal_received_ = false;
            Vcmd_ = 0.0;
            std::cout << "Goal reached!" << std::endl;
        }
        else
        {
            goal_reached_ = false;
            Vcmd_ = (dist2goal > 10.0) ? 7.0 : 2.0;
        }
    }
}

void PurePursuit::pathCB(const nav_msgs::Path::ConstPtr &path_msg)
{
    map_path_ = *path_msg;
    // 由于之前路径回溯生成，所以终点在前，起点在后
    // 此处进行反转
    // std::reverse(map_path_.poses.begin(), map_path_.poses.end());
    goal_pos_ = map_path_.poses.back().pose.position;
    goal_circle_.pose = map_path_.poses.back().pose;
    // // 更新状态
    goal_received_ = true;
    // 发布追踪终点到rviz
    marker_pub_.publish(goal_circle_);
}

double PurePursuit::getYawFromPose(const geometry_msgs::Pose &car_pose)
{
    float x = car_pose.orientation.x;
    float y = car_pose.orientation.y;
    float z = car_pose.orientation.z;
    float w = car_pose.orientation.w;

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

    /*Visualized Target Point on RVIZ*/
    /*Clear former target point Marker*/
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

double PurePursuit::getSteering(double eta)
{
    return atan2((this->L_ * sin(eta)), (this->Lfw_ / 2 + this->lfw_ * cos(eta)));
}

void PurePursuit::controlLoopCB(const ros::TimerEvent &)
{

    geometry_msgs::Pose carPose = odom_.pose.pose;
    geometry_msgs::Twist carVel = odom_.twist.twist;

    if (goal_received_)
    {
        /*Estimate Steering Angle*/
        double eta = getEta(carPose);
        if (found_forward_pt_)
        {
            steering_ = base_angle_ + getSteering(eta) * steering_gain_;
            /*Estimate Gas Input*/
            // if (goal_reached_)
            // {
            if (smooth_accel_)
                velocity_ = std::min(velocity_ + speed_incremental_, Vcmd_);
            else
                velocity_ = Vcmd_;
            // }
        }
    }

    double dist2goal = std::hypot(goal_pos_.x - odom_.pose.pose.position.x,
                                  goal_pos_.y - odom_.pose.pose.position.y);

    if (dist2goal < goal_radius_)
    {
        velocity_ = 0.0;
        steering_ = base_angle_;
    }

    ackermann_cmd_.speed = velocity_;
    ackermann_cmd_.steering_angle = steering_;
    cmd_pub_.publish(ackermann_cmd_);
}

void PurePursuit::vehicleInfoCB(const carla_msgs::CarlaEgoVehicleInfoConstPtr &vehicle_info_msg) {
  if(use_vehicle_info_){
    auto wheels = vehicle_info_msg->wheels;
    L_ = std::abs(wheels.at(0).position.x - wheels.at(2).position.x);
    lfw_ = std::abs(wheels.at(0).position.x);
    std::cout << "VehicleInfo: wheel_base = " << L_ << " wheel front to center = " << lfw_  << std::endl;
  }
  use_vehicle_info_ = false;
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
