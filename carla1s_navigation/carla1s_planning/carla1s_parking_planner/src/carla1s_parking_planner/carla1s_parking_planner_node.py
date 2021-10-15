#!/usr/bin/env python3

import math
from enum import Enum

import tf
import rospy
import actionlib
from geometry_msgs.msg import Pose, PoseStamped, Point
from nav_msgs.msg import Path, Odometry
from carla1s_msgs.msg import PathArray
from carla1s_msgs.msg import ParkingSpot
from carla1s_msgs.msg import ParkingPlannerAction, ParkingPlannerActionGoal, \
    ParkingPlannerFeedback, ParkingPlannerActionResult, ParkingPlannerResult, ParkingPlannerGoal
from visualization_msgs.msg import Marker, MarkerArray
from carla_msgs.msg import CarlaEgoVehicleInfo
from carla1s_parking_planner.get_best_parking_position import GetBestParkingPosition



from carla1s_parking_planner.arc_and_line_planning import  ArcLinePlanning
from carla1s_parking_planner.car_parking.env import Env
from carla1s_parking_planner.car_parking.parking_planning import Planning

class NodeState(Enum):
    IDLE = 0,
    RUNNING = 1,
    PAUSE = 2,
    SUCCESS = 3,
    FAILURE = 4

#这里面的信息都是无法从carla的vertical info中获取的参数
class EnvInfoMessage(float):
    road_w = 5 # 2.86*3 #尽量在4.09以上，小于的话腾挪次数要爆炸
    car_l = 4
    hou_xuan = 1
    step=0.1
    road_l=10

class CarlaVerticalParkingNode:
    def __init__(self):
        self.role_name = rospy.get_param("role_name", 'ego_vehicle')

        self.odom_sub = rospy.Subscriber('/carla/{}/odometry'.format(self.role_name), Odometry, self.odom_cb)
        self.path_markers_pub = rospy.Publisher('/carla/{}/path_markers'.format(self.role_name), MarkerArray,
                                                latch=True, queue_size=1)
        self.markers = MarkerArray()

        # set initial goal
        self.goal = None
        self.vehicle_pose = None
        self.current_route = None
        self.vertical_parking_server = actionlib.SimpleActionServer("vertical_parking", ParkingPlannerAction,
                                                                    execute_cb=self.execute_cb,
                                                                    auto_start=False)
        self.vertical_parking_server.start()
        self.path_array = PathArray()

        rospy.loginfo("VerticlaParking: Waiting for vehicle info message...")
        self.vehicle_info = rospy.wait_for_message("/carla/{}/vehicle_info".format(self.role_name), CarlaEgoVehicleInfo, 10)
        self.node_state = NodeState.IDLE

        self.action_goal =  ParkingPlannerActionGoal()
        self.action_feedback = ParkingPlannerFeedback()
        self.action_result = ParkingPlannerResult()

    def compute_best_preparking_position(self, vehicle_pose: Pose, parking_spot: ParkingSpot) -> PoseStamped:

        return GetBestParkingPosition(self.vehicle_info).get_best_parking_position(vehicle_pose, parking_spot)


    def compute_parking_path(self, vehicle_pose: Pose, parking_spot: ParkingSpot) -> PathArray:

        path_array = PathArray()
        try:
            path_array=ArcLinePlanning(self.vehicle_info).planning(vehicle_pose, parking_spot)

            if path_array!=PathArray():
                self.node_state = NodeState.SUCCESS
            else:
                self.node_state = NodeState.FAILURE
        except:
            print("planning fail")
            self.node_state = NodeState.FAILURE

        return path_array


    def odom_cb(self, odom_msg):
        self.vehicle_pose = odom_msg.pose.pose

    def execute_cb(self, goal_msg):
        rospy.loginfo("VerticalParking: Received goal, start parking...")
        # print(type(goal_msg))

        vehicle_pose = self.vehicle_pose
        self.compute_best_preparking_position(vehicle_pose, goal_msg.parking_spot)
        #计算泊车路径
        self.path_array = self.compute_parking_path(vehicle_pose, goal_msg.parking_spot)
        self.action_result.path_array = self.path_array

        if self.node_state == NodeState.FAILURE:
            self.vertical_parking_server.set_aborted(text="VerticleParking planning failed...")
        elif self.node_state == NodeState.SUCCESS:
            self.vertical_parking_server.set_succeeded(self.action_result, "Vertical parking planning succeed!")

    def publish_path_array_markers(self, path_array: PathArray):
        self.markers = MarkerArray()
        for i in range(len(path_array.paths)):
            path_marker = Marker()
            path_marker.header.frame_id = path_array.header.frame_id
            path_marker.header.stamp = path_array.header.stamp
            path_marker.ns = "vertical_parking"
            path_marker.id = 100 + i
            path_marker.type = Marker.LINE_STRIP
            path_marker.action = Marker.ADD
            path_marker.scale.x = 0.2
            path_marker.pose.orientation.w = 1.0
            for pose_stamp in path_array.paths[i].poses:
                point = Point()
                point.x = pose_stamp.pose.position.x
                point.y = pose_stamp.pose.position.y
                point.z = pose_stamp.pose.position.z
                path_marker.points.append(point)
            if path_array.driving_direction[i] == PathArray.BACKWARDS:
                path_marker.color.a = 0.8
                path_marker.color.g = 0
                path_marker.color.b = 0
                path_marker.color.r = 0.8
            else:
                path_marker.color.a = 0.8
                path_marker.color.g = 0.8
                path_marker.color.b = 0
                path_marker.color.r = 0
            self.markers.markers.append(path_marker)
        self.path_markers_pub.publish(self.markers)


def main(args=None):
    """
    main function
    """
    rospy.init_node("carla1s_parking_planner", args)

    carla1s_parking_planner = None
    try:
        carla1s_parking_planner = CarlaVerticalParkingNode()
        rospy.spin()
    except (RuntimeError, rospy.ROSException):
        rospy.logerr("CarlaVerticalParking: Error occurs!")
        pass
    except KeyboardInterrupt:
        rospy.loginfo("CarlaVerticalParking: User requested shut down.")
    finally:
        rospy.loginfo("CarlaVerticalParking: Shut down now...")


if __name__ == "__main__":
    main()
