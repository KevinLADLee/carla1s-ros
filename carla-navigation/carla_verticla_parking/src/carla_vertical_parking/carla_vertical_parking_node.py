#!/usr/bin/env python3

import math
from enum import Enum

import rospy
import actionlib
from geometry_msgs.msg import Pose, PoseStamped, Point
from nav_msgs.msg import Path, Odometry
from carla_nav_msgs.msg import Path as PathArray
from carla_nav_msgs.msg import ParkingSpot
from carla_nav_msgs.msg import ParkingPlannerAction, ParkingPlannerActionGoal, \
    ParkingPlannerFeedback, ParkingPlannerActionResult, ParkingPlannerResult, ParkingPlannerGoal
from visualization_msgs.msg import Marker, MarkerArray
from carla_msgs.msg import CarlaEgoVehicleInfo

import os
import sys
import tf
# path1 =
sys.path.append(os.path.split(os.path.abspath(__file__))[0])
# print('path1 ',path1)

# from carla_verticla_parking.src.carla_vertical_parking.find_best_parking_place import GetParkingEndPosition
from car_parking.find_best_parking_place import GetParkingEndPosition

class NodeState(Enum):
    IDLE = 0,
    RUNNING = 1,
    PAUSE = 2,
    SUCCESS = 3,
    FAILURE = 4

class EnvInfoMessage(float):
    road_w = 5 # 2.86*3 #尽量在4.09以上，小于的话腾挪次数要爆炸
    min_turning_radiu = 10
    car_l = 4
    hou_xuan = 1
    # real_parking_left_head = [16.6 ,28.4]
    # real_parking_right_head = [16.6,26.1]
    real_parking_left_head = [3, 0]
    real_parking_right_head = [6, 0]

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
        self.vehicle_info = rospy.wait_for_message("/carla/{}/vehicle_info".format(self.role_name),CarlaEgoVehicleInfo,10)
        self.node_state = NodeState.IDLE

        self.action_goal =  ParkingPlannerActionGoal()
        self.action_feedback = ParkingPlannerFeedback()
        self.action_result = ParkingPlannerResult()

    def compute_best_preparking_position(self, vehicle_pose: Pose, parking_spot: ParkingSpot) -> PoseStamped:


        msg=parking_spot.center_pose
        (r, p, theta) = tf.transformations.euler_from_quaternion(
            [msg.orientation.x,msg.orientation.y, msg.orientation.z, msg.orientation.w])
        center_pose=msg.position

        # 无法从carla中获取的信息
        hou_xuan=EnvInfoMessage.hou_xuan
        road_w=EnvInfoMessage.road_w
        car_l=EnvInfoMessage.car_l


        # 能够从carla中获取的信息
        car_w =abs(self.vehicle_info.wheels[0].position.y)+abs(self.vehicle_info.wheels[1].position.y)
        wheel_dis = abs(self.vehicle_info.wheels[0].position.x)+abs(self.vehicle_info.wheels[2].position.x)
        min_turning_radiu=2.4*car_l
        real_parking_left_head=[center_pose.x+math.cos(theta)*parking_spot.length/2-math.sin(theta)*parking_spot.width/2,
                                center_pose.y+math.sin(theta)*parking_spot.length/2+math.cos(theta)*parking_spot.width/2]
        real_parking_right_head=[center_pose.x+math.cos(theta)*parking_spot.length/2+math.sin(theta)*parking_spot.width/2,
                                center_pose.y+math.sin(theta)*parking_spot.length/2-math.cos(theta)*parking_spot.width/2]



        # 初始化寻找最优停车位方法
        get_park = GetParkingEndPosition(car_l, car_w, min_turning_radiu, wheel_dis, hou_xuan, road_w,
                                         real_parking_left_head, real_parking_right_head)

        # 获取理论最优点
        # 这个理论最优点不考虑车辆当前位置，得出的是车辆一次转弯能转到的理论极限的最优位置
        real_position_x, real_position_y, real_position_theta =get_park.get_best_place()

        best_position=PoseStamped()
        best_position.pose.position.x=real_position_x
        best_position.pose.position.y=real_position_y
        # 停车位的角度
        q = tf.transformations.quaternion_from_euler(0, 0, real_position_theta)

        best_position.pose.orientation.x=q[0]
        best_position.pose.orientation.y=q[1]
        best_position.pose.orientation.z=q[2]
        best_position.pose.orientation.w=q[3]
        # print(real_position_x, real_position_y, real_position_theta*180/math.pi)

        return best_position

    def compute_parking_path(self, vehicle_pose: Pose, parking_spot: ParkingSpot) -> PathArray:
        # self.node_state = NodeState.FAILURE
        self.node_state = NodeState.SUCCESS
        # return PathArray()



    def odom_cb(self, odom_msg):
        self.vehicle_pose = odom_msg.pose.pose

    def execute_cb(self, goal_msg: ParkingPlannerGoal):
        rospy.loginfo("VerticalParking: Received goal, start parking...")

        #测试
        self.compute_best_preparking_position(self.vehicle_pose,goal_msg.parking_spot)

        # vehicle_pose = self.vehicle_pose
        # self.path_array = self.compute_parking_path(vehicle_pose, goal_msg.parking_spot)
        # self.action_result.path_array = self.path_array
        #
        # if self.node_state == NodeState.FAILURE:
        #     self.vertical_parking_server.set_aborted(text="VerticleParking planning failed...")
        # elif self.node_state == NodeState.SUCCESS:
        #     self.vertical_parking_server.set_succeeded(self.action_result, "Vertical parking planning succeed!")

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
    rospy.init_node("carla_vertical_parking", args)

    carla_vertical_parking = None
    try:
        carla_vertical_parking = CarlaVerticalParkingNode()
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
