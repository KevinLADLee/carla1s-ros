#!/usr/bin/env python3

import math

import rospy
import actionlib
from geometry_msgs.msg import Pose, PoseStamped, Point
from nav_msgs.msg import Path, Odometry
from carla_nav_msgs.msg import Path as PathArray
from carla_nav_msgs.msg import ParkingSpot
from carla_nav_msgs.msg import ParkingPlannerAction, ParkingPlannerActionGoal, \
    ParkingPlannerActionFeedback, ParkingPlannerActionResult
from visualization_msgs.msg import Marker, MarkerArray


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

        self.action_goal = ParkingPlannerActionGoal()
        self.action_feedback = ParkingPlannerActionFeedback()
        self.action_result = ParkingPlannerActionResult()

    def compute_best_preparking_position(self, vehicle_pose: Pose, parking_spot: ParkingSpot) -> PoseStamped:
        return PoseStamped()

    def compute_parking_path(self, vehicle_pose: Pose, parking_spot: ParkingSpot) -> PathArray:
        return PathArray()

    def odom_cb(self, odom_msg):
        self.vehicle_pose = odom_msg.pose.pose

    def execute_cb(self, goal_msg: ParkingPlannerActionGoal):
        rospy.loginfo("VerticalParking: Received goal, start parking...")
        vehicle_pose = self.vehicle_pose
        self.path_array = self.compute_parking_path(vehicle_pose, goal_msg.goal.parking_spot)

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
