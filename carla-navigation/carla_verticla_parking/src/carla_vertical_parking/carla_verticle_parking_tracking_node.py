
import math
from enum import Enum
import numpy as np

import rospy
import actionlib
from geometry_msgs.msg import Pose, PoseStamped, Point
from nav_msgs.msg import Path, Odometry
from carla_nav_msgs.msg import PathTrackingAction, PathTrackingActionGoal, PathTrackingActionResult,PathTrackingResult,PathTrackingGoal

from carla_msgs.msg import CarlaEgoVehicleControl

class NodeState(Enum):
    IDLE = 0,
    RUNNING = 1,
    PAUSE = 2,
    SUCCESS = 3,
    FAILURE = 4

class VerticalParkingTrackingNode:
    def __init__(self):
        self.role_name = rospy.get_param("role_name", 'ego_vehicle')

        self.vehicle_pose = Pose()
        self.vehicle_speed = 0

        self.odom_sub = rospy.Subscriber('/carla/{}/odometry'.format(self.role_name),
                                         Odometry,
                                         self.odom_cb)
        self.control_cmd_pub = rospy.Publisher('/carla/{}/vehicle_control_cmd'.format(self.role_name),
                                               CarlaEgoVehicleControl, latch=False, queue_size=5)


        self.parking_tracking_server = actionlib.SimpleActionServer("vertical_parking_tracking",
                                                                    PathTrackingAction,
                                                                    execute_cb=self.execute_cb,
                                                                    auto_start=False)

        #self.steer_controller = 
        #self.velocity_controller = 
        self.path_array = None

        self.action_result = PathTrackingResult()
        print("init success")


    def odom_cb(self, odom_msg : Odometry):
        self.vehicle_pose = odom_msg.pose.pose
        self.vehicle_speed =  np.linalg.norm(np.array([odom_msg.twist.twist.linear.x, odom_msg.twist.twist.linear.y, odom_msg.twist.twist.linear.z], dtype=float))

    def compute_and_publish_vehicle_cmd(self, path):
        control_cmd = CarlaEgoVehicleControl()





        control_cmd.steer = self.steer_controller.run_step(self.vehicle_pose, path)
        control_cmd.throttle, control_cmd.reverse = self.velocity_controller.run_step(self.vehicle_speed)
        self.control_cmd_pub.publish(control_cmd)

    def stop_vehicle(self):
        control_cmd = CarlaEgoVehicleControl()
        control_cmd.steer = 0.0
        control_cmd.throttle = 0.0
        control_cmd.brake = 1.0
        self.control_cmd_pub.publish(control_cmd)

    def is_goal_reached(self) -> bool:

        return False


    def is_tracking_failed(self) -> bool:
        return False    
    
    def execute_cb(self, action_goal):
        rospy.loginfo("VerticalParkingTracking: Path received, start tracking...")
        r = rospy.Rate(20)
        self.path_array = action_goal.goal.path

        while not rospy.is_shutdown:
            if self.parking_tracking_server.is_preempt_requested():
                self.parking_tracking_server.set_aborted()
                self.stop_vehicle()
                break

            if self.is_tracking_failed():
                self.action_result.result.error_code = NodeState.FAILURE
                self.parking_tracking_server.set_aborted(result=self.action_result)

            # self.compute_and_publish_vehicle_cmd(action_goal.goal.path)
            if self.is_goal_reached():
                break      
            r.sleep()
            break

        self.stop_vehicle()    
        self.action_result.result.error_code = NodeState.SUCCESS
        self.parking_tracking_server.set_succeeded(result=self.action_result)


def main(args=None):
    """
    main function
    """
    rospy.init_node("carla_vertical_tracking0", args)

    carla_vertical_parking = None
    try:
        carla_vertical_parking = VerticalParkingTrackingNode()
        rospy.spin()
    except (RuntimeError, rospy.ROSException):
        rospy.logerr("CarlaVerticalParkingTracking: Error occurs!")
        pass
    except KeyboardInterrupt:
        rospy.loginfo("CarlaVerticalParkingTracking: User requested shut down.")
    finally:
        rospy.loginfo("CarlaVerticalParkingTracking: Shut down now...")


if __name__ == "__main__":
    main()

