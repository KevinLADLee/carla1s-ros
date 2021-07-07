#!/usr/bin/env python3

import math
import rospy
import actionlib

from geometry_msgs.msg import PoseStamped, Point
import carla_common.transforms as trans
from carla_msgs.msg import CarlaWorldInfo
from nav_msgs.msg import Path
from carla_nav_msgs.msg import Path as PathArray
from carla_nav_msgs.msg import PathPlannerAction, PathPlannerResult, PathPlannerFeedback
from visualization_msgs.msg import Marker, MarkerArray

import carla
from agents.navigation.global_route_planner import GlobalRoutePlanner
from agents.navigation.global_route_planner_dao import GlobalRoutePlannerDAO

from rospy import ROSException

from carla_global_planner.reeds_shepp_ros.reeds_ros import reeds_ros_inter as ReedsSheppROS

class CarlaGlobalPlanner:
    WAYPOINT_DISTANCE = 2.0

    def __init__(self):
        self.connect_to_carla()
        self.map = self.world.get_map()
        self.ego_vehicle = None
        self.ego_vehicle_location = None
        self.on_tick = None
        self.role_name = rospy.get_param("role_name", 'ego_vehicle')
        self.waypoint_publisher = rospy.Publisher('/carla/{}/waypoints'.format(self.role_name), Path, latch=True,
                                                  queue_size=1)
        
        self.path_markers_publisher = rospy.Publisher('/carla/{}/path_markers'.format(self.role_name), MarkerArray,
                                                      latch=True, queue_size=1)
        self.markers = MarkerArray()                                          

        # set initial goal
        self.goal = None

        self.current_route = None
        self.global_planner_server = actionlib.SimpleActionServer("compute_path_to_goal",
                                                                  PathPlannerAction,
                                                                  execute_cb=self.execute_cb,
                                                                  auto_start=False)
        self.rs_curve = ReedsSheppROS(min_radius=3)
        self.pose_list = []

        self.global_planner_server.start()
        self._feedback = PathPlannerFeedback()
        self._result = PathPlannerResult()

        # use callback to wait for ego vehicle

        rospy.loginfo("GlobalPlanner: Waiting for ego vehicle...")
        self.on_tick = self.world.on_tick(self.find_ego_vehicle_actor)

    def execute_cb(self, goal_msg):

        rospy.loginfo("GlobalPlanner: Received goal, trigger rerouting...")
        rospy.loginfo("GlobalPlanner: Planner id: {}".format(goal_msg.planner_id))

        carla_goal = trans.ros_pose_to_carla_transform(goal_msg.goal.pose)
        self.goal = carla_goal

        if goal_msg.planner_id == "reeds_shepp":
            self.pose_list.clear()
            if self.ego_vehicle is None or self.goal is None:
                rospy.logerr("GlobalPlanner: ego_vehicle not valid now!")
                self.global_planner_server.set_aborted(text="Error: ego_vehicle or goal not valid!")
            elif self.is_goal_reached(self.goal):
                self.global_planner_server.set_aborted(text="Already reached goal!")
            else:       
                vehicle_pose = trans.carla_transform_to_ros_pose(self.ego_vehicle.get_transform())     
                self.pose_list.append(vehicle_pose)
                self.pose_list.append(goal_msg.goal.pose)
                self._result.path = self.rs_curve.shortest_path(self.pose_list)
                self.publish_path_array_markers(self._result.path)
                self.global_planner_server.set_succeeded(self._result, "success")
            # TODO: Action server send result.
        else:
            carla_goal = trans.ros_pose_to_carla_transform(goal_msg.goal.pose)
            self.goal = carla_goal
            self._result.path = PathArray()
            self._result.path.header.frame_id = "map"
            self._result.path.header.stamp = rospy.Time.now()
            path = Path()
            self._result.path.paths.append(path)
            self._result.path.paths[0].header.frame_id = "map"
            self._result.path.paths[0].header.stamp = rospy.Time.now()
            self._result.path.driving_direction.append(0)
            if self.ego_vehicle is None or self.goal is None:
                self.global_planner_server.set_aborted(text="Error: ego_vehicle or goal not valid!")
            elif self.is_goal_reached(self.goal):
                self.global_planner_server.set_aborted(text="Already reached goal!")
            else:
                self.current_route = self.calculate_route(self.goal)
                if self.current_route is not None:
                    for wp in self.current_route:
                        pose = PoseStamped()
                        pose.pose = trans.carla_transform_to_ros_pose(wp[0].transform)
                        self._result.path.paths[0].poses.append(pose)

                waypoint_num = len(self._result.path.paths[0].poses)
                result_info = "GlobalPlanner: Got path {} waypoints.".format(waypoint_num)
                rospy.loginfo(result_info)
                if waypoint_num <= 1:
                    self.global_planner_server.set_aborted(self._result, result_info)
                else:
                    self.waypoint_publisher.publish(self._result.path.paths[0])
                    self.global_planner_server.set_succeeded(self._result, result_info)
        
    def publish_path_array_markers(self, path_array):
        self.markers = MarkerArray()
        for i in range (len(path_array.paths)):
            path_marker = Marker()
            path_marker.header.frame_id = path_array.header.frame_id
            path_marker.header.stamp = path_array.header.stamp
            path_marker.ns = "path"
            path_marker.id = 100+i
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
        self.path_markers_publisher.publish(self.markers)        


    def is_goal_reached(self, goal):
        vehicle_location = self.ego_vehicle.get_location()
        dist = math.hypot(goal.location.x - vehicle_location.x, goal.location.y - vehicle_location.y)
        if dist < 2:
            return True
        else:
            return False

    def destroy(self):
        """
        Destructor
        """
        self.ego_vehicle = None
        if self.on_tick:
            self.world.remove_on_tick(self.on_tick)

    def reroute(self):
        """
        Triggers a rerouting
        """
        if self.ego_vehicle is None or self.goal is None:
            # no ego vehicle, remove route if published
            self.current_route = None
            self.publish_waypoints()
        else:
            self.current_route = self.calculate_route(self.goal)
        self.publish_waypoints()

    def find_ego_vehicle_actor(self, _):
        """
        Look for an carla actor with name 'ego_vehicle'
        """
        hero = None
        for actor in self.world.get_actors():
            if actor.attributes.get('role_name') == self.role_name:
                hero = actor
                break

        ego_vehicle_changed = False
        if hero is None and self.ego_vehicle is not None:
            ego_vehicle_changed = True

        if not ego_vehicle_changed and hero is not None and self.ego_vehicle is None:
            ego_vehicle_changed = True

        if not ego_vehicle_changed and hero is not None and \
                self.ego_vehicle is not None and hero.id != self.ego_vehicle.id:
            ego_vehicle_changed = True

        if ego_vehicle_changed:
            rospy.loginfo("Ego vehicle changed.")
            self.ego_vehicle = hero
            self.reroute()
        elif self.ego_vehicle:
            current_location = self.ego_vehicle.get_location()
            if self.ego_vehicle_location:
                dx = self.ego_vehicle_location.x - current_location.x
                dy = self.ego_vehicle_location.y - current_location.y
                distance = math.sqrt(dx * dx + dy * dy)
                if distance > self.WAYPOINT_DISTANCE:
                    rospy.loginfo("Ego vehicle was repositioned.")
                    self.reroute()
            self.ego_vehicle_location = current_location

    def calculate_route(self, goal):
        """
        Calculate a route from the current location to 'goal'
        """
        # rospy.loginfo("Calculating route to x={}, y={}, z={}".format(
        #     goal.location.x,
        #     goal.location.y,
        #     goal.location.z))

        dao = GlobalRoutePlannerDAO(self.world.get_map(), sampling_resolution=1)
        grp = GlobalRoutePlanner(dao)
        grp.setup()
        route = grp.trace_route(self.ego_vehicle.get_location(),
                                carla.Location(goal.location.x,
                                               goal.location.y,
                                               goal.location.z))

        return route

    def publish_waypoints(self):
        """
        Publish the ROS message containing the waypoints
        """
        msg = Path()
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()
        if self.current_route is not None:
            for wp in self.current_route:
                pose = PoseStamped()
                pose.pose = trans.carla_transform_to_ros_pose(wp[0].transform)
                msg.poses.append(pose)

        self.waypoint_publisher.publish(msg)
        rospy.loginfo("Published {} waypoints.".format(len(msg.poses)))

    def connect_to_carla(self):

        rospy.loginfo("Waiting for CARLA world (topic: /carla/world_info)...")
        try:
            rospy.wait_for_message("/carla/world_info", CarlaWorldInfo, timeout=rospy.Duration(secs=15))
        except ROSException as e:
            rospy.logerr("Error while waiting for world info: {}".format(e))
            raise e

        host = rospy.get_param("host", "127.0.0.1")
        port = rospy.get_param("port", 2000)
        timeout = rospy.get_param("timeout", 10)
        rospy.loginfo("CARLA world available. Trying to connect to {host}:{port}".format(
            host=host, port=port))

        carla_client = carla.Client(host=host, port=port)
        carla_client.set_timeout(timeout)

        try:
            self.world = carla_client.get_world()
        except RuntimeError as e:
            rospy.logerr("Error while connecting to Carla: {}".format(e))
            raise e

        rospy.loginfo("Connected to Carla.")


def main(args=None):
    """
    main function
    """
    rospy.init_node("carla_global_planner", args)

    carla_global_planner = None
    try:
        carla_global_planner = CarlaGlobalPlanner()
        rospy.spin()
    except (RuntimeError, ROSException):
        pass
    except KeyboardInterrupt:
        print("User requested shut down.")
    finally:
        print("Shutting down.")

if __name__ == "__main__":
    main()
