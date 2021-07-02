#!/usr/bin/env python

import math
import rospy
import actionlib

from geometry_msgs.msg import PoseStamped, Point
import carla_common.transforms as trans
from carla_msgs.msg import CarlaWorldInfo
from nav_msgs.msg import Path
from carla_nav_msgs.msg import Path as PathArray
from carla_nav_msgs.msg import PathPlannerAction, PathPlannerResult, PathPlannerFeedback
from carla_nav_msgs.msg import GlobalPlannerAction, GlobalPlannerResult, GlobalPlannerFeedback
from visualization_msgs.msg import Marker, MarkerArray

import carla
from agents.navigation.global_route_planner import GlobalRoutePlanner
from agents.navigation.global_route_planner_dao import GlobalRoutePlannerDAO

from rospy import ROSException

import numpy as np
from collections import namedtuple
from scipy.spatial.transform import Rotation as rot
from math import sqrt, atan2, sin, cos, pi, inf, asin, acos

"""
Author: Han Ruihua
Reference: paper 'Optimal paths for a car that goes both forwards and backwards', github: https://github.com/nathanlct/reeds-shepp-curves/tree/master
"""

class ReedsShepp:

    def __init__(self, min_radius=1):

        self.min_r = min_radius
        self.element = namedtuple('element','len steer gear') # steer 1,0,-1, gear 1,-1
        self.path_formula1 = [self.LpSpLp, self.LpSpRp, self.LpRnLp, self.LpRpLnnRn, self.LpRnLnRp, self.LpRnRnLnRp]
        self.path_formula2 = [self.LpRnLn, self.LpRnSnLn, self.LpRnSnRn]

    # preprocess
    def preprocess(self, start_point=np.zeros((3, 1)), goal_point=np.zeros((3, 1))):
        
        ro_phi = start_point[2, 0]
        ro_matrix = np.array([[cos(ro_phi), sin(ro_phi)], 
                             [-sin(ro_phi), cos(ro_phi)]])

        diff = goal_point[0:2] - start_point[0:2]
        new_pos = ro_matrix @ diff

        new_x = new_pos[0, 0] / self.min_r
        new_y = new_pos[1, 0] / self.min_r
        new_phi = goal_point[2, 0] - start_point[2, 0]

        return new_x, new_y, new_phi

    # shortest path
    def shortest_path(self, start_point=np.zeros((3, 1)), goal_point=np.ones((3, 1)), step_size=0.01):
        
        x, y, phi = self.preprocess(start_point, goal_point)

        path_list1, List1 = self.symmetry_curve1(x, y, phi)
        path_list2, List2 = self.symmetry_curve2(x, y, phi)

        total_path_list = path_list1 + path_list2
        total_L_list = List1 + List2

        L_min = min(total_L_list) 
        path_min = total_path_list[total_L_list.index(L_min)]

        path_point_list = self.reeds_path_generate(start_point, path_min, step_size)

        return path_point_list

    # calculate curves
    def symmetry_curve1(self, x, y, phi):
        
        path_list = []
        L_list = []

        for timeflip in [False, True]:
            for reflect in [False, True]:
                for cal_formula in self.path_formula1:

                    rg_flag = -1 if timeflip else 1
                    rs_flag = -1 if reflect else 1
                    
                    path, L = cal_formula(rg_flag*x, rs_flag*y, rs_flag*rg_flag*phi, timeflip=timeflip, reflect=reflect)

                    path_list.append(path.copy())
                    L_list.append(L)
        
        return path_list, L_list

    def symmetry_curve2(self, x, y, phi):
        
        path_list = []
        L_list = []

        for timeflip in [False, True]:
            for reflect in [False, True]:
                for backward in [False, True]:
                    for cal_formula in self.path_formula2:

                        rg_flag = -1 if timeflip else 1
                        rs_flag = -1 if reflect else 1
                        sx, sy, sphi = self.backward(x,y,phi) if backward else (x,y,phi)
                        
                        path, L = cal_formula(rg_flag*sx, rs_flag*sy, rs_flag*rg_flag*sphi, timeflip=timeflip, reflect=reflect, backward=backward)

                        path_list.append(path.copy())
                        L_list.append(L)
        
        return path_list, L_list

    # path generate
    def reeds_path_generate(self, start_point, path, step_size):

        path_point_list = []
        start_point = start_point
        end_point = None

        if len(path) == 0:
            print('no path')
            return path_point_list

        for i in range(len(path)):
            
            path_list, end_point = self.element_sample(element=path[i], start_point=start_point, step_size=step_size)

            path_point_list = path_point_list + path_list
            start_point = end_point

        return path_point_list

    def element_sample(self, element, start_point, step_size):

        cur_x = start_point[0, 0]
        cur_y = start_point[1, 0]
        cur_theta = start_point[2, 0]

        steer = element.steer
        gear = element.gear
        length = element.len * self.min_r

        # calculate end point
        endpoint = np.zeros((3, 1))
        path_list = [start_point]

        curvature = steer * 1/self.min_r
        center_x = cur_x + cos(cur_theta + steer * pi/2) * self.min_r
        center_y = cur_y + sin(cur_theta + steer * pi/2) * self.min_r
        rot_len = abs(steer) * length 
        trans_len = (1 - abs(steer)) * length

        rot_theta = rot_len * curvature * gear
        rot_matrix = np.array([[cos(rot_theta), -sin(rot_theta)], [sin(rot_theta), cos(rot_theta)]])
        trans_matrix = trans_len * np.array([[cos(cur_theta)], [sin(cur_theta)]])
        center = np.array([[center_x], [center_y]])

        endpoint[0:2] = rot_matrix @ (start_point[0:2] - center) + center + gear * trans_matrix
        endpoint[2, 0] = self.M(cur_theta + rot_theta)

        cur_length = 0

        if length < 0:
            length = 2 * pi * self.min_r + length

        while cur_length < length:

            next_x = cur_x + gear * cos(cur_theta) * step_size
            next_y = cur_y + gear * sin(cur_theta) * step_size
            next_theta = cur_theta + gear * curvature * step_size

            d_length = np.sqrt( (next_x - cur_x) **2 + (next_y - cur_y) ** 2 )
            cur_length = cur_length + d_length

            next_point = np.array([[next_x], [next_y], [next_theta]])

            path_list.append(next_point)

            cur_x = next_x
            cur_y = next_y
            cur_theta = next_theta

        path_list.append(endpoint)

        return path_list, endpoint
    
    # transform
    # mode 2pi
    def M(self, theta):
        theta = theta % (2*pi)
        if theta < - pi: return theta + 2*pi
        if theta >= pi: return theta - 2*pi
        return theta

    # polar 
    def R(self, x, y):   
       r = sqrt(x**2 + y**2)
       theta = atan2(y, x)
       return r, theta

    def backward(self, x, y, phi):
        new_x = x*cos(phi) + y*sin(phi)
        new_y = x*sin(phi) - y*cos(phi)
        return new_x, new_y, phi

    # curve formula
    # formula 8.1
    def LpSpLp(self, x, y, phi, timeflip=False, reflect=False):
        
        path = []
        gear_flag = -1 if timeflip else 1
        steer_flag = -1 if reflect else 1

        #calculate
        u, t = self.R(x - sin(phi), y-1+cos(phi))
        v = (phi - t) % (2*pi)

        path.append(self.element(t, steer_flag, gear_flag)) 
        path.append(self.element(u, 0, gear_flag)) 
        path.append(self.element(v, steer_flag, gear_flag))

        if t < 0 or t > pi or v < 0 or v> pi:
            return path, inf
        
        L = abs(t) + abs(u) + abs(v)

        return path, L

    # formula 8.2
    def LpSpRp(self, x, y, phi, timeflip=False, reflect=False):

        path = []
        gear_flag = -1 if timeflip else 1
        steer_flag = -1 if reflect else 1

        u1, t1 = self.R(x+sin(phi), y-1-cos(phi))

        if u1**2 < 4:
            L = inf
        else:
            u = sqrt(u1**2 - 4)

            T, theta = self.R(u, 2)
            t = self.M(t1+theta)
            v = self.M(t-phi)
            L = abs(t) + abs(u) + abs(v)

            path.append(self.element(t, steer_flag*1, gear_flag)) 
            path.append(self.element(u, 0, gear_flag)) 
            path.append(self.element(v, steer_flag*-1, gear_flag))   

        return path, L

    # formula 8.3  typo in paper
    def LpRnLp(self, x, y, phi, timeflip=False, reflect=False):

        path = []
        gear_flag = -1 if timeflip else 1
        steer_flag = -1 if reflect else 1

        xi = x - sin(phi)
        eta = y - 1 + cos(phi)
        u1, theta = self.R(xi, eta)

        if u1 ** 2 > 4:
            return path, inf
        
        A = acos(u1/4)
        t = self.M(theta + pi/2 + A)
        u = self.M(pi - 2*A)
        v = self.M(phi-t-u)
        
        L = abs(t) + abs(u) + abs(v)

        path.append(self.element(t, steer_flag*1, gear_flag*1)) 
        path.append(self.element(u, steer_flag*-1, gear_flag*-1)) 
        path.append(self.element(v, steer_flag*1, gear_flag*1)) 

        return path, L

    # formula 8.4, typo in paper, 
    def LpRnLn(self, x, y, phi, timeflip=False, reflect=False, backward=False):

        path = []
        gear_flag = -1 if timeflip else 1
        steer_flag = -1 if reflect else 1

        xi = x - sin(phi)
        eta = y - 1 + cos(phi)
        u1, theta = self.R(xi, eta)

        if u1 ** 2 > 4:
            return path, inf
        
        A = acos(u1/4)
        t = self.M(theta + pi/2 + A)
        u = self.M(pi - 2*A)
        v = self.M(t+u-phi)
        
        L = abs(t) + abs(u) + abs(v)

        if backward:
            path.append(self.element(v, steer_flag*1, gear_flag*-1))
            path.append(self.element(u, steer_flag*-1, gear_flag*-1))
            path.append(self.element(t, steer_flag*1, gear_flag*1))
        else:
            path.append(self.element(t, steer_flag*1, gear_flag*1)) 
            path.append(self.element(u, steer_flag*-1, gear_flag*-1)) 
            path.append(self.element(v, steer_flag*1, gear_flag*-1)) 

        return path, L
    
    # formula 8.7 typo in paper
    def LpRpLnnRn(self, x, y, phi, timeflip=False, reflect=False):

        path = []
        gear_flag = -1 if timeflip else 1
        steer_flag = -1 if reflect else 1

        xi = x + sin(phi)
        eta = y - 1 - cos(phi)
        u1, theta = self.R(xi, eta)

        if u1 > 4:
            return path, inf 
        
        if u1 <= 2:
            A = acos((u1+2)/4)
            t = self.M(theta+pi/2+A)
            u = self.M(A)
            v = self.M(phi-t+2*u)
        else:
            A = acos((u1-2)/4)
            t = self.M(theta+pi/2-A)
            u = self.M(pi-A)
            v = self.M(phi-t+2*u)
        
        L = abs(t) + 2*abs(u) + abs(v)

        path.append(self.element(t, steer_flag*1, gear_flag*1)) 
        path.append(self.element(u, steer_flag*-1, gear_flag*1)) 
        path.append(self.element(u, steer_flag*1, gear_flag*-1))
        path.append(self.element(v, steer_flag*-1, gear_flag*-1))  

        return path, L

    # formula 8.8 
    def LpRnLnRp(self, x, y, phi, timeflip=False, reflect=False):
        
        path = []
        gear_flag = -1 if timeflip else 1
        steer_flag = -1 if reflect else 1

        xi = x + sin(phi)
        eta = y - 1 - cos(phi)
        u1, theta = self.R(xi, eta)
        rho = (20 - u1**2) / 16

        if rho >= 0 and rho <= 1:
            u = acos(rho)
            A = asin(2*sin(u)/u1)
            t = self.M(theta+pi/2+A)
            v = self.M(t-phi)

            L = abs(t) + 2*abs(u) + abs(v)

            path.append(self.element(t, steer_flag*1, gear_flag*1)) 
            path.append(self.element(u, steer_flag*-1, gear_flag*-1)) 
            path.append(self.element(u, steer_flag*1, gear_flag*-1))
            path.append(self.element(v, steer_flag*-1, gear_flag*1))  

        else:
            return path, inf

        return path, L
    
    # formula 8.9
    def LpRnSnLn(self, x, y, phi, timeflip=False, reflect=False, backward=False):
        
        path = []
        gear_flag = -1 if timeflip else 1
        steer_flag = -1 if reflect else 1

        xi = x - sin(phi)
        eta = y - 1 + cos(phi)
        rho, theta = self.R(xi, eta)
        
        if rho < 2:
            return path, inf
        
        u = sqrt(rho**2-4) -2
        A = atan2(2, u+2)
        t = self.M(theta+pi/2+A)
        v = self.M(t-phi+pi/2)

        L = abs(t) + pi/2 + abs(u) + abs(v)

        if backward:
            path.append(self.element(v, steer_flag*1, gear_flag*-1)) 
            path.append(self.element(u, steer_flag*0, gear_flag*-1))
            path.append(self.element(pi/2, steer_flag*-1, gear_flag*-1)) 
            path.append(self.element(t, steer_flag*1, gear_flag*1))               
        else:
            path.append(self.element(t, steer_flag*1, gear_flag*1)) 
            path.append(self.element(pi/2, steer_flag*-1, gear_flag*-1)) 
            path.append(self.element(u, steer_flag*0, gear_flag*-1))
            path.append(self.element(v, steer_flag*1, gear_flag*-1)) 

        return path, L

    # formula 8.10
    def LpRnSnRn(self, x, y, phi, timeflip=False, reflect=False, backward=False):

        path = []
        gear_flag = -1 if timeflip else 1
        steer_flag = -1 if reflect else 1

        xi = x + sin(phi)
        eta = y - 1 - cos(phi)
        rho, theta = self.R(xi, eta)

        if rho < 2:
            return path, inf

        t = self.M(theta+pi/2)
        u = rho-2
        v = self.M(phi - t -pi/2)

        L = abs(t) + pi/2 + abs(u) + abs(v)
        
        if backward:
            path.append(self.element(v, steer_flag*-1, gear_flag*-1))
            path.append(self.element(u, steer_flag*0, gear_flag*-1)) 
            path.append(self.element(pi/2, steer_flag*-1, gear_flag*-1)) 
            path.append(self.element(t, steer_flag*1, gear_flag*1))
        else:
            path.append(self.element(t, steer_flag*1, gear_flag*1)) 
            path.append(self.element(pi/2, steer_flag*-1, gear_flag*-1)) 
            path.append(self.element(u, steer_flag*0, gear_flag*-1))
            path.append(self.element(v, steer_flag*-1, gear_flag*-1))

        return path, L

    # formula 8.11 typo in paper
    def LpRnRnLnRp(self, x, y, phi, timeflip=False, reflect=False):

        path = []
        gear_flag = -1 if timeflip else 1
        steer_flag = -1 if reflect else 1

        xi = x + sin(phi)
        eta = y - 1 - cos(phi)
        rho, theta = self.R(xi, eta)

        if rho < 4:
            return path, inf

        u = sqrt(rho**2 - 4) - 4
        A = atan2(2, u+4)
        t = self.M(theta+pi/2+A)
        v = self.M(t-phi)

        L = abs(t) + pi/2 + abs(u) + pi/2 + abs(v)

        path.append(self.element(t, steer_flag*1, gear_flag*1)) 
        path.append(self.element(pi/2, steer_flag*-1, gear_flag*-1)) 
        path.append(self.element(u, steer_flag*0, gear_flag*-1))
        path.append(self.element(pi/2, steer_flag*1, gear_flag*-1))
        path.append(self.element(v, steer_flag*-1, gear_flag*1))

        return path, L


class ReedsSheppROS(ReedsShepp):

    def __init__(self, min_radius):
        super().__init__(min_radius)

    def shortest_path(self, pose_list, step_size=0.01):

        path_array = PathArray()
        path_array.header.frame_id = "map"
        path_array.header.stamp = rospy.Time.now()

        for i in range(len(pose_list) - 1):

            start_point = self.pose2point(pose_list[i])
            goal_point = self.pose2point(pose_list[i+1])

            x, y, phi = self.preprocess(start_point, goal_point)

            path_list1, List1 = self.symmetry_curve1(x, y, phi)
            path_list2, List2 = self.symmetry_curve2(x, y, phi)

            total_path_list = path_list1 + path_list2
            total_L_list = List1 + List2

            L_min = min(total_L_list) 
            path_min = total_path_list[total_L_list.index(L_min)]

            path_array_each_pose = self.reeds_path_generate(start_point, path_min, step_size)

            path_array.paths = path_array.paths + path_array_each_pose.paths
            path_array.driving_direction = path_array.driving_direction + path_array_each_pose.driving_direction

        return path_array

    def reeds_path_generate(self, start_point, path, step_size):

        path_array = PathArray()
        
        end_point = None

        if len(path) == 0:
            print('no path')
            return path_array

        for i in range(len(path)):
            
            sample_path, driving_direction, end_point = self.element_sample(element=path[i], start_point=start_point, step_size=step_size)

            start_point = end_point
            path_array.paths.append(sample_path)
            path_array.driving_direction.append(driving_direction)

        return path_array

    def element_sample(self, element, start_point, step_size):
        
        sample_path = Path()
        sample_path.header.frame_id = "map"
        sample_path.header.stamp = rospy.Time.now()
        driving_direction = 0 if element.gear == 1 else 1

        cur_x = start_point[0, 0]
        cur_y = start_point[1, 0]
        cur_theta = start_point[2, 0]

        sample_path.poses.append(self.point2pose(start_point))

        steer = element.steer
        gear = element.gear
        length = element.len * self.min_r

        # calculate end point
        endpoint = np.zeros((3, 1))

        curvature = steer * 1 / self.min_r
        center_x = cur_x + cos(cur_theta + steer * pi/2) * self.min_r
        center_y = cur_y + sin(cur_theta + steer * pi/2) * self.min_r
        rot_len = abs(steer) * length 
        trans_len = (1 - abs(steer)) * length

        rot_theta = rot_len * curvature * gear
        rot_matrix = np.array([[cos(rot_theta), -sin(rot_theta)], [sin(rot_theta), cos(rot_theta)]])
        trans_matrix = trans_len * np.array([[cos(cur_theta)], [sin(cur_theta)]])
        center = np.array([[center_x], [center_y]])

        endpoint[0:2] = rot_matrix @ (start_point[0:2] - center) + center + gear * trans_matrix
        endpoint[2, 0] = self.M(cur_theta + rot_theta)

        cur_length = 0

        if length < 0:
            length = 2 * pi * self.min_r + length

        while cur_length < length:

            next_x = cur_x + gear * cos(cur_theta) * step_size
            next_y = cur_y + gear * sin(cur_theta) * step_size
            next_theta = cur_theta + gear * curvature * step_size

            d_length = np.sqrt( (next_x - cur_x) **2 + (next_y - cur_y) ** 2 )
            cur_length = cur_length + d_length

            next_point = np.array([[next_x], [next_y], [next_theta]])

            sample_path.poses.append(self.point2pose(next_point))

            cur_x = next_x
            cur_y = next_y
            cur_theta = next_theta

        sample_path.poses.append(self.point2pose(endpoint))

        return sample_path, driving_direction, endpoint

    def pose2point(self, pose):

        x = pose.position.x
        y = pose.position.y

        quater_x = pose.orientation.x
        quater_y = pose.orientation.y
        quater_z = pose.orientation.z
        quater_w = pose.orientation.w

        r = rot.from_quat([quater_x, quater_y, quater_z, quater_w])
        euler_rad = r.as_euler('xyz')
        theta = euler_rad[2]

        return np.array([[x], [y], [theta]])

    def point2pose(self, point):

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = point[0, 0]
        pose.pose.position.y = point[1, 0]
        
        theta = point[2, 0]

        r = rot.from_euler('z', theta)
        quat = r.as_quat()

        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]

        return pose


class CarlaToRosWaypointConverter:
    """
    This class generates a plan of waypoints to follow.

    The calculation is done whenever:
    - the hero vehicle appears
    - a new goal is set
    """
    WAYPOINT_DISTANCE = 2.0

    def __init__(self):
        """
        Constructor
        """
        # super(CarlaToRosWaypointConverter, self).__init__('carla_waypoint_publisher')
        self.connect_to_carla()
        self.map = self.world.get_map()
        self.ego_vehicle = None
        self.ego_vehicle_location = None
        self.on_tick = None
        self.role_name = rospy.get_param("role_name", 'ego_vehicle')
        self.waypoint_publisher = rospy.Publisher('/carla/{}/waypoints'.format(self.role_name), Path, latch=True,
                                                  queue_size=1)
        
        self.path_markers_publisher = rospy.Publisher('/carla/{}/path_markers'.format(self.role_name), MarkerArray, latch=True,
                                                  queue_size=1)
        self.markers = MarkerArray()                                          
                                                

        # set initial goal
        self.goal = None

        self.current_route = None

        self.rs_curve = ReedsSheppROS(min_radius=1)
        self.pose_list = []

        self.route_polanner_server = actionlib.SimpleActionServer("compute_path_to_goal",
                                                                  PathPlannerAction,
                                                                  execute_cb=self.execute_cb,
                                                                  auto_start=False)
        self.route_polanner_server.start()
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
                self.route_polanner_server.set_aborted(text="Error: ego_vehicle or goal not valid!")
            elif self.is_goal_reached(self.goal):
                self.route_polanner_server.set_aborted(text="Already reached goal!")
            else:       
                vehicle_pose = trans.carla_transform_to_ros_pose(self.ego_vehicle.get_transform())     
                self.pose_list.append(vehicle_pose)
                self.pose_list.append(goal_msg.goal.pose)
                self._result.path = self.rs_curve.shortest_path(self.pose_list)
                self.publish_path_array_markers(self._result.path)
                self.route_polanner_server.set_succeeded(self._result, "success")
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
                self.route_polanner_server.set_aborted(text="Error: ego_vehicle or goal not valid!")
            elif self.is_goal_reached(self.goal):
                self.route_polanner_server.set_aborted(text="Already reached goal!")
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
                    self.route_polanner_server.set_aborted(self._result, result_info)
                else:
                    self.waypoint_publisher.publish(self._result.path.paths[0])
                    self.route_polanner_server.set_succeeded(self._result, result_info)
        

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

    waypoint_converter = None
    try:
        waypoint_converter = CarlaToRosWaypointConverter()
        rospy.spin()
    except (RuntimeError, ROSException):
        pass
    except KeyboardInterrupt:
        print("User requested shut down.")
    finally:
        print("Shutting down.")
        if waypoint_converter:
            waypoint_converter.destroy()


if __name__ == "__main__":
    main()
