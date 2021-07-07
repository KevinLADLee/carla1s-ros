import numpy as np
from carla_global_planner.reeds_shepp_ros.reeds_shepp import reeds_shepp
from transforms3d.euler import quat2euler, euler2quat
from geometry_msgs.msg import Pose
from carla_nav_msgs.msg import Path as rpath
from nav_msgs.msg import Path as npath
from geometry_msgs.msg import PoseStamped

class reeds_ros_inter(reeds_shepp):

    def __init__(self, min_radius = 1):
        super().__init__(min_radius)

    def shortest_path(self, pose_list, step_size=0.01):

        final_path = rpath()

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

            path = self.reeds_path_generate(start_point, path_min, step_size)
        
            final_path.paths = final_path.paths + path.paths
            final_path.driving_direction = final_path.driving_direction + path.driving_direction

        return final_path

    def reeds_path_generate(self, start_point, path, step_size):

        path_segment_list = rpath()
        
        end_point = None

        if len(path) == 0:
            print('no path')
            return path_segment_list

        for i in range(len(path)):
            
            path_seg, end_point = self.element_sample(element=path[i], start_point=start_point, step_size=step_size)

            start_point = end_point

            path_segment_list.paths.append(path_seg)
            path_gear = 0 if path[i].gear == 1 else 1
            path_segment_list.driving_direction.append(path_gear)

        return path_segment_list

    def element_sample(self, element, start_point, step_size):
        
        Path_seg = npath()
        Path_seg.poses.append(self.point2pose_stamp(start_point))

        length = element.len * self.min_r
        cur_length = 0

        while cur_length < length:

            pre_length = cur_length + step_size

            if cur_length <= length and pre_length > length:
                step_size = length - cur_length

            next_point = self.motion_acker_step(start_point, element.gear, element.steer, step_size)

            Path_seg.poses.append(self.point2pose_stamp(next_point))

            cur_length = cur_length + step_size
            start_point = next_point

        return Path_seg, next_point

    def pose2point(self, pose):

        x = pose.position.x
        y = pose.position.y

        quater_x = pose.orientation.x
        quater_y = pose.orientation.y
        quater_z = pose.orientation.z
        quater_w = pose.orientation.w

        _, _, theta = quat2euler([quater_w, quater_x, quater_y, quater_z])

        return np.array([[x], [y], [theta]])

    def point2pose(self, point):

        pose = Pose()
        pose.position.x = point[0, 0]
        pose.position.y = point[1, 0]
        
        quat = euler2quat(0, 0, point[2, 0])

        pose.orientation.w = quat[0]
        pose.orientation.x = quat[1]
        pose.orientation.y = quat[2]
        pose.orientation.z = quat[3]
    
        return pose

    def point2pose_stamp(self, point):

        pose_stamp = PoseStamped()
        pose_stamp.pose.position.x = point[0, 0]
        pose_stamp.pose.position.y = point[1, 0]
        
        quat = euler2quat(0, 0, point[2, 0])

        pose_stamp.pose.orientation.w = quat[0]
        pose_stamp.pose.orientation.x = quat[1]
        pose_stamp.pose.orientation.y = quat[2]
        pose_stamp.pose.orientation.z = quat[3]
    
        return pose_stamp


