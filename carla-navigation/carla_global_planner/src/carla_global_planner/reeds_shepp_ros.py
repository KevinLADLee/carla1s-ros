
from rospy import Time
from nav_msgs.msg import Path
from carla_nav_msgs.msg import Path as PathArray
from geometry_msgs.msg import PoseStamped

from carla_global_planner.reeds_shepp import ReedsShepp

class ReedsSheppROS(ReedsShepp):

    def __init__(self, min_radius):
        super().__init__(min_radius)

    def shortest_path(self, pose_list, step_size=0.01):

        path_array = PathArray()
        path_array.header.frame_id = "map"
        path_array.header.stamp = Time.now()

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
        sample_path.header.stamp = Time.now()
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
