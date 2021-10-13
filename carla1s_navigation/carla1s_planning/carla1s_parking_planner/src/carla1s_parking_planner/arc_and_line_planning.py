import math

from carla1s_msgs.msg import ParkingSpot

from geometry_msgs.msg import Pose, PoseStamped, Point
import rospy
import tf
from carla1s_parking_planner.car_parking.find_best_parking_place import GetParkingEndPosition


from nav_msgs.msg import Path, Odometry
from carla1s_msgs.msg import PathArray
from carla1s_parking_planner.car_parking.env import Env
from carla1s_parking_planner.car_parking.parking_planning import Planning

#这里面的信息都是无法从carla的vertical info中获取的参数
class EnvInfoMessage(float):
    road_w = 5 # 2.86*3 #尽量在4.09以上，小于的话腾挪次数要爆炸
    car_l = 4
    hou_xuan = 1
    step=0.1
    road_l=10

class ArcLinePlanning():
    def __init__(self,vehicle_info):
        self.vehicle_info=vehicle_info

    def planning(self, vehicle_pose: Pose, parking_spot: ParkingSpot):
        msg = parking_spot.center_pose
        (_, _, parking_theta) = tf.transformations.euler_from_quaternion(
            [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        center_pose = msg.position

        # 无法从carla中获取的信息
        hou_xuan = EnvInfoMessage.hou_xuan
        road_w = EnvInfoMessage.road_w
        car_l = EnvInfoMessage.car_l
        step = EnvInfoMessage.step
        road_l = EnvInfoMessage.road_l

        # print(self.vehicle_info)

        # 能够从carla中获取的信息
        parking_l = parking_spot.length
        parking_w = parking_spot.width
        car_w = abs(self.vehicle_info.wheels[0].position.y) + abs(self.vehicle_info.wheels[1].position.y)
        wheel_dis = abs(self.vehicle_info.wheels[0].position.x) + abs(self.vehicle_info.wheels[2].position.x)

        # 最小转弯半价的经验计算方法
        min_turning_radiu = 2.4 * car_l
        real_parking_left_head = [center_pose.x + math.cos(parking_theta) * parking_spot.length / 2 - math.sin(
            parking_theta) * parking_spot.width / 2,
                                  center_pose.y + math.sin(parking_theta) * parking_spot.length / 2 + math.cos(
                                      parking_theta) * parking_spot.width / 2]
        real_parking_right_head = [center_pose.x + math.cos(parking_theta) * parking_spot.length / 2 + math.sin(
            parking_theta) * parking_spot.width / 2,
                                   center_pose.y + math.sin(parking_theta) * parking_spot.length / 2 - math.cos(
                                       parking_theta) * parking_spot.width / 2]

        parking_left_point = real_parking_left_head
        parking_right_point = real_parking_right_head

        # 获取车辆位置
        car_position_x = vehicle_pose.position.x
        car_position_y = vehicle_pose.position.y
        (_, _, car_position_theta) = tf.transformations.euler_from_quaternion(
            [vehicle_pose.orientation.x, vehicle_pose.orientation.y,
             vehicle_pose.orientation.z, vehicle_pose.orientation.w])

        # 整合env类用作planning输入
        env0 = Env()
        env0.set_car_info(car_l, car_w, min_turning_radiu, wheel_dis, hou_xuan, step)
        env0.set_env_info(parking_l, parking_w, road_w, road_l, parking_left_point, parking_right_point)

        # 设置当前车位的泊车路径规划类
        plan = Planning(env0)
        route = []

        path_array = PathArray()

        try:
            route_x, route_y, route_theta_r, dir_info = plan.planning(car_position_x, car_position_y,
                                                                      car_position_theta)

            for i in range(len(route_x)):
                route.append([route_x[i], route_y[i], route_theta_r[i]])

            now_path_dir = dir_info[0]
            now_route = [[route_x[0], route_y[0], route_theta_r[0]]]
            routes_dir = []
            routes = []
            type_route = Path()
            path_array.driving_direction.append(dir_info[0])
            for i in range(1, len(dir_info)):
                tmp_pose = PoseStamped()
                tmp_pose.pose.position.x = route_x[i]
                tmp_pose.pose.position.y = route_y[i]

                q = tf.transformations.quaternion_from_euler(0, 0, route_theta_r[i])
                tmp_pose.pose.orientation.x = q[0]
                tmp_pose.pose.orientation.y = q[1]
                tmp_pose.pose.orientation.z = q[2]
                tmp_pose.pose.orientation.w = q[3]
                # type_route.append(tmp_pose)

                if dir_info[i] == now_path_dir:
                    now_route.append([route_x[i], route_y[i], route_theta_r[i]])
                    type_route.poses.append(tmp_pose)
                else:
                    routes.append(now_route)
                    routes_dir.append(now_path_dir)
                    now_route = [[route_x[i], route_y[i], route_theta_r[i]]]
                    now_path_dir = dir_info[i]

                    path_array.paths.append(type_route)
                    path_array.driving_direction.append(now_path_dir)
                    type_route = Path()
                    type_route.poses.append(tmp_pose)

            path_array.paths.append(type_route)

            routes.append(now_route)
            routes_dir.append(now_path_dir)


        except:
            print("planning fail")

        return path_array
