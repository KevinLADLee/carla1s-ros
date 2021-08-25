import math

from carla_nav_msgs.msg import ParkingSpot

from geometry_msgs.msg import Pose, PoseStamped, Point
import rospy
import tf
from carla_vertical_parking.car_parking.find_best_parking_place import GetParkingEndPosition

#这里面的信息都是无法从carla的vertical info中获取的参数
class EnvInfoMessage(float):
    road_w = 5 # 2.86*3 #尽量在4.09以上，小于的话腾挪次数要爆炸
    car_l = 4
    hou_xuan = 1
    step=0.1
    road_l=10

class GetBestParkingPosition():
    def __init__(self,vehicle_info):
        self.vehicle_info=vehicle_info

    def get_best_parking_position(self, vehicle_pose: Pose, parking_spot: ParkingSpot):
        msg = parking_spot.center_pose
        (_, _, parking_theta) = tf.transformations.euler_from_quaternion(
            [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        center_pose = msg.position

        # 无法从carla中获取的信息
        hou_xuan = EnvInfoMessage.hou_xuan
        road_w = EnvInfoMessage.road_w
        car_l = EnvInfoMessage.car_l

        # 能够从carla中获取的信息
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

        # 初始化寻找最优停车位方法
        get_park = GetParkingEndPosition(car_l, car_w, min_turning_radiu, wheel_dis, hou_xuan, road_w,
                                         real_parking_left_head, real_parking_right_head)

        # 获取理论最优点
        # 这个理论最优点不考虑车辆当前位置，得出的是车辆一次转弯能转到的理论极限的最优位置
        real_position_x, real_position_y, real_position_theta = get_park.get_best_place()

        # print('best ', real_position_x, real_position_y, real_position_theta)

        best_position = PoseStamped()
        best_position.pose.position.x = real_position_x
        best_position.pose.position.y = real_position_y

        # 停车位的角度
        q = tf.transformations.quaternion_from_euler(0, 0, real_position_theta)

        best_position.pose.orientation.x = q[0]
        best_position.pose.orientation.y = q[1]
        best_position.pose.orientation.z = q[2]
        best_position.pose.orientation.w = q[3]
        return best_position
