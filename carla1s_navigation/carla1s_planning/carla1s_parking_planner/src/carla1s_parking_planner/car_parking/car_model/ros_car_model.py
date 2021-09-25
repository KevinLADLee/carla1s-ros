import math
import numpy as np
from matplotlib import pyplot as plt
import rospy
from nav_msgs.msg import Odometry
from carla_msgs.msg import CarlaEgoVehicleStatus
from std_msgs.msg import Float32
from  car_parking.car_model.simple_car_model import SimpleCarModel

class CarModel(SimpleCarModel):
    def __init__(self,x,y,theta,v=0,w=0, cl=None, cw=None, min_r=None, wl=None,
                 hx=None, step=None):
        SimpleCarModel.__init__(self, x,y,theta,v,w, cl, cw, min_r, wl,hx, step)
        self.mode="ros"



    def update(self,v,w):
        # print("set ",v,w)
        rospy.set_param("/parking_planning/car_speed_need", v)
        rospy.set_param("/parking_planning/car_steer_need", w)

        data = rospy.wait_for_message('/carla/ego_vehicle/odometry', Odometry, 10)

        x = data.pose.pose.orientation.x
        y = data.pose.pose.orientation.y
        z = data.pose.pose.orientation.z
        w = data.pose.pose.orientation.w

        self._position_x = data.pose.pose.position.x
        self._position_y = data.pose.pose.position.y
        self._position_theta = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

        self._position_x, self._position_y, self._position_theta=self.__get_backshaft_center_from_center(
            self._position_x,self._position_y,self._position_theta)

        data = rospy.wait_for_message('/carla/ego_vehicle/speedometer', Float32, 10)
        self._v = data.data
        # self.__position_x = rospy.get_param("/parking_planning/position_x")
        # self.__position_y = rospy.get_param("/parking_planning/position_y")
        # self.__position_theta = rospy.get_param("/parking_planning/position_theta")
        # self.__v = rospy.get_param("/parking_planning/car_speed")


    def __get_backshaft_center_from_center(self, x, y, theta):
        backdis = self._car_l / 2 - self._hou_xuan

        return x - backdis * math.cos(theta), y - backdis * math.sin(theta), theta

    def stop(self):
        while (1):
            self._v = rospy.get_param("/parking_planning/car_speed")
            rospy.set_param("/parking_planning/car_speed_need", 0)
            rospy.set_param("/parking_planning/car_steer_need", 0)
            # print('force stop , now speed = ',self.__v )
            if abs(self._v) < 0.1:
                break




if __name__ == "__main__":
    parking_l = 5.3
    road_w = 2.86 * 1.6  # 2.86*3 #尽量在4.09以上，小于的话腾挪次数要爆炸
    car_l = 4.7
    car_w = 1.7
    min_turning_radiu = 5.3
    wheel_dis = 2.6
    road_l = 3 * parking_l
    step = 0.1
    hou_xuan = (car_l - wheel_dis) / 2
    car0=CarModel(0, 0, 0)
    #cl, cw, min_r, wl, hx, step
    car0.set_infomation(car_l,car_w,min_turning_radiu,wheel_dis,hou_xuan,step)

    import random
    random.seed(0)
    for i in range(100):
        v=random.randint(0,10)/10
        w=random.randint(-3,3)/10
        plt.xlim(-10,10)
        plt.ylim(-10,10)
        car0.update(v,w)
        car0.draw_car()
        plt.pause(step)
        plt.clf()
        print("v w ",v,w)
        print("position",car0.get_position())




