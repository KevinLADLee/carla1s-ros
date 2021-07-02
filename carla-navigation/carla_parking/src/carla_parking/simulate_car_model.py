import math
import numpy as np
from matplotlib import pyplot as plt


class car_model():
    def __init__(self,x,y,theta,v=0,w=0, cl=None, cw=None, min_r=None, wl=None,
                 hx=None, step=None):
        self.__car_l = cl
        self.__car_w = cw
        self.__min_turning_radiu = min_r
        self.__wheel_dis = wl
        self.__hou_xuan = hx
        self.__step = step

        self.__position_x=x
        self.__position_y=y
        self.__position_theta=theta
        self.__v=v
        self.__w=w
        self.mode="simulate"


    def set_infomation(self, cl, cw, min_r, wl, hx, step):
        self.__car_l = cl
        self.__car_w = cw
        self.__min_turning_radiu = min_r
        self.__wheel_dis = wl
        self.__hou_xuan = hx
        self.__step = step

    def update(self,v,w):
        self.__position_x = self.__position_x + v * np.cos(self.__position_theta) * self.__step
        self.__position_y = self.__position_y + v * np.sin(self.__position_theta) * self.__step
        self.__position_theta = self.__position_theta + w * self.__step
        self.__v=v
        self.__w=w


    # #模拟，使用速度和角度进行状态更新
    # def simulate_update_information(self, x, y, theta, v, w):
    #     x = x + v * np.cos(theta) * self.dt
    #     y = y + v * np.sin(theta) * self.dt
    #     theta = theta + w * self.dt
    #     return x, y, theta, v
    #
    # def simulate_force_stop(self):
    #     return 0


    def get_infomation(self):
        return self.__position_x, self.__position_y, self.__position_theta,self.__v,self.__w

    def set_position(self,x,y,theta):
        self.__position_x=x
        self.__position_y=y
        self.__position_theta=theta
        return self.__position_x, self.__position_y, self.__position_theta

    def stop(self):
        self.__v=0
        self.__w=0
        return

    def draw_car(self,color='black'):

        whx1= self.__position_x - math.sin(self.__position_theta) * self.__car_w / 2
        whx2= self.__position_x + math.sin(self.__position_theta) * self.__car_w / 2
        why1= self.__position_y + math.cos(self.__position_theta) * self.__car_w / 2
        why2= self.__position_y - math.cos(self.__position_theta) * self.__car_w / 2

        #车后轮的轴线
        x1 = [whx1,  whx2]
        y1 = [why1, why2]
        plt.plot(x1, y1,color)
        #车边框
        #车左边框
        x1 = [whx1 - (self.__hou_xuan) * math.cos(self.__position_theta),
              whx1 + (self.__car_l - self.__hou_xuan) * math.cos(self.__position_theta)]
        y1 = [why1 - (self.__hou_xuan) * math.sin(self.__position_theta),
              why1 + (self.__car_l - self.__hou_xuan) * math.sin(self.__position_theta)]
        plt.plot(x1, y1, color)
        #车右边框
        x1 = [whx2 - (self.__hou_xuan) * math.cos(self.__position_theta),
              whx2 + (self.__car_l - self.__hou_xuan) * math.cos(self.__position_theta)]
        y1 = [why2 - (self.__hou_xuan) * math.sin(self.__position_theta),
              why2 + (self.__car_l - self.__hou_xuan) * math.sin(self.__position_theta)]
        plt.plot(x1, y1, color)
        # 车尾边框
        x1 = [whx1 - (self.__hou_xuan) * math.cos(self.__position_theta),
              whx2 - (self.__hou_xuan) * math.cos(self.__position_theta)]
        y1 = [why1 - (self.__hou_xuan) * math.sin(self.__position_theta),
              why2 - (self.__hou_xuan) * math.sin(self.__position_theta)]
        plt.plot(x1, y1,color)
        # 车头边框
        x1 = [whx1 + (self.__car_l - self.__hou_xuan) * math.cos(self.__position_theta),
              whx2 + (self.__car_l - self.__hou_xuan) * math.cos(self.__position_theta)]
        y1 = [why1 + (self.__car_l - self.__hou_xuan) * math.sin(self.__position_theta),
              why2 + (self.__car_l - self.__hou_xuan) * math.sin(self.__position_theta)]
        plt.plot(x1, y1, color)


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
    car0=car_model(0,0,0)
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
        print("position", car0.get_infomation())




