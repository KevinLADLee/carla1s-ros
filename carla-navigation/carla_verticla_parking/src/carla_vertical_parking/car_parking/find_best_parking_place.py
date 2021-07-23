import matplotlib.pyplot as plt

from .car_model.simulate_car_model import CarModel
import math
from .space_change import SpaceChange

class GetParkingEndPosition():
    def __init__(self, cl=None, cw=None, min_r=None, wl=None,
                 hx=None,rw=None,lp=None,rp=None):
        self.__road_w=rw
        self.__car_l = cl
        self.__car_w = cw
        self.__min_turning_radiu = min_r
        self.__wheel_dis = wl
        self.__hou_xuan = hx
        self.__lp=lp
        self.__rp=rp

        self.__parking_w=self.__distance(self.__lp[0], self.__lp[1],self.__rp[0], self.__rp[1])

        # 转换成算法所能理解的坐标系
        simulate_parking_left_point = [self.__parking_w, 0]
        simulate_parking_right_point = [2 * self.__parking_w, 0]

        # 设置坐标转换函数
        self.space_change = SpaceChange([self.__lp[0], self.__lp[1]],
                                        [self.__rp[0], self.__rp[1]]
                                        , simulate_parking_left_point, simulate_parking_right_point)

        #计算常用参数
        self.__r_add_front = math.sqrt(math.pow(self.__min_turning_radiu + self.__car_w / 2, 2) + math.pow(self.__car_l - self.__hou_xuan, 2))
        self.__r_dec_front = math.sqrt(math.pow(self.__min_turning_radiu - self.__car_w / 2, 2) + math.pow(self.__car_l - self.__hou_xuan, 2))
        self.__r_add_rear = math.sqrt(math.pow(self.__min_turning_radiu + self.__car_w / 2, 2) + math.pow(self.__hou_xuan, 2))
        self.__r_dec_rear = math.sqrt(math.pow(self.__min_turning_radiu - self.__car_w / 2, 2) + math.pow(self.__hou_xuan, 2))

    def __distance(self,x1,y1,x2,y2):
        return math.sqrt(math.pow(y1-y2,2)+math.pow(x1-x2,2))

    def __calculate_acos_by_atan2(self,a,c):
        return  math.atan2(math.sqrt(math.pow(c, 2) - math.pow(a, 2)),a)

    def __calculate_asin_by_atan2(self,a,c):
        return  math.atan2(a,math.sqrt(math.pow(c, 2) - math.pow(a, 2)))

    def __get_move_distance(self, x, y, theta):
        l=0
        r=20

        for k in range(l,r):
            t=k
            # print('theta -r ',theta_r,math.cos(theta_r),math.sin(theta_r))

            #求车辆移动t米后上圆心的位置
            x2 = x - self.__min_turning_radiu * math.sin(theta) + t * math.cos(theta)
            y2 = y + self.__min_turning_radiu * math.cos(theta) + t * math.sin(theta)

            # circle_line_method_parallel.draw_cz()
            # circle_line_method_parallel.draw_car(x2,y2,theta_r)
            # circle_line_method_parallel.draw_car(x,y,theta_r)
            #
            # plt.show()
            #计算转过多少角度倒车不会撞车
            cos_theta_r2p = (y2 - self.__road_w + self.__r_add_front) / (2 * self.__min_turning_radiu)
            if cos_theta_r2p > 1:
                continue
            theta_r2p = math.acos(cos_theta_r2p)
            theta_r2 = theta_r2p
            x3 = x2 + 2 * self.__min_turning_radiu * math.sin(theta_r2)
            y3 = y2 - 2 * self.__min_turning_radiu * math.cos(theta_r2)
            dis = self.__distance(self.__lp[0], self.__lp[1], x3, y3)
            if dis < min_turning_radiu-car_w/2:
                l=t
        # if l==0:
        #     print("初始化有问题！！！！！！！！！！！！！！！")
        #     return 0

        while (r-l>0.001):
            # print("l  r  ",l,r)
            mid=(l+r)/2
            t=mid
            x2 = x + t * math.cos(theta) - self.__min_turning_radiu * math.sin(theta)
            y2 = y + t * math.sin(theta) + self.__min_turning_radiu * math.cos(theta)
            cos_theta_r2p = (y2 - self.__road_w + self.__r_add_front) / (2 * self.__min_turning_radiu)
            if cos_theta_r2p > 1:
                break
            theta_r2p = math.acos(cos_theta_r2p)
            theta_r2 = theta_r2p
            x3 = x2 + 2 * self.__min_turning_radiu * math.sin(theta_r2)
            y3 = y2 - 2 * self.__min_turning_radiu * math.cos(theta_r2)
            dis = self.__distance(self.__lp[0], self.__lp[1], x3, y3)
            # print(t,dis, min_turning_radiu-car_w/2)
            if dis < min_turning_radiu-car_w/2:
                l=t
            if dis > min_turning_radiu-car_w/2:
                r=t
            if dis== min_turning_radiu-car_w/2:
                l=t
                r=t




        return l

    def __angle_init_turn(self,x, y, theta):

        y2 = y + self.__min_turning_radiu * math.cos(theta)
        cos_theta_r2p = (y2 - self.__road_w + self.__r_add_front) / (2 * self.__min_turning_radiu)
        theta_r2p = math.acos(cos_theta_r2p)
        return theta_r2p

    #寻找当前车辆位置相对车位的局部最优停车点
    def get_better_place(self,x,y,theta,show=False):
        # 计算在车位里，车辆左侧至多需要留多少距离才能保证不会撞到左侧车位
        x_min = math.pow(self.__hou_xuan, 2) / (
                    math.sqrt(math.pow(self.__min_turning_radiu + self.__car_w / 2, 2) + math.pow(self.__hou_xuan, 2)) + (
                        self.__min_turning_radiu + self.__car_w / 2))
        while (theta < 0):
            theta += math.pi * 2
        while (theta > math.pi * 2):
            theta -= math.pi * 2

        t=self.__get_move_distance(x,y,theta)

        # 添加路径
        mid_x = x + t * math.cos(theta)
        mid_y = y + t * math.sin(theta)
        # print(mid_x,mid_y)

        # 寻找在上方的圆心的坐标
        o_up_x = mid_x - self.__min_turning_radiu * math.cos(math.pi / 2 - theta)
        o_up_y = mid_y + self.__min_turning_radiu * math.sin(math.pi / 2 - theta)

        angle=self.__angle_init_turn(mid_x,mid_y,theta)

        # print(angle,angle*180/math.pi,(angle - math.pi / 2)*180/math.pi)
        # print( math.sin(angle - math.pi / 2),math.cos(angle - math.pi / 2))

        ed_x=o_up_x+self.__min_turning_radiu * math.cos(angle - math.pi / 2)
        ed_y=o_up_y+self.__min_turning_radiu * math.sin(angle - math.pi / 2)
        ed_theta=angle

        if show:
            plt.scatter(x, y)
            plt.scatter(o_up_x,o_up_y)
            plt.scatter(ed_x, ed_y)
            # plt.scatter(x1, self.__road_w)
            car0.set_position(x, y, theta)
            car0.draw_car()
            car0.set_position(mid_x, mid_y, theta)
            car0.draw_car()
            car0.set_position(ed_x, ed_y, ed_theta)
            car0.draw_car()
            # plt.scatter(x2, self.__road_w)
            plt.plot([0, 5 * self.__parking_w], [self.__road_w, self.__road_w])
            plt.plot([0, 5 * self.__parking_w], [0, 0])
            plt.plot([self.__parking_w, self.__parking_w], [0, -2])
            plt.plot([2 * self.__parking_w, 2 * self.__parking_w], [0, -2])
            # plt.plot([2 * self.__parking_w, 2 * self.__parking_w + 9], [0, 0 + math.tan(road_turn_max_r) * 9])
            plt.show()

        return ed_x, ed_y, ed_theta



    #通过车辆左前角的位置反推车辆后轴中心的额位置
    def __get_car_position_from_left_head(self,x,y,theta):
        x1=x-(self.__car_l-self.__hou_xuan)*math.cos(theta)
        car_x=x1+self.__car_w/2*math.sin(theta)
        y1=y-(self.__car_l-self.__hou_xuan)*math.sin(theta)
        car_y=y1-self.__car_w/2*math.cos(theta)

        return car_x,car_y,theta

    #寻找车位的全局最优停车点
    def get_best_place(self,show=False):
        # 计算在车位里，车辆左侧至多需要留多少距离才能保证不会撞到左侧车位
        x_min = math.pow(self.__hou_xuan, 2) / (
                    math.sqrt(math.pow(self.__min_turning_radiu + self.__car_w / 2, 2) + math.pow(self.__hou_xuan, 2)) + (
                        self.__min_turning_radiu + self.__car_w / 2))

        # 计算车辆平行于车道时最多能转多少度
        tan_theta_small1 = (self.__car_l - self.__hou_xuan) / (self.__min_turning_radiu - self.__car_w / 2)
        road_turn_max_r = self.__calculate_acos_by_atan2((self.__min_turning_radiu + x_min + self.__car_w / 2 - self.__road_w),math.sqrt(
            math.pow(self.__min_turning_radiu - self.__car_w / 2, 2) + math.pow(self.__car_l - self.__hou_xuan, 2)))- math.atan(tan_theta_small1)
        # print("【理论极限】（前进）通过车道宽度计算，车道上的车从平行行车到最大转角一次性最多能转过", road_turn_max_r * 180 / math.pi, "度")
        # x1=self.__road_w/math.tan(road_turn_max_r)+2*self.__parking_w+self.__car_w/math.sin(road_turn_max_r)
        x2=self.__road_w/math.tan(road_turn_max_r)+2*self.__parking_w-self.__car_w/math.sin(road_turn_max_r)

        car_x,car_y,car_theta=self.__get_car_position_from_left_head(x2,self.__road_w,road_turn_max_r)



        if show:
            plt.scatter(car_x,car_y)
            # plt.scatter(x1, self.__road_w)
            car0.set_position(car_x,car_y,car_theta)
            car0.draw_car()
            plt.scatter(x2, self.__road_w)
            plt.plot([0,5*self.__parking_w],[self.__road_w,self.__road_w])
            plt.plot([0,5*self.__parking_w],[0,0])
            plt.plot([self.__parking_w,self.__parking_w],[0,-2])
            plt.plot([2*self.__parking_w,2*self.__parking_w],[0,-2])
            plt.plot([2*self.__parking_w,2*self.__parking_w+9],[0,0+math.tan(road_turn_max_r)*9])
            plt.show()

        real_position_x,real_position_y,real_position_theta=self.space_change.sim_to_real(car_x,car_y,car_theta)

        return real_position_x,real_position_y,real_position_theta


if __name__ == "__main__":
    #基础参数
    parking_l = 5.3
    road_w = 2.86 * 1.6  # 2.86*3 #尽量在4.09以上，小于的话腾挪次数要爆炸
    car_l = 4.7
    car_w = 1.7
    min_turning_radiu = 10
    wheel_dis = 2.6
    road_l = 3 * parking_l
    step = 0.1
    hou_xuan = (car_l - wheel_dis) / 2
    # real_parking_left_head = [16.6 ,28.4]
    # real_parking_right_head = [16.6,26.1]
    real_parking_left_head = [3 ,0]
    real_parking_right_head = [6,0]

    #设置车辆模型
    car0=CarModel(0, 0, 0)
    car0.set_infomation(car_l,car_w,min_turning_radiu,wheel_dis,hou_xuan,step)

    #初始化寻找最优停车位方法
    get_park=GetParkingEndPosition(car_l, car_w, min_turning_radiu, wheel_dis, hou_xuan, road_w, real_parking_left_head, real_parking_right_head)

    #获取理论最优点
    #这个理论最优点不考虑车辆当前位置，得出的是车辆一次转弯能转到的理论极限的最优位置
    # real_position_x, real_position_y, real_position_theta =get_park.get_best_place()

    #获取当前位置的局部最优点
    real_position_x, real_position_y, real_position_theta =get_park.get_better_place(-10,1.5,0,show=True)
    print(real_position_x, real_position_y, real_position_theta)

    #真实坐标系下绘图，观察结果
    # plt.plot([real_parking_left_head[0],real_parking_right_head[0]],[real_parking_left_head[1],real_parking_right_head[1]])
    # car0.set_position(real_position_x, real_position_y, real_position_theta)
    # car0.draw_car()
    # plt.show()