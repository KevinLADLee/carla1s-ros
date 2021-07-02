import copy
import math
import os
import sys
import threading

import matplotlib.pyplot as plt

from Vertical_Parking import vertical_parking
from Env import ENV
from Space_Change import S_change
import rospy
import Carla_Info_Setting

class Planning():
    def __init__(self,env):
        self.env=env

        # 转换成算法所能理解的坐标系
        simulate_parking_left_point = [self.env.parking_w, 0]
        simulate_parking_right_point = [2 * self.env.parking_w, 0]

        # 设置坐标转换函数
        self.space_change = S_change([self.env.parking_left_front_point_x, self.env.parking_left_front_point_y],
                                     [self.env.parking_right_front_point_x, self.env.parking_right_front_point_y]
                                     , simulate_parking_left_point, simulate_parking_right_point)


        #算法的环境需要进行重新设置
        self.sim_env=copy.deepcopy(self.env)
        self.sim_env.set_env_info(self.env.parking_l, self.env.parking_w, self.env.road_w, self.env.road_l, [self.env.parking_w, 0], [self.env.parking_w * 2, 0])

        #设置垂直停车算法
        self.__parking_algorithm=vertical_parking(self.sim_env)

    # ==================================================================
    # 函数名：reset_env
    # 作者：孙耀威
    # 日期：2021.6.15
    # 功能：重新设置泊车规划的环境
    # 输入参数：env 环境类
    # 返回值：
    # 修改记录：
    # ==================================================================
    def reset_env(self,env):
        self.env=env
        # 转换成算法所能理解的坐标系
        simulate_parking_left_point = [self.env.parking_w, 0]
        simulate_parking_right_point = [2 * self.env.parking_w, 0]

        # 设置坐标转换函数
        self.space_change = S_change([self.env.parking_left_front_point_x, self.env.parking_left_front_point_y],
                                     [self.env.parking_right_front_point_x, self.env.parking_right_front_point_y]
                                     , simulate_parking_left_point, simulate_parking_right_point)

        # 算法的环境需要进行重新设置
        sim_env = copy.deepcopy(self.env)
        sim_env.set_env_info(self.env.parking_l, self.env.parking_w, self.env.road_w, self.env.road_l,
                             [self.env.parking_w, 0], [self.env.parking_w * 2, 0])

        # 设置垂直停车算法
        self.__parking_algorithm = vertical_parking(sim_env)

    # ==================================================================
    # 函数名：planning
    # 作者：孙耀威
    # 日期：2021.6.15
    # 功能：实车泊车规划
    # 输入参数：车辆在实际坐标系下的坐标
    # 返回值：三个list ，分别是路径x,y,theta(弧度制)的集合
    # 修改记录：
    # ==================================================================
    def planning(self,real_car_position_x,real_car_position_y,real_car_position_theta):

        # 计算车辆在算法坐标系中的位置
        sim_car_x, sim_car_y, sim_car_theta = self.space_change.real_to_sim(real_car_position_x, real_car_position_y, real_car_position_theta)

        # 计算路径
        route_x, route_y, route_theta_r =self.__parking_algorithm.planning(sim_car_x, sim_car_y, sim_car_theta)

        #将算法路径转换成实际路径
        real_route_x=[]
        real_route_y=[]
        real_route_theta_r=[]
        for  i in range(len(route_x)):
            x,y,r=self.space_change.sim_to_real(route_x[i], route_y[i], route_theta_r[i])
            real_route_x.append(x)
            real_route_y.append(y)
            real_route_theta_r.append(r)


        return real_route_x, real_route_y, real_route_theta_r




def test_simulate():
    # # 车辆实际坐标
    # car_x = -33
    # car_y = 30
    # car_theta_r = math.pi / 2
    #
    # # 车位实际坐标
    # real_parking_left_head = [-31.6, 25.5]
    # real_parking_right_head = [-31.6, 28.7]

    #车辆实际坐标
    car_x=-37.131065
    car_y=-14.28638
    car_theta_r=-1.356844


    # 车位实际坐标
    real_parking_left_head = [-42.3, -3.1]
    real_parking_right_head = [-42.3, -6.1]

    # 车位数据，对于垂直停车，车位宽度由车位前侧左右两点决定，不需要输入
    parking_l = 7  # 魔改成7，实际标准是6
    road_w = 7
    road_l = 3 * parking_l

    # 进行规划所需要的车辆数据
    car_l = 4.7
    car_w = 1.7
    min_turning_radiu = 13
    wheel_dis = 2.6
    step = 0.3
    hou_xuan = (car_l - wheel_dis) / 2
    parking_w = math.sqrt(math.pow(real_parking_left_head[0] - real_parking_right_head[0], 2) + math.pow(
        real_parking_left_head[1] - real_parking_right_head[1], 2))

    env0 = ENV()
    env0.set_env_info(parking_l, parking_w, road_w, road_l, real_parking_left_head, real_parking_right_head)
    env0.set_car_info(car_l, car_w, min_turning_radiu, wheel_dis, hou_xuan, step)

    plan = Planning(env0)

    route_x, route_y, route_theta_r = plan.planning(car_x, car_y, car_theta_r)

    print('----------------------------')
    for i in range(len(route_x)):
        print(route_x[i], route_y[i], route_theta_r[i])

    plt.plot(route_x,route_y)
    plt.show()


"""
函数名称：ros_planning
输入参数：
    car_real_position_x,car_real_position_y,car_real_position_theta 车辆坐标
    parking_l  车位长度 根据实际标准填写
    road_w     道路宽度
    road_l     道路长度
    car_l        车辆长度
    car_w       车辆宽度
    min_turning_radiu  规划所要求的最小转弯半径
    wheel_dis     车辆轴距
    step     运动步长(常用于模拟时间长度)
    hou_xuan    车辆后悬长度
    real_parking_left_head 车位左前角
    real_parking_right_head 车位右前角
    
输出结果：
   route_x, route_y, route_theta_r 泊车路径的轨迹点的集合
   起点是车辆坐标，终点是车位内部的一个点
   route theta r 是路径点的角度值，每个点都有一个推荐角度（又或者是点的斜率换成的角度），单位是弧度制的
   
   【注意】如果当前车位位置不存在一条泊车路径，那么会返回None,None,None

函数功能：
    计算一条从给定车辆坐标的位置到车位内部的路线
    
函数流程：
     1.将所有环境变量的参数打包进env（环境）类中
     2.将环境类代入规划模块进行环境初始化
     3.调用规划模块的planning方法并获取一条到车位的路径
"""
def ros_planning(car_x, car_y, car_theta_r,
                 parking_l, road_w, road_l, car_l, car_w, min_turning_radiu, wheel_dis, step, hou_xuan
                 , real_parking_left_head, real_parking_right_head):


    parking_w = math.sqrt(math.pow(real_parking_left_head[0] - real_parking_right_head[0], 2) + math.pow(
        real_parking_left_head[1] - real_parking_right_head[1], 2))

    env0 = ENV()
    env0.set_env_info(parking_l, parking_w, road_w, road_l, real_parking_left_head, real_parking_right_head)
    env0.set_car_info(car_l, car_w, min_turning_radiu, wheel_dis, hou_xuan, step)

    plan = Planning(env0)

    route_x, route_y, route_theta_r = plan.planning(car_x, car_y, car_theta_r)


    if route_x==None:
        print("规划路径失败，请移动车辆")
        return

    # print('----------------------------')
    # for i in range(len(route_x)):
    #     print(route_x[i], route_y[i], route_theta_r[i])

    plt.plot(route_x,route_y)
    plt.plot([real_parking_right_head[0],real_parking_left_head[0]],[real_parking_right_head[1],real_parking_left_head[1]])
    plt.show()


    print("test ros ",car_x, car_y, car_theta_r)


if __name__ == "__main__":
    #参数来源
    #参数来源可以选择ros 或者其他，其他的时候就自己规定车位坐标与车辆坐标
    # car_position_from="11"
    car_position_from="ros"

    #===============================================================================
    #参数获取

    # 获取车辆位置的方法
    # 当选择ros时，可以用来验证当前车辆所在位置是否能有一条停进车位的路线
    # 不选择ros可以用来测试算法本身有没有问题
    if car_position_from=="ros":
        #从ros中读取车辆坐标
        rospy.set_param("/parking_planning/parking_end", 1)
        info=Carla_Info_Setting.info_set()
        car_real_position_x,car_real_position_y,car_real_position_theta =info.get_car_position()
        rospy.set_param("/parking_planning/parking_end", 0)
    else:
        #20.358104705810547 17.876855850219727 -1.5735255342800099
        #车辆虚拟坐标
        car_real_position_x =20.358104705810547
        car_real_position_y =17.876855850219727
        car_real_position_theta =-1.5735255342800099



    # # 车位实际坐标,（前是指靠近车道的那边，左右是指站在车位里看向车道的方向的左右）车位左前角，车位右前角
    # real_parking_left_head = [-42.3, -3.1]
    # real_parking_right_head = [-42.3, -6.1]

    # 新车位车位实际坐标,（前是指靠近车道的那边，左右是指站在车位里看向车道的方向的左右）车位左前角，车位右前角
    real_parking_left_head = [16.6 ,28.4]
    real_parking_right_head = [16.6,26.1]

    # 车位数据，
    parking_l = 7  #车位长度 根据实际标准填写
    road_w = 7  #道路宽度
    road_l = 3 * parking_l  #道路长度

    # 进行规划所需要的车辆数据
    car_l = 4.7    #车辆长度
    car_w = 1.7    #车辆宽度
    min_turning_radiu = 8    #规划所要求的最小转弯半径
    wheel_dis = 2.6    #车辆轴距
    step = 0.3    #运动步长
    hou_xuan = (car_l - wheel_dis) / 2    #车辆后悬长度



    #===============================================================================
    #代码执行部分
    ros_planning(car_real_position_x, car_real_position_y, car_real_position_theta,
                 parking_l, road_w, road_l, car_l, car_w, min_turning_radiu, wheel_dis, step, hou_xuan
                 , real_parking_left_head, real_parking_right_head)
    # test_simulate()

