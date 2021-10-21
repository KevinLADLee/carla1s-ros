import math
import time

import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator

import os
import sys

path1 = os.path.split(os.path.abspath(__file__))[0].split("/")
add_path=""
for i in range(1,len(path1)-2):
   add_path+="/"+path1[i]
sys.path.append(add_path)
# print("vertical parking.py 添加工作目录　",add_path)

sys.path.append(os.path.split(os.path.abspath(__file__))[0])

path1 = os.path.split(os.path.abspath(__file__))[0].split("/")
add_path=""
for i in range(1,len(path1)-1):
   add_path+="/"+path1[i]
sys.path.append(add_path)
# print("vertical parking.py 添加工作目录　",add_path)
from env import Env


class VerticalParking():
    def __init__(self,env,max_switch_times=20,log=False):
        self.env=env
        self.parking_left_front_point_x = self.env.parking_left_front_point_x
        self.parking_left_front_point_y = self.env.parking_left_front_point_y
        self.parking_right_front_point_x = self.env.parking_right_front_point_x
        self.parking_right_front_point_y = self.env.parking_right_front_point_y
        #是否在函数出错时提示
        self.log=log
        #计算常用参数
        self.r_add_front = math.sqrt(math.pow(self.env.min_turning_radiu + self.env.car_w / 2, 2) + math.pow(self.env.car_l - self.env.hou_xuan, 2))
        self.r_dec_front = math.sqrt(math.pow(self.env.min_turning_radiu - self.env.car_w / 2, 2) + math.pow(self.env.car_l - self.env.hou_xuan, 2))
        self.r_add_rear = math.sqrt(math.pow(self.env.min_turning_radiu + self.env.car_w / 2, 2) + math.pow(self.env.hou_xuan, 2))
        self.r_dec_rear = math.sqrt(math.pow(self.env.min_turning_radiu - self.env.car_w / 2, 2) + math.pow(self.env.hou_xuan, 2))
        # 车辆最大换挡次数
        self.max_switch_times = max_switch_times

    # ==================================================================
    # 函数名：__add_dir_route
    # 作者：孙耀威
    # 日期：2021.6.10
    # 功能：给定两个点和方向，生成一段包含方向的,两点之间间距为step的直线路径点
    # 输入参数：start_x, start_y 起点的坐标
    #         end_x, end_y 终点坐标
    #         car_position_theta 车辆在起点和终点的角度
    # 返回值：三个list ，分别是路径x,y,theta(弧度制)的集合
    # 修改记录：
    # ==================================================================
    def __add_dir_route(self, start_x, start_y, end_x, end_y, theta_r):
        #初始化
        route_x_list=[]
        route_y_list=[]
        route_theta_list=[]

        #通过两点之间的距离计算应该添加的点的数目
        dis_start_end=math.sqrt(math.pow(start_x-end_x,2)+math.pow(start_y-end_y,2))
        point_number=(int)((dis_start_end ) / self.env.step)

        if point_number!=0:
            #通过点的数目计算两点之间的坐标差值
            derta_x=(end_x-start_x)/point_number
            derta_y=(end_y-start_y)/point_number

            #避免生成的路径点超过终点
            if point_number*self.env.step>dis_start_end:
                point_number-=1

            #添加路径点
            for i in range(point_number):
                route_x_list.append(start_x+i*derta_x)
                route_y_list.append(start_y+i*derta_y)
                route_theta_list.append(theta_r)

        route_x_list.append(end_x)
        route_y_list.append(end_y)
        route_theta_list.append(theta_r)

        return route_x_list, route_y_list, route_theta_list

    # ==================================================================
    # 函数名：__route_add_arc
    # 作者：孙耀威
    # 日期：2021.6.11
    # 功能：通过圆的参数方程获取两个点之间直连的弧线
    # 输入参数：ox, oy, r 圆的坐标与半径
    #         start_position_theta, end_position_theta 起点与终点的位置的角度
    #         start_dir_theta, end_dir_theta
    # 返回值：三个list ，分别是路径x,y,theta(弧度制)的集合
    # 修改记录： 可以像直线一样修改
    # ==================================================================
    def __add_arc_route(self, ox, oy, r, start_position_theta, end_position_theta, start_dir_theta, end_dir_theta):
        route_x = [ox + r * math.cos(start_position_theta)]
        route_y = [oy + r * math.sin(start_position_theta)]
        route_theta = [start_dir_theta]

        derta_theta = self.env.step / r

        if end_position_theta < start_position_theta:
            if start_position_theta - end_position_theta < math.pi:
                pr = start_position_theta - derta_theta
                p_dir_r = start_dir_theta
                while pr > end_position_theta:
                    route_x.append(ox + r * math.cos(pr))
                    route_y.append(oy + r * math.sin(pr))
                    route_theta.append(p_dir_r - derta_theta)
                    p_dir_r = p_dir_r - derta_theta
                    pr = pr - derta_theta
                route_x.append(ox + r * math.cos(end_position_theta))
                route_y.append(oy + r * math.sin(end_position_theta))
                route_theta.append(end_dir_theta)

            else:
                end_position_theta += 2 * math.pi
                pr = start_position_theta + derta_theta
                p_dir_r = start_dir_theta

                while pr < end_position_theta:
                    route_x.append(ox + r * math.cos(pr))
                    route_y.append(oy + r * math.sin(pr))
                    route_theta.append(p_dir_r + derta_theta)
                    pr = pr + derta_theta
                    p_dir_r = p_dir_r + derta_theta

                route_x.append(ox + r * math.cos(end_position_theta))
                route_y.append(oy + r * math.sin(end_position_theta))
                route_theta.append(end_dir_theta)
        else:
            if end_position_theta - start_position_theta < math.pi:
                pr = start_position_theta + derta_theta
                p_dir_r = start_dir_theta

                while pr < end_position_theta:
                    route_x.append(ox + r * math.cos(pr))
                    route_y.append(oy + r * math.sin(pr))
                    route_theta.append(p_dir_r + derta_theta)
                    pr = pr + derta_theta
                    p_dir_r = p_dir_r + derta_theta

                route_x.append(ox + r * math.cos(end_position_theta))
                route_y.append(oy + r * math.sin(end_position_theta))
                route_theta.append(end_dir_theta)
            else:
                end_position_theta -= 2 * math.pi
                pr = start_position_theta - derta_theta
                p_dir_r = start_dir_theta
                while pr > end_position_theta:
                    route_x.append(ox + r * math.cos(pr))
                    route_y.append(oy + r * math.sin(pr))
                    route_theta.append(p_dir_r - derta_theta)
                    p_dir_r = p_dir_r - derta_theta
                    pr = pr - derta_theta
                route_x.append(ox + r * math.cos(end_position_theta))
                route_y.append(oy + r * math.sin(end_position_theta))
                route_theta.append(end_dir_theta)

        return route_x, route_y, route_theta

    # ==================================================================
    # 函数名：__distance
    # 作者：孙耀威
    # 日期：2021.6.11
    # 功能：计算两点之间的欧式距离
    # 输入参数：x1,y1 起点坐标
    #         x2,y2 终点坐标
    # 返回值：两点之间的距离
    # 修改记录：
    # ==================================================================
    def __distance(self,x1,y1,x2,y2):
        return math.sqrt(math.pow(y1-y2,2)+math.pow(x1-x2,2))

    def __calculate_acos_by_atan2(self,a,c):
        return  math.atan2(math.sqrt(math.pow(c, 2) - math.pow(a, 2)),a)

    def __calculate_asin_by_atan2(self,a,c):
        return  math.atan2(a,math.sqrt(math.pow(c, 2) - math.pow(a, 2)))

    # ==================================================================
    # 函数名：__compare_route
    # 作者：孙耀威
    # 日期：2021.6.11
    # 功能：比较两条路径的优劣，返回其中更好的那条
    # 输入参数：route1_x, route1_y, route1_theta 第一条路径的信息
    #         route2_x, route2_y, route2_theta 第二条路径的信息
    # 返回值：三个list ，分别是路径x,y,theta(弧度制)的集合
    #        三个None 因为两条输入的路径都是None
    # 修改记录：
    # ==================================================================
    def __compare_route(self, route1_x, route1_y, route1_theta,
                        route2_x, route2_y, route2_theta):
        #如果两条路径有一条是不存在的路径，那么就直接选另外一条
        if route1_x==None :
            return route2_x, route2_y, route2_theta
        if route2_x==None:
            return route1_x, route1_y, route1_theta

        #通过比较长度选择路径
        if len(route1_x)<len(route2_x):
            return route1_x, route1_y, route1_theta
        if len(route1_x) >len(route2_x):
            return route2_x, route2_y, route2_theta

        #如果长度一致，比较终点位置，终点位置更靠近车位内部的优先
        if len(route1_x) ==len(route2_x):
            if (route1_y[-1]>route2_y[-1]):
                return route2_x, route2_y, route2_theta
            else:
                return route1_x, route1_y, route1_theta

    # ==================================================================
    # 函数名：__into_parking
    # 作者：孙耀威
    # 日期：2021.6.11
    # 功能：在车辆平行于车位的时候做入库的最后一段直线
    # 输入参数：car_position_x, y, theta 车辆当前的位置
    # 返回值：三个list ，分别是路径x,y,theta(弧度制)的集合
    # 修改记录：
    # ==================================================================
    def __into_parking(self,x, y, theta):
        if theta == math.pi / 2:
            #车停在车位内部底线或车头平行车道

            end_y0 = -self.env.car_l + self.env.hou_xuan
            end_y1 = -self.env.parking_l + self.env.hou_xuan
            # end_y=max(end_y1,end_y0)
            end_y=end_y1
            if y>end_y:
                return self.__add_dir_route(x, y, x, end_y, theta)

        return [], [], []

    # ==================================================================
    # 函数名：__move_dir_back
    # 作者：孙耀威
    # 日期：2021.6.11
    # 功能：计算车辆用 直线倒车的方法 移动到 车辆可以右转打死转进车位的位置
    #      针对车辆右转方向盘打死会撞到车位右边进行移动
    # 输入参数：car_position_x, y, theta 车辆当前的位置
    # 返回值：三个list ，分别是路径x,y,theta(弧度制)的集合
    #        三个None 因为不存在合法的结果
    # 修改记录：
    # ==================================================================
    def __move_dir_back(self, car_position_x, car_position_y, car_position_theta):
        while (car_position_theta < 0):
            car_position_theta += math.pi * 2
        while (car_position_theta > math.pi * 2):
            car_position_theta -= math.pi * 2

        if car_position_theta > math.pi / 2 and car_position_theta < math.pi * 3 / 2:
            if self.log:
                print("调用此函数前需注意，此函数只考虑了角度小于九十度(试图改变成右侧都可行)的情况!!!!!!!!!!!!!!!!!!!")
                print(car_position_x, car_position_y, car_position_theta, ' -- ', end=' ')
                print("退出 move_left_dir 函数")
            return None, None, None

        # 在下方的圆心的坐标
        o_down_x = car_position_x + math.cos(math.pi / 2 - car_position_theta) * self.env.min_turning_radiu
        o_down_y = car_position_y - math.sin(math.pi / 2 - car_position_theta) * self.env.min_turning_radiu


        # 计算距离
        distance_Odown_ParkingRightFront = self.__distance(o_down_x, o_down_y,
                            self.parking_right_front_point_x, self.parking_right_front_point_y)



        # 判断是否需要倒退
        if (distance_Odown_ParkingRightFront <= self.env.min_turning_radiu - self.env.car_w / 2):
            return car_position_x, car_position_y, car_position_theta

        #计算圆心移动轨迹与车位右前角垂线的交点
        if car_position_theta != 0:
            ko = math.tan(car_position_theta)
            bo = o_down_y - o_down_x * ko
            kp = -1 / ko
            bp = self.parking_right_front_point_y - kp * self.parking_right_front_point_x
            x_meet = (bp - bo) / (ko - kp)
            y_meet = kp * x_meet + bp
        else:
            y_meet = o_down_y
            x_meet = self.parking_right_front_point_x



        #计算 圆心移动轨迹与车位右前角垂线的交点 到 车辆当前圆心 的距离
        dis_Odown_Meet = self.__distance(x_meet, y_meet, o_down_x, o_down_y)

        #计算 圆心移动轨迹与车位右前角垂线的交点 到 车位右前角 的距离
        dis_ParkingRightFront_Meet = self.__distance(x_meet, y_meet, self.parking_right_front_point_x, self.parking_right_front_point_y)




        #如果车辆需要前进才会达到右转后轴撞到圆心的状态，那么直接返回车辆的当前位置，因为不需要倒车
        if (x_meet - o_down_x >= 0):
            return [car_position_x, car_position_y, car_position_theta]

        # 如果 圆心移动轨迹与车位右前角垂线的交点 到 车位右前角 的距离 大于车辆能转的半径，那么车辆怎么倒车都会撞
        if dis_ParkingRightFront_Meet > self.env.min_turning_radiu - self.env.car_w / 2:
            if self.log:
                print("【ERROR】车辆已经撞在了车位角上 , 车辆圆心轨迹到车位角的距离是 ", dis_ParkingRightFront_Meet, " 然而车辆最大距离是 ", self.env.min_turning_radiu
                      - self.env.car_w / 2)
            return None, None, None


        move_dis = dis_Odown_Meet - math.sqrt(math.pow(self.env.min_turning_radiu - self.env.car_w / 2, 2) - math.pow(dis_ParkingRightFront_Meet, 2))



        return car_position_x - move_dis * math.cos(car_position_theta), car_position_y - move_dis * math.sin(car_position_theta), car_position_theta


    # ==================================================================
    # 函数名：__move_right
    # 作者：孙耀威
    # 日期：2021.6.11
    # 功能：计算车辆方向盘右打死能倒车倒到的位置
    # 输入参数：car_position_x, car_position_y, car_position_theta 车辆当前的位置
    # 返回值：三个list ，分别是路径x,y,theta(弧度制)的集合
    #        三个None 因为不存在合法的结果
    # 修改记录：
    # ==================================================================
    def __move_right_back(self, car_position_x, car_position_y, car_position_theta):
        while (car_position_theta < 0):
            car_position_theta += math.pi * 2
        while (car_position_theta > math.pi * 2):
            car_position_theta -= math.pi * 2

        if car_position_theta > math.pi / 2 and car_position_theta < math.pi * 3 / 2:
            if self.log:
                print("调用此函数前需注意，此函数只考虑了角度小于九十度的情况!!!!!!!!!!!!!!!!!!!!!!")
                print("退出 move_left 函数")
            return None, None, None

        # 在下方的圆心的坐标
        o_down_x = car_position_x + math.cos(math.pi / 2 - car_position_theta) * self.env.min_turning_radiu
        o_down_y = car_position_y - math.sin(math.pi / 2 - car_position_theta) * self.env.min_turning_radiu

        # c=(self.r_add_front)
        # a=math.acos((self.env.road_w - o_down_y))
        # print(math.acos((self.env.road_w - o_down_y) / (self.r_add_front))," equal to  "
        #       ,math.atan2(math.sqrt(math.pow(c,2)-math.pow(a,2)),a))

        # 判断车辆左前角是否会撞上车道
        if o_down_y + self.r_add_front > self.env.road_w:
            acos_theta4=self.__calculate_acos_by_atan2((self.env.road_w - o_down_y) , (self.r_add_front))
            theta_rr = math.atan((self.env.car_l - self.env.hou_xuan) / (self.env.min_turning_radiu + self.env.car_w / 2)) - acos_theta4
            if car_position_theta <= theta_rr:
                p_max_theta_r = theta_rr
                return o_down_x - self.env.min_turning_radiu * math.sin(p_max_theta_r), o_down_y + self.env.min_turning_radiu * math.cos(
                    p_max_theta_r), p_max_theta_r

        """ and ( self.env.min_turning_radiu-self.env.car_w/2>
                      self.__distance(o_down_x,o_down_y,self.parking_right_front_point_x,self.parking_right_front_point_y))"""
        # 判断是否能直接入库
        if o_down_x - self.r_add_rear > self.parking_left_front_point_x:

            return o_down_x - self.env.min_turning_radiu, o_down_y, math.pi / 2

        # # 计算车辆能转过的角度
        # cos_theta1 = (self.env.min_turning_radiu + self.env.car_w / 2) / self.r_add_rear
        # cos_theta2 = (o_down_x - self.parking_left_front_point_x) / self.r_add_rear
        # p_max_theta_r = math.acos(cos_theta2) + math.acos(cos_theta1)

        acos_theta2=self.__calculate_acos_by_atan2((o_down_x - self.parking_left_front_point_x),self.r_add_rear)
        acos_theta1 =self.__calculate_acos_by_atan2(self.env.min_turning_radiu + self.env.car_w / 2, self.r_add_rear)

        # print(acos_theta2,)

        p_max_theta_r = acos_theta2 + acos_theta1



        return o_down_x - self.env.min_turning_radiu * math.cos(p_max_theta_r), o_down_y + self.env.min_turning_radiu * math.sin(
            p_max_theta_r), math.pi / 2 - p_max_theta_r

    # ==================================================================
    # 函数名：__move_left_up
    # 作者：孙耀威
    # 日期：2021.6.11
    # 功能：计算车辆方向盘左打死能前进到的位置
    # 输入参数：car_position_x, car_position_y, car_position_theta 车辆当前的位置
    # 返回值：三个list ，分别是路径x,y,theta(弧度制)的集合
    #        三个None 因为不存在合法的结果
    # 修改记录：
    # ==================================================================
    def __move_left_up(self, car_position_x, car_position_y, car_position_theta):

        while(car_position_theta < 0):
            car_position_theta+= math.pi * 2
        while (car_position_theta > math.pi * 2):
            car_position_theta -= math.pi * 2

        if car_position_theta>math.pi/2:
            if self.log:
                print("调用此函数前需注意，此函数只考虑了角度小于九十度的情况!!!!!!!!!!!!!!!!!!!")
                print("退出 move_up 函数")
            return None,None,None



        #寻找在上方的圆心的坐标
        o_up_x = car_position_x - self.env.min_turning_radiu * math.cos(math.pi / 2 - car_position_theta)
        o_up_y = car_position_y + self.env.min_turning_radiu * math.sin(math.pi / 2 - car_position_theta)



        #判断是否能直接入库
        if o_up_x+self.env.min_turning_radiu>self.parking_left_front_point_x+self.env.car_w/2 and \
                o_up_x+self.env.min_turning_radiu<self.parking_right_front_point_x-self.env.car_w/2:
            if o_up_y+self.env.car_l-self.env.hou_xuan<self.env.road_w:
                return o_up_x + self.env.min_turning_radiu, o_up_y, math.pi / 2


        #如果圆心在车道线以上，碰到车道线的角度计算公式要改变
        if (o_up_y>self.env.road_w):
            # cos_theta1= (self.env.min_turning_radiu - self.env.car_w / 2) / self.r_dec_front
            acos_theta1=self.__calculate_acos_by_atan2((self.env.min_turning_radiu - self.env.car_w / 2) ,self.r_dec_front)
            # print(math.acos(cos_theta1),acos_theta1)
            # sin_theta2=(o_up_y-self.env.road_w)/self.r_dec_front
            asin_theta2=self.__calculate_asin_by_atan2((o_up_y-self.env.road_w),self.r_dec_front)
            p_max_theta_r=math.pi/2-asin_theta2-acos_theta1
        else:
            # cos_theta1=(self.env.road_w-o_up_y)/self.r_dec_front
            tan_theta2= (self.env.car_l - self.env.hou_xuan) / (self.env.min_turning_radiu - self.env.car_w / 2)
            acos_theta1 = self.__calculate_acos_by_atan2((self.env.road_w-o_up_y),
                                                         self.r_dec_front)
            p_max_theta_r=math.pi-acos_theta1-math.atan(tan_theta2)



        #验算结果是否会撞上车库右边
        check_x= o_up_x + self.env.min_turning_radiu * math.sin(p_max_theta_r)
        check_y= o_up_y - self.env.min_turning_radiu * math.cos(p_max_theta_r)
        check_theta_r=p_max_theta_r

        check_x_edge= check_x + self.env.car_w / 2 * math.sin(check_theta_r)
        check_y_edge= check_y - self.env.car_w / 2 * math.cos(check_theta_r)

        check_k=math.tan(check_theta_r)
        check_b=check_y_edge-check_k*check_x_edge

        check_x_click=-check_b/check_k



        if check_x_click>self.parking_right_front_point_x:
            l=car_position_theta
            r=p_max_theta_r

            #通过二分法计算车辆转过多少角度刚好使 车位右前角 在 车辆右侧边缘 的延长线上
            while (r-l)>0.001:
                mid=(l+r)/2

                #计算车辆右边缘的点的坐标
                x_edge= o_up_x + (self.env.min_turning_radiu + self.env.car_w / 2) * math.sin(mid)
                y_edge= o_up_y - (self.env.min_turning_radiu + self.env.car_w / 2) * math.cos(mid)

                #计算车辆右边缘的直线表达式
                k_edge=math.tan(mid)
                b_edge=y_edge-k_edge*x_edge

                #令y=0，求直线与右边缘的交点
                edge_clik=-b_edge/k_edge

                if edge_clik > self.parking_right_front_point_x:
                    r = mid
                if edge_clik < self.parking_right_front_point_x:
                    l = mid
                if edge_clik == self.parking_right_front_point_x:
                    l = mid
                    break


            p_max_theta_r=l

        return o_up_x + self.env.min_turning_radiu * math.sin(p_max_theta_r), o_up_y - self.env.min_turning_radiu * math.cos(p_max_theta_r), p_max_theta_r

    # ==================================================================
    # 函数名：__add_dir_back_route
    # 作者：孙耀威
    # 日期：2021.6.11
    # 功能：通过__move_dir_back计算能移动到的位置，然后用__add_dir_route获取路径并返回
    # 输入参数：car_position_x, car_position_y, car_position_theta 车辆当前的位置
    #           route_x, route_y, route_theta 车辆之前的路径
    # 返回值：三个list ，分别是路径x,y,theta(弧度制)的集合
    #        三个None 因为不存在合法的结果
    # 修改记录：
    # ==================================================================
    def __add_dir_back_route(self, car_position_x, car_position_y, car_position_theta, route_x, route_y, route_theta):
        # 通过直线移动获得可以转进库位的位置
        new_position_x, new_position_y, new_theta_r = self.__move_dir_back(car_position_x, car_position_y,
                                                                           car_position_theta)
        # print(new_position_x, new_position_y, new_theta_r)
        #print(new_position_y)
        if new_position_y == None:
            return None, None, None,route_x, route_y, route_theta

        if not ((new_position_x == car_position_x) and (new_position_y == car_position_y)):
            # 添加路径
            rx, ry, rtheta = self.__add_dir_route(car_position_x, car_position_y, new_position_x, new_position_y,
                                                  car_position_theta)
            route_x = route_x + rx
            route_y = route_y + ry
            route_theta = route_theta + rtheta

            if self.log:
                print('dir back', new_position_x, new_position_y, new_theta_r)

        return new_position_x, new_position_y, new_theta_r, route_x, route_y, route_theta

    # ==================================================================
    # 函数名：__add_right_back_route
    # 作者：孙耀威
    # 日期：2021.6.11
    # 功能：通过__move_right_back计算能移动到的位置，然后用__add_arc_route获取路径并返回
    # 输入参数：car_position_x, car_position_y, car_position_theta 车辆当前的位置
    #           route_x, route_y, route_theta 车辆之前的路径
    # 返回值：三个list ，分别是路径x,y,theta(弧度制)的集合
    #        三个None 因为不存在合法的结果
    # 修改记录：
    # ==================================================================
    def __add_right_back_route(self, car_position_x, car_position_y, car_position_theta, route_x, route_y, route_theta):


        # 在下方的圆心的坐标
        o_down_x = car_position_x + math.cos(math.pi / 2 - car_position_theta) * self.env.min_turning_radiu
        o_down_y = car_position_y - math.sin(math.pi / 2 - car_position_theta) * self.env.min_turning_radiu

        new_position_x, new_position_y, new_theta_r = self.__move_right_back(car_position_x, car_position_y,
                                                                             car_position_theta)

        if self.log:
            print('right back', new_position_x, new_position_y, new_theta_r)

        if new_position_y == None:
            return None, None, None, route_x, route_y, route_theta

        # 添加路径
        rx, ry, rtheta = self.__add_arc_route(o_down_x, o_down_y, self.env.min_turning_radiu,
                                              car_position_theta + math.pi / 2,
                                              new_theta_r + math.pi / 2, car_position_theta, new_theta_r)
        route_x = route_x + rx
        route_y = route_y + ry
        route_theta = route_theta + rtheta

        return new_position_x, new_position_y, new_theta_r, route_x, route_y, route_theta

    # ==================================================================
    # 函数名：__add_left_up_route
    # 作者：孙耀威
    # 日期：2021.6.11
    # 功能：通过__move_left_up计算能移动到的位置，然后用__add_arc_route获取路径并返回
    # 输入参数：car_position_x, car_position_y, car_position_theta 车辆当前的位置
    #           route_x, route_y, route_theta 车辆之前的路径
    # 返回值：三个list ，分别是路径x,y,theta(弧度制)的集合
    #        三个None 因为不存在合法的结果
    # 修改记录：
    # ==================================================================
    def __add_left_up_route(self, car_position_x, car_position_y, car_position_theta, route_x, route_y,
                            route_theta):

        # 现在车辆在靠近车位左侧，接着向上转弯
        new_position_x, new_position_y, new_theta_r = self.__move_left_up(car_position_x, car_position_y,
                                                                          car_position_theta)

        if self.log:
            print('up', new_position_x, new_position_y, new_theta_r)

        if new_position_y == None:
            return None, None, None,route_x, route_y, route_theta

        # 寻找在上方的圆心的坐标
        o_up_x = car_position_x - self.env.min_turning_radiu * math.cos(math.pi / 2 - car_position_theta)
        o_up_y = car_position_y + self.env.min_turning_radiu * math.sin(math.pi / 2 - car_position_theta)

        # 添加路径
        rx, ry, rtheta = self.__add_arc_route(o_up_x, o_up_y, self.env.min_turning_radiu,
                                              car_position_theta - math.pi / 2,
                                              new_theta_r - math.pi / 2, car_position_theta, new_theta_r)
        route_x = route_x + rx
        route_y = route_y + ry
        route_theta = route_theta + rtheta

        return new_position_x, new_position_y, new_theta_r, route_x, route_y, route_theta


    def draw_cz(self):
        plt.figure(dpi=70, figsize=(self.env.road_l, self.env.road_w + self.env.parking_l))
        x_major_locator = MultipleLocator(1)
        y_major_locator = MultipleLocator(1)
        ax = plt.gca()
        # ax为两条坐标轴的实例
        ax.xaxis.set_major_locator(x_major_locator)
        # 把x轴的主刻度设置为1的倍数
        ax.yaxis.set_major_locator(y_major_locator)
        plt.plot([0, self.env.parking_w * 3], [self.env.road_w, self.env.road_w], color='black')
        plt.plot([0, self.env.parking_w], [0, 0], color='black')
        plt.plot([2 * self.env.parking_w, self.env.parking_w * 3], [0, 0], color='black')
        plt.plot([self.env.parking_w, self.env.parking_w], [0, -self.env.parking_l], color='black')
        plt.plot([2 * self.env.parking_w, 2 * self.env.parking_w], [0, -self.env.parking_l], color='black')

    # ==================================================================
    # 函数名：__planning_move_right_back_first
    # 作者：孙耀威
    # 日期：2021.6.11
    # 功能：车辆以[直线倒车，方向盘右打死倒车，方向盘左打死前进]的顺序循环规划入库路线
    # 输入参数：car_position_x, car_position_y, car_position_theta 车辆当前的位置
    # 返回值：三个list ，分别是路径x,y,theta(弧度制)的集合
    #        三个None 因为不存在合法的结果
    # 修改记录：
    # ==================================================================
    def __planning_move_right_back_first(self, car_position_x, car_position_y, car_position_theta):

        route_x = []
        route_y = []
        route_theta = []

        #前进的方向信息
        dir_info=[]

        switch_times=0

        now_position_x = car_position_x
        now_position_y = car_position_y
        now_position_theta = car_position_theta
        # self.env.show()
        # print('sim car position ',car_position_x, car_position_y, car_position_theta*180/math.pi)
        # print('sim parking ',self.parking_left_front_point_x,self.parking_left_front_point_y,self.parking_right_front_point_x,self.parking_right_front_point_y)

        while 1 :
            # print("switch_times1 ",switch_times,' max ',self.max_switch_times,' pose ',now_position_x, now_position_y, now_position_theta)
            if self.max_switch_times < switch_times:
                if self.log:
                    print("迭代次数过多，强制退出!")
                return None, None, None, None
            if now_position_x==None:
                return None, None, None, None

            ox,oy,ot=now_position_x, now_position_y, now_position_theta

            now_position_x, now_position_y, now_position_theta, route_x, route_y,route_theta=self.__add_dir_back_route(now_position_x, now_position_y, now_position_theta, route_x, route_y,route_theta)

            # print('直线倒车至',now_position_x, now_position_y, now_position_theta)


            for i in range(len(route_x)-len(dir_info)):
                dir_info.append(1)



            # print("switch_times2 ",switch_times,' max ',self.max_switch_times,' pose ',now_position_x, now_position_y, now_position_theta)

            if now_position_x==None:
                return None, None, None, None
            ox, oy, ot = now_position_x, now_position_y, now_position_theta
            now_position_x, now_position_y, now_position_theta, route_x, route_y, route_theta = self.__add_right_back_route( now_position_x, now_position_y, now_position_theta, route_x, route_y, route_theta)

            # print('右后移动',now_position_x, now_position_y, now_position_theta)


            for i in range(len(route_x)-len(dir_info)):
                dir_info.append(1)


            if (now_position_theta == math.pi / 2):
                break
            if now_position_x==None:
                return None, None, None, None


            # print("switch_times3 ",switch_times,' max ',self.max_switch_times,' pose ',now_position_x, now_position_y, now_position_theta)


            ox, oy, ot = now_position_x, now_position_y, now_position_theta
            now_position_x, now_position_y, now_position_theta, route_x, route_y, route_theta = self.__add_left_up_route(
                now_position_x, now_position_y, now_position_theta, route_x, route_y, route_theta)

            # print('左前移动',now_position_x, now_position_y, now_position_theta)

            for i in range(len(route_x)-len(dir_info)):
                dir_info.append(0)



            # print("switch_times4 ",switch_times,' max ',self.max_switch_times,' pose ',now_position_x, now_position_y, now_position_theta)
            if (now_position_theta == math.pi / 2):
                break
            switch_times += 1


        return route_x, route_y, route_theta,dir_info

    # ==================================================================
    # 函数名：__planning_move_left_front_first
    # 作者：孙耀威
    # 日期：2021.6.11
    # 功能：车辆以[方向盘左打死前进,直线倒车，方向盘右打死倒车]的顺序循环规划入库路线
    # 输入参数：car_position_x, car_position_y, car_position_theta 车辆当前的位置
    # 返回值：三个list ，分别是路径x,y,theta(弧度制)的集合
    #        三个None 因为不存在合法的结果
    # 修改记录：
    # ==================================================================
    def __planning_move_left_front_first(self, car_position_x, car_position_y, car_position_theta):
        # 路程初始化
        route_x = []
        route_y = []
        route_theta = []

        #前进的方向信息
        #1 倒车 0 前进
        dir_info=[]

        now_position_x = car_position_x
        now_position_y = car_position_y
        now_position_theta = car_position_theta

        now_position_x, now_position_y, now_position_theta, route_x, route_y, route_theta = self.__add_left_up_route(
            now_position_x, now_position_y, now_position_theta, route_x, route_y, route_theta)

        for i in range(len(route_x) - len(dir_info)):
            dir_info.append(0)

        # 右后方移动的路线
        rx, ry, rtheta,tmp_dir_info = self.__planning_move_right_back_first(now_position_x, now_position_y, now_position_theta)
        if rx == None:
            return None,None,None

        route_x = route_x + rx
        route_y = route_y + ry
        route_theta = route_theta + rtheta
        dir_info = dir_info+tmp_dir_info
        return route_x, route_y, route_theta,dir_info


    # ==================================================================
    # 函数名：planning
    # 作者：孙耀威
    # 日期：2021.6.11
    # 功能：车辆通过比较左前和右后两条路线，选择一条更优路线作为泊车路径
    # 输入参数：car_position_x, car_position_y, car_position_theta 车辆当前的位置
    # 返回值：三个list ，分别是路径x,y,theta(弧度制)的集合
    #        三个None 因为不存在合法的结果
    # 修改记录：
    # ==================================================================
    def planning(self, car_position_x, car_position_y, car_position_theta):

        print('arc car position ',car_position_x, car_position_y, car_position_theta)
        print('arc parking ',   self.parking_left_front_point_x , self.env.parking_left_front_point_y,self.env.parking_right_front_point_x,self.env.parking_right_front_point_y)

        # 判断车辆是否能够直接入库
        if car_position_x >= self.parking_left_front_point_x + self.env.car_w / 2 and \
                car_position_x <= self.parking_right_front_point_x - self.env.car_w / 2 \
                and car_position_theta == math.pi / 2:

            route_x, route_y, route_theta = self.__into_parking(car_position_x, car_position_y, car_position_theta)

            dir_info=[]
            for i in range(len(route_x) - len(dir_info)):
                dir_info.append(1)

        else:
            # 如果车辆无法一次性入库，那么就进入来回腾挪入库模式

            # 车先向右后方移动的路线
            print('right')
            route_right_back_x, route_right_back_y, route_right_back_theta,route_right_back_dir_info = self.__planning_move_right_back_first(
                car_position_x, car_position_y, car_position_theta)

            # print("right back")
            # print(route_right_back_x)
            # print("============================================================================")
            # print("left front")


            # 车先向左前方方移动的路线
            print('left')
            route_left_front_x, route_left_front_y, route_left_front_theta,route_left_front_dir_info = self.__planning_move_left_front_first(car_position_x, car_position_y, car_position_theta)

            # print(route_left_front_x)

            # 比较路径，选择最优的路径
            route_x, route_y, route_theta = self.__compare_route(route_right_back_x, route_right_back_y, route_right_back_theta, route_left_front_x, route_left_front_y, route_left_front_theta)

            if route_x==route_left_front_x:
                dir_info=route_left_front_dir_info
            else:
                dir_info = route_right_back_dir_info

            if route_x == None:
                if self.log:
                    print('规划失败,当前位置不存在简单入库的路径')
                return route_x, route_y, route_theta,None



            end_position_x = route_x[-1]
            end_position_y = route_y[-1]
            end_position_theta = route_theta[-1]

            if end_position_y + self.env.car_l - self.env.hou_xuan >= 0:
                rx, ry, rtheta = self.__into_parking(end_position_x, end_position_y, end_position_theta)

                route_x += rx
                route_y += ry
                route_theta += rtheta


            for i in range(len(route_x) - len(dir_info)):
                dir_info.append(1)

        return route_x, route_y, route_theta,dir_info

if __name__ == "__main__":
    start_time=time.time()
    env0=Env()
    parking_l = 5.4
    road_w = 5  # 2.86*3 #尽量在4.09以上，小于的话腾挪次数要爆炸
    car_l = 4
    car_w = 1.4298046875000026
    min_turning_radiu = 9.6
    wheel_dis = 2.5064953996428505
    road_l = 10
    step = 0.1
    hou_xuan = 1

    # parking_w = 2.12
    # parking_left_point = [parking_w, 0]
    # parking_right_point = [2 * parking_w, 0]

    #车辆实际坐标
    # car_x=-37.131065
    # car_y=-14.28638
    # car_theta_r=-1.356844




    #7.373004391908768 2.6422429420110305 36.45990889764952
    #2.5 0 5.0 0

    # # 车位实际坐标
    # real_parking_left_head = [-42.3, -3.1]
    # real_parking_right_head = [-42.3, -6.1]

    # 车位实际坐标
    real_parking_left_head = [2.5, 0]
    real_parking_right_head = [5,0]

    # 计算车位长度
    parking_w = math.sqrt(math.pow(real_parking_left_head[0] - real_parking_right_head[0], 2) + math.pow(
        real_parking_left_head[1] - real_parking_right_head[1], 2))
    parking_left_point=real_parking_left_head
    parking_right_point=real_parking_right_head

    env0.set_car_info(car_l, car_w, min_turning_radiu, wheel_dis, hou_xuan, step)
    env0.set_env_info(parking_l,parking_w,road_w,road_l,parking_left_point,parking_right_point)



    vp=VerticalParking(env0)

    # car_position_x =8
    # car_position_y =2.5
    # car_position_theta =0.12217304763960307


    # car_position_x =8
    # car_position_y =2.5
    # car_position_theta =0.12217304763960307

    car_position_x =7.373004391908768
    car_position_y =2.6422429420110305
    car_position_theta=36.45990889764952*math.pi/180


    rx,ry,rtheta,dir_info=vp.planning(car_position_x,car_position_y,car_position_theta)

    # print(rx,ry)
    plt.plot(rx,ry)
    plt.show()

    end_time=time.time()
    time_cost=(end_time-start_time)*1000
    time_cost_ms = time_cost % 1000
    time_cost=(time_cost-time_cost_ms)/1000
    print("本次运行用时 ",int(time_cost)," s ",int(time_cost_ms)," ms ")













