from PIL import Image
from pathlib import Path
import math
from carla_vertical_parking.hybrid_astar_main.hy_src.env_base import env_base
from carla_vertical_parking.hybrid_astar_main.hy_src.hybrid_astar import hybrid_astar
from carla_vertical_parking.hybrid_astar_main.hy_src.grid_graph import grid_graph
from carla_vertical_parking.hybrid_astar_main.space_change import SpaceChange
from pathlib import Path
import numpy as np
import cProfile

from carla_nav_msgs.msg import ParkingSpot
from geometry_msgs.msg import Pose, PoseStamped, Point
import rospy
import tf
from nav_msgs.msg import Path, Odometry
from carla_nav_msgs.msg import Path as PathArray
from matplotlib import pyplot as plt
import os

#这里面的信息都是无法从carla的vertical info中获取的参数
class EnvInfoMessage(float):
    road_w = 5 # 2.86*3 #尽量在4.09以上，小于的话腾挪次数要爆炸
    car_l = 4
    hou_xuan = 1
    step=0.1
    road_l=10


class HybirdAStarPlanning():
    def __init__(self,vehicle_info):
        self.vehicle_info=vehicle_info
        # print(self.vehicle_info)

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


        road_w+=2

        # print(car_position_x,car_position_y,car_position_theta)
        # print(parking_left_point,parking_right_point)
        # print(parking_l,parking_w,'road ',road_w)


        l_limit=2
        sim_x=-99999999
        sim_y=-99999999
        sim_theta=-99999999
        while sim_x<=0:
            l_limit+=1
            simulate_parking_left_point=[parking_w*(l_limit-1),parking_l]
            simulate_parking_right_point=[parking_w*l_limit,parking_l]
            self.space_change = SpaceChange(parking_left_point,parking_right_point
                                            , simulate_parking_left_point, simulate_parking_right_point)
            sim_x,sim_y,sim_theta=self.space_change.real_to_sim(car_position_x,car_position_y,car_position_theta)
            # print('simulate position',sim_x,sim_y,sim_theta)
        r_limit=int((sim_x-parking_w*l_limit)/parking_w)+2
        r_limit=max(r_limit,l_limit+3)

        hy_rate=0.01
        rate=1/hy_rate

        world_height=int((parking_l+road_w))
        world_width=int((l_limit+r_limit)*parking_w)


        # print('shape world ',world_height,world_width)
        config = {'world_height': world_height,
                  'world_width':world_width,
                  'step_time': 0.1,
                  'xy_resolution': hy_rate,
                  'yaw_resolution': 5,
                  'acker_number': 1,
                  'acker_init_state_list': [[sim_x,sim_y,sim_theta, 0]],
                  'acker_goal_list': [[(l_limit+0.5)*parking_w, parking_l/2, math.pi/2, 0]],
                  'acker_shape_list': [[car_l, car_w, wheel_dis, car_w]],
                  'acker_vel_limit': 4,
                  'acker_vel_ang_limit': 5}
        # print('config ',config)
        world_path = config
        world_map =self.get_map(world_height, world_width,l_limit,r_limit,parking_w,parking_l,road_w,rate)



        #get path
        cur_path=os.path.dirname(__file__)
        reeds_lookup_path =cur_path+'/hybrid_astar_main' + '/reeds_lookup.npy'
        env0 = env_base(world_path, world_map, yaml_file=True)
        env0.initialization_robot()
        env0.initialization_world(map_file=True)

        grid = grid_graph(env0.map_matrix, xy_reso=env0.xy_reso, yaw_reso=env0.yaw_reso)
        # print('shape ',grid.grid_matrix.shape)
        hy_astar = hybrid_astar(grid, env0.car.shape, step_size=2, reeds_size=1, min_radius=9, test_plot=env0,
                                lookup_path=reeds_lookup_path)
        path_list = hy_astar.hy_astar_search(env0.car.state, env0.car.goal, show_process=True)


        if len(path_list)>0:

            route_x=[]
            route_y=[]
            route_theta_r=[]
            dir_info=[]
            sim_route_x=[]
            sim_route_y=[]
            for point in path_list:
                # print(point[0],point[1],point[2],point[3])
                tmp_x,tmp_y,tmp_theta=self.space_change.sim_to_real(point[0][0],point[1][0],point[2][0])
                sim_route_y.append(point[1][0])
                sim_route_x.append(point[0][0])
                route_x.append(tmp_x)
                route_y.append(tmp_y)
                route_theta_r.append(tmp_theta)

                dir_info.append(int(point[3][0]))
                # print('TMP ',tmp_x,tmp_y)

            path_array = self.change_list_into_path_array(route_x,route_y,route_theta_r,dir_info)
            # plt.plot(sim_route_x,sim_route_y)
            # plt.show()
            if 0:
                env0.world.path_plot(path_list, path_color='r', show_point=False)
                env0.render(0.1)

                for point in path_list:
                    # print(point,type(point))
                    env0.world.com_cla()
                    env0.car.update_state(point)
                    env0.world.path_plot(path_list, path_color='r', show_point=False)

                    env0.render(0.1)

                env0.show()

        else:
            print('A* planning fail')
            path_array = PathArray()

        # print('l,r ',l_limit,r_limit)
        # plt.plot([0,parking_w*(l_limit+r_limit)],[parking_l+road_w,parking_l+road_w])
        # for i in range(l_limit+r_limit-1):
        #     plt.plot([parking_w*i,parking_w*(i+1)],[parking_l,parking_l])
        # plt.show()

        # print(path_array)
        return path_array

    def get_map(self,world_height, world_width,l_limit,r_limit,parking_w,parking_l,road_w,rate):
        map=np.zeros((int(world_height*rate), int(world_width*rate)))
        low_bound=int(rate*(parking_l+road_w))
        while low_bound/rate>=parking_l+road_w:
            low_bound-=1

        # print('low bound ',low_bound,len(map),len(map[0]))
        # for i in range(low_bound,len(map)):
        #     for j in range(0,len(map[0])):
        #         # print('up bound ',i/rate,j/rate)
        #         map[i][j]=255
        #
        #
        # # parking collision
        # for i in range(0,int(parking_l*rate)):
        #     for j in range(0,int((l_limit)*parking_w*rate)):
        #         map[i][j]=255
        #
        # for i in range(0,int(parking_l*rate)):
        #     for j in range(int((l_limit+1)*parking_w*rate),len(map[0])):
        #         # print(j/rate,i/rate)
        #         map[i][j]=255


        return map.T

    def change_list_into_path_array(self,route_x,route_y,route_theta_r,dir_info):
        path_array = PathArray()
        route=[]
        for i in range(len(route_x)):
            route.append([route_x[i], route_y[i], route_theta_r[i]])

        now_path_dir = dir_info[0]
        now_route = [[route_x[0], route_y[0], route_theta_r[0]]]

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
                now_route = [[route_x[i], route_y[i], route_theta_r[i]]]
                now_path_dir = dir_info[i]

                path_array.paths.append(type_route)
                path_array.driving_direction.append(now_path_dir)
                type_route = Path()
                type_route.poses.append(tmp_pose)

        path_array.paths.append(type_route)

        return path_array


if __name__ == '__main__':

    cur_path = Path(__file__).parent
    save_path = str(cur_path / 'tmp')


    config_path = str(cur_path / 'hy_astar_world.yaml')
    reeds_lookup_path = str(cur_path / 'reeds_lookup.npy')
    world_map = str(cur_path / 'map_image' / 'map_100_100.png')

    astar_env=HybirdAStarPlanning([0, 0, 0])

    # path=astar_env.searching(config_path,reeds_lookup_path,world_map)

