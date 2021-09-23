import yaml
import time
import numpy as np
from ..hy_src.env_plot import env_plot
from ..hy_src.car_robot import car_robot
from ..hy_src.mobile_robot import mobile_robot
from ..hy_src.obs_circle import obs_circle
from PIL import Image
from math import pi

class env_base:

    def __init__(self, world_name = None, world_map=None,yaml_file=False):
        
        if world_name != None:

            if yaml_file == False:
                with open(world_name) as file:
                    com_list = yaml.load(file, Loader=yaml.FullLoader)
                    # print(com_list)
            else:
                com_list = world_name

            # robot
            self.robot_num = com_list.get('robot_number', 0)
            if self.robot_num > 0:
                self.robot_mode = com_list['robot_mode']
                self.robot_radius_list = com_list['robot_radius_list']
                self.init_state_list = com_list['robot_init_state_list']
                self.goal_list = com_list['robot_goal_list']

            # ackermann
            self.acker_num = com_list.get('acker_number', 0)
            if self.acker_num > 0:
                self.acker_init_state_list=com_list['acker_init_state_list']
                self.acker_goal_list=com_list['acker_goal_list']
                self.acker_shape_list=com_list['acker_shape_list']
                self.acker_vl=com_list['acker_vel_limit']
                self.acker_val=com_list['acker_vel_ang_limit']
            # obstacle
            self.num_obs_cir = com_list.get('obstacle_circle_number', 0)
            if self.num_obs_cir > 0:
                self.obs_cir_pos_list = com_list['obstacle_circle_pos_list']
                self.obs_cir_rad_list=com_list['obstacle_circle_radius_list']
                self.obs_line_points = com_list['obstacle_line_number']

            # world
            self.width = com_list['world_width']
            self.height = com_list['world_height']
            self.step_time = com_list['step_time']
            # print('xy_resolution   ',com_list['xy_resolution'],com_list.get('xy_resolution', 1))
            self.xy_reso = com_list.get('xy_resolution', 1)
            self.yaw_reso = com_list.get('yaw_resolution', pi/36)
            self.world_map = world_map
            
        self.robot_list=[]
        self.obs_cir_list=[]
        self.car_list=[]

        # self.world = env_plot(self.width, self.height)

    def initialization(self, **kwargs):
        self.initialization_robot(**kwargs)

        self.initialization_world(**kwargs)

    def initialization_robot(self, **kwargs):

        # robot
        for i in range(self.robot_num):
            robot = mobile_robot(id=i, mode=self.robot_mode, radius=self.robot_radius_list[i],
                                 init_state=self.init_state_list[i], goal=self.goal_list[i], step_time=self.step_time,
                                 **kwargs)
            self.robot_list.append(robot)
            self.robot = robot if i == 0 else None

        # car robot
        for j in range(self.acker_num):
            car = car_robot(id=j, length=self.acker_shape_list[j][0], width=self.acker_shape_list[j][1],
                            wheelbase=self.acker_shape_list[j][2], wheelbase_w=self.acker_shape_list[j][3],
                            init_state=self.acker_init_state_list[j], goal=self.acker_goal_list[j],
                            step_time=self.step_time, vel_limit=self.acker_vl, vel_ang_limit=self.acker_val, **kwargs)
            self.car_list.append(car)
            self.car = car if j == 0 else None

        # obstacle circle 
        for k in range(self.num_obs_cir):
            obs_cir = obs_circle(position=self.obs_cir_pos_list[k], radius=self.obs_cir_rad_list[k], **kwargs)
            self.obs_cir_list.append(obs_cir)


    def initialization_world(self, map_file):

        # world
        if map_file==False:
            if self.world_map != None:
                    img = Image.open(self.world_map).convert('L')
                    px = self.width / self.xy_reso
                    py = self.height / self.xy_reso
                    img = img.resize((int(px), int(py)), Image.NEAREST)
                    map_matrix = np.array(img)
                    map_matrix = 255 - map_matrix
                    map_matrix[map_matrix > 255 / 2] = 255
                    map_matrix[map_matrix < 255 / 2] = 0
                    self.map_matrix = np.fliplr(map_matrix.T)
                    # print('world_map = ',type(self.map_matrix))
                    # print('world_map = [')
                    # for i in self.map_matrix:
                    #     print('[',end='')
                    #     for j in i:
                    #         print(j,' , ',end='')
                    #     print('],')
                    # print(']')
            else:
                self.map_matrix = None
        else:
            self.map_matrix =self.world_map

        # print('ori map ', self.width, self.height)
        # print('xy radio ', self.xy_reso)
        # print('map shape ', len(self.map_matrix), len(self.map_matrix[0]))

        self.world = env_plot(self.width, self.height, robot_list=self.robot_list, obs_cir_list=self.obs_cir_list,
                              car_list=self.car_list, map_matrix=self.map_matrix)
        self.world.init_plot()

    def cal_des_list(self):
        vel_list = list(map(lambda x: x.cal_des_vel() , self.robot_list))
        return vel_list
    
    def cal_des_car_list(self):
        vel_list = list(map(lambda x: x.cal_des_vel() , self.car_list))
        return vel_list

    def step(self, vel_list=[], vel_car_list=[], **vel_kwargs):

        # vel_kwargs: vel_type = 'diff', 
        #             stop=True, whether stop when arrive at the goal
        #             noise=False, 
        #             alpha = [0.01, 0, 0, 0.01, 0, 0], noise for diff
        #             control_std = [0.01, 0.01], noise for omni

        for robot, vel in zip(self.robot_list, vel_list):
            robot.move_forward(vel, **vel_kwargs)

        for car, vel in zip(self.car_list, vel_car_list):
            car.move_forward(vel, **vel_kwargs)
     
    def render(self, time=0.1, **kwargs):
        
        # self.world.com_cla()
        self.world.draw_robot_diff_list()
        self.world.draw_obs_cir_list()
        self.world.draw_car_list(**kwargs)
        
        self.world.pause(time)
    
    def render_cla(self, time=0.1, **kwargs):

        self.world.com_cla()

        self.world.draw_robot_diff_list()
        self.world.draw_obs_cir_list()
        self.world.draw_car_list(**kwargs)
        self.world.pause(time)

    def show(self):
        self.world.show()
    
    def show_ani(self):
        self.world.show_ani()
    
    def save_ani(self):
        self.world.save_ani()

    def step_time_adjust(self, start_time):
        time_el = 0
        while time_el < self.step_time:
            time.sleep(0.01)
            end = time.time()
            time_el = end - start_time
    
    def arrive_all(self):

        for robot in self.robot_list:
            if not robot.arrive():
                return False

        for car in self.car_list:
            if not car.arrive():
                return False
        
        return True



        
        
            
    
        

        

