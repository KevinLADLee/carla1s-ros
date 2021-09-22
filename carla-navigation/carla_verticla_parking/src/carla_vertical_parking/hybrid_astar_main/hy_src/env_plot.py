import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import imageio
import platform
import shutil

from matplotlib.pyplot import MultipleLocator
from math import cos, sin, pi
from pathlib import Path

class env_plot:
    def __init__(self, width=10, height=10, robot_num = 1, obs_num=0, robot_list=[], obs_cir_list=[], car_list=[], resolution=1, full=False, keep_path=False, map_matrix=None):
    
        self.fig, self.ax = plt.subplots()
        
        self.width = width
        self.height = height
        self.res = resolution
        
        self.robot_num = robot_num
        self.obs_num = obs_num

        self.keep_path=keep_path

        self.robot_list = robot_list
        self.obs_cir_list = obs_cir_list
        self.car_list=car_list
        self.color_list = ['g', 'b', 'r', 'c', 'm', 'y', 'k', 'w']

        self.map_matrix = map_matrix

        self.init_plot()

        if full:
            mode = platform.system()
            if mode == 'Linux':
                mng = plt.get_current_fig_manager()
                mng.resize(*mng.window.maxsize())

            elif mode == 'Windows':
                figManager = plt.get_current_fig_manager()
                figManager.window.showMaximized()

    # draw ax
    def init_plot(self, **kwargs):
        self.ax.set_aspect('equal')
        self.ax.set_xlim(-1, self.width)
        self.ax.set_ylim(-1, self.height)
        # self.ax.legend(loc='upper right')
        self.ax.set_xlabel("x [m]")
        self.ax.set_ylabel("y [m]")

        self.draw_robot_diff_list()
        self.draw_obs_cir_list()
        self.draw_car_list(**kwargs)

        if self.map_matrix is not None:
            # self.ax.imshow(np.flipud(self.map_matrix.T), cmap='Greys', origin='lower', extent=[0,self.width,0,self.height])
            self.ax.imshow(self.map_matrix.T, cmap='Greys', origin='lower', extent=[0,self.width,0,self.height])
        
        return self.ax.patches + self.ax.texts + self.ax.artists

    # draw components
    def draw_robot_diff(self, robot, robot_color = 'g', goal_color='r'):
        
        x = robot.state[0][0]
        y = robot.state[1][0]
        theta = robot.state[2][0]
        
        goal_x = robot.goal[0, 0]
        goal_y = robot.goal[1, 0]

        robot_circle = mpl.patches.Circle(xy=(x, y), radius = robot.radius, color = robot_color)
        goal_circle = mpl.patches.Circle(xy=(goal_x, goal_y), radius = 1.5*robot.radius, color=goal_color, alpha=0.5)

        self.ax.add_patch(goal_circle)
        self.ax.text(goal_x + 0.3, goal_y, 'g'+ str(robot.id), fontsize = 12, color = 'k')

        self.ax.add_patch(robot_circle)
        self.ax.arrow(x, y, 0.5*cos(theta), 0.5*sin(theta), head_width = 0.2)
        self.ax.text(x - 0.5, y, 'r'+ str(robot.id), fontsize = 10, color = 'k')

    def draw_car(self, car, car_color='g', goal_color='c', goal_l=2, text=False, line_length=0.3, pre_state=False):

        x = car.angular_position[0, 0]
        y = car.angular_position[1, 0]
        r_phi=car.state[2, 0] - pi/2
        r_phi_ang = 180*r_phi/pi

        line_rad_f = car.state[3, 0] + car.state[2, 0]
        line_rad_b = car.state[2, 0]

        gx = car.goal[0, 0]
        gy = car.goal[1, 0]
        gdx = goal_l*cos(car.goal[2, 0])
        gdy = goal_l*sin(car.goal[2, 0])

        for i in range(4):

            if 0 < i < 3:
                wx = car.wheel_position[0, i]
                wy = car.wheel_position[1, i]

                lx0 = wx + line_length * cos(line_rad_f) / 2
                ly0 = wy + line_length * sin(line_rad_f) / 2

                lx1 = wx - line_length * cos(line_rad_f) / 2
                ly1 = wy - line_length * sin(line_rad_f) / 2
                
                self.ax.plot([lx0, lx1], [ly0, ly1], 'k-')

            else:
                wx = car.wheel_position[0, i]
                wy = car.wheel_position[1, i]

                lx0 = wx + line_length * cos(line_rad_b) / 2
                ly0 = wy + line_length * sin(line_rad_b) / 2

                lx1 = wx - line_length * cos(line_rad_b) / 2
                ly1 = wy - line_length * sin(line_rad_b) / 2
                
                self.ax.plot([lx0, lx1], [ly0, ly1], 'k-')

        car_rect = mpl.patches.Rectangle(xy=(x, y), width=car.width, height=car.length, angle=r_phi_ang, edgecolor=car_color, fill=False)
        goal_arrow = mpl.patches.Arrow(x=gx, y=gy, dx=gdx, dy=gdy, color=goal_color)

        self.ax.add_patch(car_rect)
        self.ax.add_patch(goal_arrow)

        if text:
            self.ax.text(x - 0.5, y, 'c'+ str(car.id), fontsize = 10, color = 'k')
            self.ax.text(car.goal[0, 0] + 0.3, car.goal[1, 0], 'cg'+ str(car.id), fontsize = 12, color = 'k')

        if pre_state:
            self.point_arrow_plot(car.pre_state)

    def draw_robot_diff_list(self, **kwargs):

        for robot in self.robot_list:    
            self.draw_robot_diff(robot, **kwargs)

    def draw_obs_cir_list(self):

        for obs_cir in self.obs_cir_list:
            x = obs_cir.pos[0, 0]
            y = obs_cir.pos[1, 0]

            obs_circle = mpl.patches.Circle(xy=(x, y), radius = obs_cir.radius, color = 'k')
            self.ax.add_patch(obs_circle)
    
    def draw_car_list(self, **kwargs):
        
        if len(self.car_list) > 1:
            for car, color in zip(self.car_list, self.color_list):
                self.draw_car(car, car_color=color, text=True, **kwargs)
        else:
            for car in self.car_list:
                self.draw_car(car, **kwargs)

    def com_cla(self):
        
        self.ax.patches = []
        self.ax.texts=[]
        # self.ax.artists=[]
        self.ax.lines=[]

    # animation method 1
    def animate(self, j):

        self.draw_robot_diff_list()

        return self.ax.patches + self.ax.texts + self.ax.artists

    def show_ani(self):
        ani = animation.FuncAnimation(
        self.fig, self.animate, init_func=self.init_plot, interval=100, blit=True, frames=100, save_count=100)
        plt.show()
    
    def save_ani(self, name='animation'): 
        ani = animation.FuncAnimation(
        self.fig, self.animate, init_func=self.init_plot, interval=1, blit=False, save_count=300)
        ani.save(name+'.gif', writer='pillow')

    # # animation method 2
    def save_gif_figure(self, path, i):

        if path.exists():
            order = str(i).zfill(3)
            plt.savefig(str(path)+'/'+order)
        else:
            path.mkdir()
            order = str(i).zfill(3)
            plt.savefig(str(path)+'/'+order)

    def create_animate(self, image_path, ani_path, ani_name='animated', keep_len=30, rm_fig_path=True):

        if not ani_path.exists():
            ani_path.mkdir()

        images = list(image_path.glob('*.png'))
        images.sort()
        image_list = []
        for i, file_name in enumerate(images):

            if i == 0:
                continue

            image_list.append(imageio.imread(file_name))
            if i == len(images) - 1:
                for j in range(keep_len):
                    image_list.append(imageio.imread(file_name))

        imageio.mimsave(str(ani_path)+'/'+ ani_name+'.gif', image_list)
        print('Create animation successfully')

        if rm_fig_path:
            shutil.rmtree(image_path)
    
    # plot path
    def path_plot(self, path_list, path_color=None, show_point=False):
        
        path_x_list = list(map(lambda x: x[0, 0], path_list))
        path_y_list = list(map(lambda y: y[1, 0], path_list))

        self.ax.plot(path_x_list, path_y_list, color=path_color)

        if show_point:
            for point in path_list:
                self.point_plot(point)
                
    def point_arrow_plot(self, point, length=0.5, width=0.3, color='r'):

        px = point[0, 0]
        py = point[1, 0]
        theta = point[2, 0]

        pdx = length * cos(theta)
        pdy = length * sin(theta)

        point_arrow = mpl.patches.Arrow(x=px, y=py, dx=pdx, dy=pdy, color=color, width=width)

        self.ax.add_patch(point_arrow)
    
    def point_plot(self, point, markersize=2, color="k"):
        
        if isinstance(point, tuple):
            x = point[0]
            y = point[1]
        else:
            x = point[0,0]
            y = point[1,0]
    
        self.ax.plot([x], [y], marker='o', markersize=markersize, color=color)

    # plt 
    def cla(self):
        self.ax.cla()

    def pause(self, time=0.001):
        plt.pause(time)
    
    def show(self):
        plt.show()

    # def draw_start(self, start):
    #     self.ax.plot(start[0, 0], start[1, 0], 'rx')

    # def plot_trajectory(self, robot, num_estimator, label_name = [''], color_line=['b-']):

    #     self.ax.plot(robot.state_storage_x, robot.state_storage_y, 'g-', label='trajectory')

    #     for i in range(num_estimator):
    #         self.ax.plot(robot.estimator_storage_x[i], robot.estimator_storage_y[i], color_line[i], label = label_name[i])

    #     self.ax.legend()

    # def plot_pre_tra(self, pre_traj):
    #     list_x = []
    #     list_y = []

    #     if pre_traj != None:
    #         for pre in pre_traj:
    #             list_x.append(pre[0, 0])
    #             list_y.append(pre[1, 0])
            
    #         self.ax.plot(list_x, list_y, '-b')
    
    # def draw_path(self, path_x, path_y, line='g-'):
    #     self.ax.plot(path_x, path_y, 'g-')



    



