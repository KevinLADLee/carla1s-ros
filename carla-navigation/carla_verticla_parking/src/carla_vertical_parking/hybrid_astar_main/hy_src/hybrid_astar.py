import heapdict
import numpy as np
import logging

from ..hy_src.motion_model import motion_acker_step
from math import pi
from collections import namedtuple
from ..hy_src.reeds_shepp import reeds_shepp
from ..hy_src.astar import astar

from math import sin, cos
from tqdm import tqdm
from transforms3d.euler import euler2mat
from time import time

class hybrid_astar:
    def __init__(self, grid_graph, car_shape=[4.6, 1.6, 3, 1.6], step_size=1, reeds_size=1, min_radius=3, test_plot=None, lookup_path=None):
        
        self.grid = grid_graph
        
        self.car_shape = car_shape  # [length, width, wheelbase, wheelbase_w]
        self.min_radius = min_radius
        self.reeds_size = reeds_size

        self.step_size = step_size

        self.steer_list = [-1, 0, 1]
        self.gear_list = [-1, 1]

        self.node_tuple = namedtuple('node', 'point, cost, parent')  # point: x, y theta, gear(-1, 1)
        
        self.lookup_path = lookup_path
        
        self.rs = reeds_shepp(self.min_radius)
        self.astar = astar(grid_graph, car_shape[1] * 0.5)
        
        self.test_plot = test_plot

    def hy_astar_search(self, start_point=np.zeros((4, 1)), goal_point=np.zeros((4, 1)), show_process=False):
        
        print('hybrid a star start')
        start_time = time()
        reeds_lookup_matrix = np.load(self.lookup_path)

        start_node = self.node_tuple(start_point, 0, None)
        goal_node = self.node_tuple(goal_point, 0, None) 
        
        # open and close set
        open_set = heapdict.heapdict()
        open_list = [start_node]
        open_set[0] = 0

        start_index = self.grid.point3index(start_node.point)
        open_cell_index=[start_index] # list of the grid cell index

        if start_index[0] > self.grid.len_x or start_index[1] > self.grid.wid_y:
            logging.error("The start position overstep the boundary")
            return []

        close_cell_index=[]

        g_cost = np.zeros((self.grid.len_x, self.grid.wid_y, self.grid.yaw_z)) # cost so far
        
        final_path = []
        iter = 0

        while len(open_set.keys()) != 0:

            list_index = open_set.popitem()[0]
            current_node = open_list[list_index]
            current_index = self.grid.point3index(current_node.point)
            # test plot
            if show_process:
                
                x = current_index[0]*self.grid.xy_reso
                y = current_index[1]*self.grid.xy_reso
                self.test_plot.world.point_plot((x,y))
            # self.test_plot.world.pause(0.00001)

            # reeds_shepp expansions
            if iter % 10 == 0:
                arrive, final_path = self.arrive_reeds(current_node, goal_node)
                if arrive:
                    # print(iter)
                    end_time = time()
                    print('time', end_time - start_time)
                    return final_path

            # child node expansions
            child_node_list = self.child_node(current_node)

            for next_node in child_node_list:

                cell_index = self.grid.point3index(next_node.point)
                
                if cell_index in close_cell_index:
                    continue

                rectangle = self.angular_pos(next_node.point)
                rect_collision = self.grid.check_collision_rectangle(rectangle)

                if rect_collision:
                    continue

                if cell_index not in open_cell_index:
                    g_cost[cell_index] = next_node.cost
                    priority = next_node.cost + self.heuristic(next_node.point, goal_node.point, reeds_lookup_matrix)
                    open_cell_index.append(cell_index)
                    open_list.append(next_node)
                    open_set[open_cell_index.index(cell_index)] = priority
                
                elif next_node.cost < g_cost[cell_index]:
                    g_cost[cell_index] = next_node.cost
                    priority = next_node.cost + self.heuristic(next_node.point, goal_node.point, reeds_lookup_matrix)
                    open_set[open_cell_index.index(cell_index)] = priority
                # close_cell_index.append(cell_index)

            close_cell_index.append(current_index)
            iter = iter + 1
    
        return final_path

    def arrive_reeds(self, cur_node, goal_node):
        
        astar_path = []
        
        reeds_path = self.rs.shortest_path(cur_node.point, goal_node.point, self.reeds_size, include_gear=True)
        arrive = not self.path_collsion_check(reeds_path)
         
        if arrive:
            print('reedsshepp path arrive')
            n_path = self.node_path(cur_node)
            astar_path = n_path + reeds_path
        
        return arrive, astar_path
        
    def path_collsion_check(self, path_point_list):
        
        for point in path_point_list:
            ap = self.angular_pos(point)   
            if self.grid.check_collision_rectangle(ap):
                return True
        print('no colision')

        return False   
    
    # calculate the angular position of the car
    def angular_pos(self, state):

        if isinstance(state, np.ndarray):
            x = state[0, 0]
            y = state[1, 0]
            phi = state[2, 0]
        else:
            x = state[0]
            y = state[1]
            phi = state[2]

        length = self.car_shape[0]
        width = self.car_shape[1]
        wheelbase = self.car_shape[2]

        car_x0 = - width / 2 
        car_y0 = - (length-wheelbase)/2

        car_x1 = car_x0 
        car_y1 = car_y0 + length

        car_x2 = car_x0 + width
        car_y2 = car_y0 + length

        car_x3 = car_x0 + width
        car_y3 = car_y0

        car_point = np.array([ [car_x0, car_x1, car_x2, car_x3], [car_y0, car_y1, car_y2, car_y3] ])

        r_phi = phi - pi/2
        rotation_matrix = np.array([[cos(r_phi), -sin(r_phi)], [sin(r_phi), cos(r_phi)]])
        transition_matrix = np.array([[x], [y]])
        angular_position = rotation_matrix @ car_point + transition_matrix

        return angular_position

    def node_path(self, node):
        path_point_list = []
        
        while node != None:
            path_point_list.append(node.point)
            node = node.parent
        
        path_point_list.reverse()

        return path_point_list

    def child_node(self, node):
        
        child_node_list = []
        cur_gear = node.point[3, 0]

        for steer in self.steer_list:
            for gear in self.gear_list:
                
                switch_cost = 1 if cur_gear != gear else 0
                reverse_cost = self.step_size if gear == -1 else 0

                cost = node.cost + self.step_size + switch_cost + reverse_cost
                next_node_state = motion_acker_step(node.point, gear, steer, self.step_size, self.min_radius, include_gear=True)
                next_node = self.node_tuple(next_node_state, cost, node)
                child_node_list.append(next_node)

        return child_node_list
    
    def heuristic(self, cur_point, goal_point, reeds_lookup_matrix):
        reeds_cost = self.nh_without_obs(cur_point, goal_point, reeds_lookup_matrix)
        astar_cost = self.hol_obs(cur_point, goal_point)
        h_cost = max(reeds_cost, astar_cost)
        # h_cost = reeds_cost
        return h_cost

    def nh_without_obs(self, cur_point, goal_point, reeds_lookup_matrix):
        # 
        # look up reeds cost in the matrix
        diff = goal_point[0:3] - cur_point[0:3]
        rot_matrix = euler2mat(0, 0, -cur_point[2, 0], 'sxyz')
        new_point = rot_matrix @ diff
        new_point[2, 0] = self.wraptopi(goal_point[2, 0] - cur_point[2, 0])

        index = self.grid.point3index_reeds(new_point)

        length = reeds_lookup_matrix[index]
       
        return length
        
    def hol_obs(self, cur_point=None, goal_point=None):

        start_index = self.grid.point2index(cur_point)
        goal_index = self.grid.point2index(goal_point)
        astar_cost = self.astar.astar_length(start_index, goal_index)
        return astar_cost

    
    def reeds_lookup_cal(self):
        
        print('process of calculating the reeds lookup matrix')

        reeds_cost_matrix = np.zeros((self.grid.len_x, self.grid.wid_y, self.grid.yaw_z))
        start_point = np.zeros((3, 1))

        with tqdm(total=self.grid.len_x * self.grid.wid_y * self.grid.yaw_z) as pbar:
            for i in range(self.grid.len_x):
                for j in range(self.grid.wid_y):
                    for k in range(self.grid.yaw_z):
                        goal_point = np.array([ [ i * self.grid.xy_reso], [j * self.grid.xy_reso], [k * self.grid.yaw_reso * pi / 180] ])
                        
                        reeds_cost_matrix[i, j, k] = self.rs.shortest_length(start_point, goal_point)
                        pbar.update(1)

        with open(self.lookup_path, 'wb') as f:
            np.save(f, reeds_cost_matrix)

        print('completed, save matrix successfully')
        return reeds_cost_matrix

    def wraptopi(self, radian):
    # -pi to pi

        if radian > pi:
            radian2 = radian - 2 * pi
        elif radian < -pi:
            radian2 = radian + 2 * pi
        else:
            radian2 = radian

        return radian2

        


    