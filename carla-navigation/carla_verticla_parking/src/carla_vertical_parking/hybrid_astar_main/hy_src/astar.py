import heapdict
import numpy as np

from collections import namedtuple
from math import atan2, cos, sin, floor, sqrt, inf

class astar:
    def __init__(self, grid_graph, collision_radius=1):
        self.grid = grid_graph
        self.node_tuple = namedtuple('node', 'index, cost, parent')  
        self.cr = collision_radius
        self.actions = [(0, 1, 1), (0, -1, 1), (1, 0, 1), (1, 1, 1.4142), (1, -1, 1.4142), (-1, 0, 1), (-1, 1, 1.4142), (-1, -1, 1.4142)]
        self.cr_index = int(self.cr / self.grid.xy_reso)

    def astar_search(self, start_index, goal_index):

        start_node = self.node_tuple(start_index, 0, None)
        goal_node = self.node_tuple(goal_index, 0, None)

        open_set = heapdict.heapdict()
        open_set[start_node] = 0 
        open_cell_index=[start_index] 
        
        close_cell_index=[]
        g_cost = np.zeros((self.grid.len_x, self.grid.wid_y))
        
        iter = 0

        while len(open_set.keys()) != 0:

            cur_node = open_set.popitem()[0]
            cur_index = cur_node.index
            
            if iter % 10 == 0:
                arrive, node = self.direct_check(cur_node, goal_index)
                if arrive:
                    return node
            
            # child node expansions
            child_node_list = self.child_node(cur_node)

            for child_node in child_node_list:
                
                if child_node.index in close_cell_index:
                    continue
                
                if child_node.index not in open_cell_index or child_node.cost < g_cost[child_node.index]:
                    g_cost[child_node.index] = child_node.cost
                    priority = child_node.cost + self.heuristic(child_node.index, goal_index)
                    open_set[child_node] = priority

                    if child_node.index not in open_cell_index:
                        open_cell_index.append(child_node.index)

                # close_cell_index.append(child_node.index)
            close_cell_index.append(cur_index)
            iter = iter + 1

        return None

    def direct_check(self, node, goal_index):

        index = node.index
        step = 1
        dis = np.sqrt( (goal_index[1] - index[1])**2 + (goal_index[0] - index[0])**2)
        rad = atan2(goal_index[1] - index[1], goal_index[0] - index[0])

        cur_len = 0
        while cur_len < dis:
            cur_len = cur_len + step
            new_x = cur_len * cos(rad) + index[0]
            new_y = cur_len * sin(rad) + index[1]
            new_index = (int(floor(new_x)), int(floor(new_y)))

            if self.check_collision(new_index):
                return False, node

        return True, node
            
    def heuristic(self, cur_index, goal_index):
        return np.sqrt( (goal_index[1] - cur_index[1])**2 + (goal_index[0] - cur_index[0])**2 )                

    def child_node(self, node):

        child_node_list = []
        for action in self.actions:
            new_index = (node.index[0] + action[0], node.index[1] + action[1])
            if not self.check_collision(new_index):
                new_cost = node.cost + action[2] * self.grid.xy_reso
                new_node = self.node_tuple(new_index, new_cost, node)
                child_node_list.append(new_node)

        return child_node_list

    def check_collision1(self, index):
        
        x = np.arange(index[0] - self.cr_index, index[0] + self.cr_index+1)
        y = np.arange(index[1] - self.cr_index, index[1] + self.cr_index+1)
        
        X,Y = np.meshgrid(x,y)
        collision_area = self.grid.grid_matrix[X,Y]
        
        if np.max(collision_area)>= self.grid.threshold:
            return True
        else:
            return False

    def check_collision(self, index):
        
        x = np.arange(index[0] - self.cr_index, index[0] + self.cr_index+1)
        y = np.arange(index[1] - self.cr_index, index[1] + self.cr_index+1)

        for i in x:
            for j in y:
                if self.grid.grid_matrix[i, j] >= self.grid.threshold:
                    return True
                    
        return False
    
    def check_collision1(self, index):

        action_index = int(self.cr / self.grid.xy_reso) + 1
        for i in range(-action_index, action_index):
            for j in range(-action_index, action_index):
                new_index = (index[0] + i, index[1] + j)
                if self.grid.grid_matrix[new_index] >= self.grid.threshold:
                    return True
        
        return False

    def astar_path(self, start_index, goal_index):
        
        node = self.astar_search(start_index, goal_index)

        path_point_list = []

        cur_node = node

        while cur_node != None:

            cur_index = cur_node.index
            cur_point = np.array([[cur_index[0] * self.grid.xy_reso], [cur_index[1] * self.grid.xy_reso]])
            path_point_list.append(cur_point)
            cur_node = cur_node.parent
        
        path_point_list.reverse()
        goal_point = np.array([[goal_index[0] * self.grid.xy_reso], [goal_index[1] * self.grid.xy_reso]])
        path_point_list.append(goal_point)

        return path_point_list
    
    def astar_length(self, start_index, goal_index):
        node = self.astar_search(start_index, goal_index)

        if node is None:
            return inf

        length_to_goal = sqrt( (node.index[1] - goal_index[1]) ** 2 + (node.index[0] - goal_index[0]) ** 2 ) * self.grid.xy_reso

        return node.cost + length_to_goal


    def equal_index(self, index1, index2):
        return index1[0] == index2[0] and index1[1] == index2[1]