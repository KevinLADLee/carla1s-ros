import numpy as np
from math import pi, atan2, floor, sin, cos
from collections import namedtuple

class grid_graph:

    def __init__(self, grid_matrix, xy_reso = 1, yaw_reso = 5, threshold=255):
        
        self.grid_matrix = grid_matrix
        self.xy_reso = xy_reso
        self.yaw_reso = yaw_reso # degree
        self.threshold = threshold
        self.len_x = grid_matrix.shape[0]
        self.wid_y = grid_matrix.shape[1]
        self.yaw_z = int(360 / yaw_reso)
    
    def check_collision_line(self, point1=np.zeros((2, 1)), point2=np.zeros((2, 1))):

        # point: (2, 1)
        diff = point2-point1
        length = np.linalg.norm(diff)
        radian = atan2(diff[1, 0], diff[0, 0])

        cur_len = 0
        step_len = self.xy_reso

        while cur_len < length:
            point_step = point1 + step_len*np.array([[cos(radian)], [sin(radian)]])
            cur_len = cur_len + step_len
            point1 = point_step
            
            if self.check_collision_point(point_step):
                return True

        return False
    
    def check_collision_rectangle(self, rectangle=np.zeros((2, 4))):

        line_list = [ [rectangle[:, 0:1], rectangle[:, 1:2]], [rectangle[:, 1:2], rectangle[:, 2:3]], [rectangle[:, 2:3], rectangle[:, 3:4]],  [rectangle[:, 3:4], rectangle[:, 0:1]]]
        for line in line_list:
            if self.check_collision_line(*line) == True:
                return True
        
        return False

    def check_collision_point(self, point):

        index = self.point2index(point)

        if index[0] >= self.grid_matrix.shape[0] or index[1] >= self.grid_matrix.shape[1]:
            return True

        if self.grid_matrix[index] >= self.threshold:
            return True
        
        return False

    def point2index(self, point):

        # if isinstance(point, np.ndarray):
        #     x = point[0, 0]
        #     y = point[1, 0]
        # else:
        #     x = point[0]
        #     y = point[1]
            
        in_x = floor(point[0, 0] / self.xy_reso)
        in_y = floor(point[1, 0] / self.xy_reso)
        
        return (in_x, in_y)

    def point3index(self, point):
        
        x = point[0, 0]
        y = point[1, 0]
        yaw = point[2, 0]

        in_x = floor(abs(x) / self.xy_reso)
        in_y = floor(abs(y) / self.xy_reso)

        if yaw < 0:
            new_yaw = yaw + 2*pi
        else:
            new_yaw = yaw

        degree = new_yaw * 180 / pi

        in_yaw = floor( degree / self.yaw_reso) 
        # in_yaw = 0
        return (in_x, in_y, in_yaw)
    
    def point3index_reeds(self, point):

        x = point[0, 0]
        y = point[1, 0]
        yaw = point[2, 0]
       
        if x < 0:
            x = -x
            yaw = - yaw
            
        if y< 0:
            y = -y
            yaw = - yaw

        in_x = floor(x / self.xy_reso)
        in_y = floor(y / self.xy_reso)

        if yaw < 0:
            new_yaw = yaw+ 2*pi
        else:
            new_yaw = yaw

        degree = new_yaw * 180 / pi
        in_yaw = floor( degree / self.yaw_reso) 

        return (in_x, in_y, in_yaw)

    






    # def neighbors(self, node):
    #     # x, y, cost

    #     dirs = [[1, 0, 1], [0, 1, 1], [-1, 0, 1], [0, -1, 1],
    #             [-1, -1, math.sqrt(2)], [-1, 1, math.sqrt(2)], [1, -1, math.sqrt(2)], [1, 1, math.sqrt(2)] ]  

    #     node_neighbors = []

    #     for dir in dirs:

    #         new_x = dir[0] + node.x
    #         new_y = dir[1] + node.y
    #         new_cost = dir[2] + node.cost
            
    #         if new_x < self.width and new_y < self.height and new_x >= 0 and new_y >= 0:
    #             if self.obstacle[new_x, new_y] != 1 :

    #                 node_neighbor = self.node_tuple(new_x, new_y, new_cost, node)
    #                 node_neighbors.append(node_neighbor)
    
    #     return  node_neighbors

    # def node_equal(self, node1, node2):
        
    #     return node1.x == node2.x and node1.y == node2.y

    