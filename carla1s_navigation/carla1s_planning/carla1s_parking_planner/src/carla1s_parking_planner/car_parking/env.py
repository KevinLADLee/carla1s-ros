class Env():
    def __init__(self, pl=None, pw=None, rw=None, cl=None, cw=None, min_r=None, wl=None, rl=None,
                 hx=None, step=None, parking_left_front_point=None, parking_right_front_point=None):
        self.car_l = cl
        self.car_w = cw
        self.min_turning_radiu = min_r
        self.wheel_dis = wl
        self.hou_xuan = hx
        self.step = step

        self.parking_l = pl
        self.parking_w = pw
        self.road_w = rw
        self.road_l = rl
        if parking_left_front_point != None:
            self.parking_left_front_point_x = parking_left_front_point[0]
            self.parking_left_front_point_y = parking_left_front_point[1]
        else:
            self.parking_left_front_point_x = None
            self.parking_left_front_point_y = None

        if parking_right_front_point != None:
            self.parking_right_front_point_x = parking_right_front_point[0]
            self.parking_right_front_point_y = parking_right_front_point[1]
        else:
            self.parking_right_front_point_x = None
            self.parking_right_front_point_y = None

    def set_car_info(self, cl, cw, min_r, wl, hx, step):
        self.car_l = cl
        self.car_w = cw
        self.min_turning_radiu = min_r
        self.wheel_dis = wl
        self.hou_xuan = hx
        self.step = step

    def set_env_info(self, pl, pw, rw, rl, parking_left_front_point, parking_right_front_point):
        self.parking_l = pl
        self.parking_w = pw
        self.road_w = rw
        self.road_l = rl
        self.parking_left_front_point_x = parking_left_front_point[0]
        self.parking_left_front_point_y = parking_left_front_point[1]
        self.parking_right_front_point_x = parking_right_front_point[0]
        self.parking_right_front_point_y = parking_right_front_point[1]

    def show(self):
        print('==============================')
        print('self.car_l =',self.car_l )
        print('self.car_w = ',self.car_w )
        print('self.min_turning_radiu  = ',self.min_turning_radiu )
        print('self.wheel_dis  = ',self.wheel_dis )
        print('self.hou_xuan = ',self.hou_xuan)
        print('self.step  = ',self.step )

        print('self.parking_l  = ',self.parking_l )
        print('self.parking_w  = ',self.parking_w )
        print('self.road_w  = ',self.road_w )
        print('self.road_l  = ',self.road_l )
        print('==============================')

