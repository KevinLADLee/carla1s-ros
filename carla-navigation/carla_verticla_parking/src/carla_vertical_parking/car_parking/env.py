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



