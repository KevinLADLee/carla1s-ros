from math import atan2, pi
import numpy as np

def wraptopi(radian):
    # -pi to pi

    if radian > pi:
        radian2 = radian - 2 * pi
    elif radian < -pi:
        radian2 = radian + 2 * pi
    else:
        radian2 = radian

    # diff = radian1 - radian2
    # print(diff)
    return radian2

def wrapto2pi(radians):
    
    if radians > 2 * pi:
        radians_wrap = radians - 2 * pi
    elif radians < 0:
        radians_wrap = radians + 2 * pi
    else:
        radians_wrap = radians

    return radians_wrap
    
def relative(state1, state2):
        
    dif = state2 - state1

    dis = np.linalg.norm(dif)
    radian = atan2(dif[1, 0], dif[0, 0])
    
    return dis, radian