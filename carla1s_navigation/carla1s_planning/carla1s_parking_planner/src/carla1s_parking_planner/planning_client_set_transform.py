#! /usr/bin/env python
import tf
import roslib
# roslib.load_manifest('carla_nav_msgs')
import rospy
import actionlib
from nav_msgs.msg import Odometry
import time
# from carla_control import CarlaControl
# from carla_nav_msgs.msg import ParkingPlannerAction, ParkingPlannerActionGoal, \
#     ParkingPlannerFeedback, ParkingPlannerActionResult, ParkingPlannerResult,ParkingPlannerGoal
from carla1s_msgs.msg import ParkingPlannerAction, ParkingPlannerActionGoal, \
    ParkingPlannerFeedback, ParkingPlannerActionResult, ParkingPlannerResult,ParkingPlannerGoal
import math
import carla
# from actionlib_parking.msg import parking_planningAction,parking_planningGoal



class CarlaControl():
    def __init__(self,host='localhost',port=2000):
        self.client = carla.Client(host, port)
        self.client.set_timeout(5.0)
        self.world = self.client.get_world()
        self.actor_list =  []

        # print(dir(self.world ))

    def get_vehicle(self,id=None):
        import rospy
        from carla_msgs.msg import CarlaActorList
        if id==None:
            data = rospy.wait_for_message('/carla/actor_list', CarlaActorList, 10)
            car_list = []
            for i in data.actors:
                # print(i.id,i.type,type(i.type))
                if 'vehicle' in i.type:
                    car_list.append(i.id)

            if len(car_list) == 0:
                print("未寻找到车辆")
                exit()
            return self.world.get_actor(car_list[0])
        else:
            return self.world.get_actor(id)


    def close(self):
        for name,actor in self.actor_list:
            actor.destroy()
        print("All cleaned up!")

def print_route(rx, ry, rtheta ):
    print('[')
    for i in range(len(rx)):
        print('[',rx[i],',',ry[i],',',rtheta[i],'],')
    print(']')

def carla_set_transform(car,x_arr,y_arr,theta_arr):
    from carla import Transform, Location, Rotation
    # print(x_arr)
    location=car.get_location()
    # rotation = car.get_rotation()
    # print(dir(car))
    print(location)
    # print(rotation)
    print('-------------')
    for i  in range(len(x_arr)):
        time.sleep(0.01)

        spawn_point = Transform(Location(x=x_arr[i], y=-y_arr[i], z=location.z),
                                Rotation(pitch=0, yaw=-theta_arr[i]*180/math.pi, roll=0))
        print(spawn_point)
        car.set_transform(spawn_point)
        # car.set_location(Location(x=x_arr[i], y=y_arr[i], z=location.z))

def set_transform_tracking(path,host='localhost',port=2000,id=None):
    for i in range(len(path.paths)):
        x_arr=[]
        y_arr=[]
        theta_arr=[]
        for pose in path.paths[i].poses:
            x=pose.pose.position.x
            y=pose.pose.position.y
            (_, _, theta) = tf.transformations.euler_from_quaternion(
                [pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w])
            x_arr.append(x)
            y_arr.append(y)
            theta_arr.append(theta)
        # print(x_arr)
        carla_control=CarlaControl(host,port)
        time.sleep(1)

        carla_car=carla_control.get_vehicle(id)
        carla_set_transform(carla_car,x_arr,y_arr,theta_arr)
        carla_control.close()



def client(parking_left_head_x,parking_left_head_y,parking_right_head_x,parking_right_head_y,position_x,position_y,position_theta,log=False):


    msg= ParkingPlannerGoal()
    # print(msg)

    # 旧的msg格式
    msg.parking_spot.width=2.5
    msg.parking_spot.length=5

    msg.parking_spot.center_pose.position.x=14
    msg.parking_spot.center_pose.position.y=27
    msg.parking_spot.center_pose.position.z=0

    #停车位的角度
    q=tf.transformations.quaternion_from_euler(0, 0, 0)
    
    msg.parking_spot.center_pose.orientation.x=q[0]
    msg.parking_spot.center_pose.orientation.y=q[1]
    msg.parking_spot.center_pose.orientation.z=q[2]
    msg.parking_spot.center_pose.orientation.w=q[3]
    

    """
    # print(dir(msg.parking_spot_corners))
    # msg.header=5
    msg.road_width=5
    msg.parking_spot_length=7
    corners=[]
    corners.append([16.6 ,28.4])
    corners.append([16.6,26.1])
    msg.parking_spot_corners=corners
    
    """

    if log==True:
        print("getting  result")

    # # Fill in the goal here
    client = actionlib.SimpleActionClient("vertical_parking", ParkingPlannerAction)
    client.wait_for_server()
    client.send_goal(msg)

    result = client.wait_for_result(rospy.Duration.from_sec(500))
    print("result ",result)
    ans = client.get_result()


    set_transform_tracking(ans.path_array)

    if log==True:
        # print_route(ans.path_x, ans.path_y, ans.path_theta)
        # print('len path = ', len(ans.path_x))
        # print('ans ',ans)
        # print(ParkingPlannerResult())
        print(ans.path_array)
        for i in range(len(ans.path_array.paths)):
            print("=======================================================")
            for j in range(len(ans.path_array.paths[i].poses)):
                print(ans.path_array.paths[i].poses[j].pose.position.x,ans.path_array.paths[i].poses[j].pose.position.y)

    # return ans.path_x, ans.path_y, ans.path_theta

if __name__ == '__main__':
    rospy.init_node('parking_planning_client')

    #carla town5
    # parking_left_head_x = -31.6
    # parking_left_head_y = 25.5
    # parking_right_head_x = -31.6
    # parking_right_head_y = 28.7
    #华为车库
    parking_left_head_x = -42.3
    parking_left_head_y = -3.1
    parking_right_head_x = -42.3
    parking_right_head_y = -6.1

    data = rospy.wait_for_message('/carla/ego_vehicle/odometry', Odometry, 10)

    x = data.pose.pose.orientation.x
    y = data.pose.pose.orientation.y
    z = data.pose.pose.orientation.z
    w = data.pose.pose.orientation.w
    # print(x,y,z,w)

    position_x = data.pose.pose.position.x
    position_y = data.pose.pose.position.y
    position_theta = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

    # print('car position ',position_x,position_y,position_theta*180/math.pi)
    # position_x = -33
    # position_y  = 30
    # position_theta = math.pi / 2

    client(parking_left_head_x,parking_left_head_y,parking_right_head_x,parking_right_head_y,position_x,position_y,position_theta,log=False)
