import threading

import rospy
import math
from nav_msgs.msg import Odometry
from carla_msgs.msg import CarlaEgoVehicleStatus
from carla_msgs.msg import CarlaEgoVehicleInfo
from carla_msgs.msg import CarlaActorList
from carla_msgs.msg import CarlaActorInfo
from ackermann_msgs.msg._AckermannDrive import AckermannDrive
from std_msgs.msg import Float32
import carla

class info_set():
    def __init__(self):
        self.position_x=None
        self.position_y=None
        self.position_theta=None
        self.wheel_dis=None
        self.car_w=None
        self.v=None

        self.v = None
        self.acc = None
        self.car_turning_angle =None

        self.MAIN_THREAD_END_FLAG = 1
        rospy.init_node('combine_control')
        # 创建一个Publisher，发布名为/laneline_info的topic，消息类型为laneline_publisher::laneline，队列长度10
        self.laneline_info_pub = rospy.Publisher('/carla/ego_vehicle/ackermann_cmd', AckermannDrive, queue_size=10)

        rospy.Subscriber('/carla/ego_vehicle/vehicle_status', CarlaEgoVehicleStatus, self.callback_speed)
        rospy.Subscriber('/carla/ego_vehicle/odometry', Odometry, self.callback_pos)
        rospy.Subscriber('/carla/ego_vehicle/vehicle_info', CarlaEgoVehicleInfo, self.callback_info)


        self.vehicle = self.__get_vehicle()
        # print('actor --',self.vehicle)

        # 设置循环的频率
        rate = rospy.Rate(10)

    def __get_vehicle(self):
        data = rospy.wait_for_message('/carla/actor_list', CarlaActorList, 10)
        car_list=[]
        for i in data.actors:
            # print(i.id,i.type,type(i.type))
            if 'vehicle' in i.type:
                car_list.append(i.id)

        client = carla.Client('localhost', 2000)
        client.set_timeout(2.0)

        world = client.get_world()
        # print("actors --", world.get_actors())
        # print(car_list)
        if len(car_list)==0:
            print("未寻找到车辆")
            exit()
        return world.get_actor(car_list[0])



    # 获取的xyz是车辆的位置信息，rpy0是车辆的绕xyz轴的旋转信息，需要的是最后一个
    def callback_pos(self,data):

        x = data.pose.pose.orientation.x
        y = data.pose.pose.orientation.y
        z = data.pose.pose.orientation.z
        w = data.pose.pose.orientation.w

        self.position_x = data.pose.pose.position.x
        self.position_y = data.pose.pose.position.y
        self.position_theta =  math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        if self.position_x!=None:
            rospy.set_param("/parking_planning/position_x", self.position_x)
            # print()
        if self.position_y!=None:
            rospy.set_param("/parking_planning/position_y", self.position_y)
        if self.position_theta!=None:
            rospy.set_param("/parking_planning/position_theta", self.position_theta)
        # print('x,y,theta = ', self.position_x, self.position_y, self.position_theta * 180 / math.pi)

    def callback_info(self,data):

        self.wheel_dis = abs(data.wheels[0].position.x) + abs(data.wheels[3].position.x)
        self.car_w = abs(2 * data.wheels[0].position.y)

        if self.wheel_dis!=None:
            rospy.set_param("/parking_planning/wheel_dis", self.wheel_dis)
        if self.car_w!=None:
            rospy.set_param("/parking_planning/car_w", self.car_w)

        print('wheel_dis car_w = ', self.wheel_dis, self.car_w)

    # 弧度制
    def callback_speed(self,data):
        self.v = data.velocity
        self.acc = data.acceleration.linear.x
        x = data.orientation.x
        y = data.orientation.y
        z = data.orientation.z
        w = data.orientation.w
        self.car_turning_angle =math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

        if self.v!=None:
            rospy.set_param("/parking_planning/car_speed", self.v)
        if self.acc!=None:
            rospy.set_param("/parking_planning/car_acc", self.acc)
        if self.car_turning_angle!=None:
            rospy.set_param("/parking_planning/car_turning_angle", self.car_turning_angle)

    def get_car_position(self):
        data=rospy.wait_for_message('/carla/ego_vehicle/odometry', Odometry,10)

        x = data.pose.pose.orientation.x
        y = data.pose.pose.orientation.y
        z = data.pose.pose.orientation.z
        w = data.pose.pose.orientation.w

        self.position_x = data.pose.pose.position.x
        self.position_y = data.pose.pose.position.y
        self.position_theta = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        # print(self.position_x,self.position_y,self.position_theta)
        return self.position_x,self.position_y,self.position_theta

    def carla_control(self,target_v,target_w):
        current_speed=self.get_car_speed()
        send_w=target_w/(math.pi / 6)*4
        if current_speed>0:
            send_w=-send_w
        print("carla-control ",' tar = ',target_v,send_w,"current speed",current_speed)

        if send_w>1:
            send_w=1
        if send_w<-1:
            send_w=-1

        if target_v==0:
            self.vehicle.apply_control(carla.VehicleControl(throttle=0, steer=0.0, brake=1, reverse=True))
            return
        force=1.7
        if (target_v*current_speed<0):
            if target_v>0:
                car_reverse = False
                if target_v>current_speed+1:
                    car_brake=1
                    car_throttle=0
                else:
                    car_brake=0
                    car_throttle =(target_v-current_speed)/force
            else:
                car_reverse = True

                if target_v<current_speed-1:
                    car_brake=1
                    car_throttle=0
                else:
                    car_brake=0
                    car_throttle =(current_speed-target_v)/force
        else:
            if target_v>0:
                car_reverse=False
            else:
                car_reverse=True

            abs_current_speed=abs(current_speed)
            abs_target_v=abs(target_v)
            if abs_target_v>abs_current_speed:
                car_brake=0
                car_throttle=(abs_target_v-abs_current_speed)/force
            else:
                car_brake=(abs_current_speed-abs_target_v)/force
                car_throttle=0


        if car_throttle<0:
            car_throttle=0
        if car_throttle>1:
            car_throttle=1

        self.vehicle.apply_control(carla.VehicleControl(throttle=car_throttle, steer=send_w, brake=car_brake, reverse=car_reverse))



    def send_msg(self):

        while (self.MAIN_THREAD_END_FLAG):

            if rospy.has_param("/parking_planning/parking_end"):
                self.MAIN_THREAD_END_FLAG = rospy.get_param("/parking_planning/parking_end")
            # self.MAIN_THREAD_END_FLAG=1
            if rospy.has_param("/parking_planning/car_steer_need") and rospy.has_param(
                    "/parking_planning/car_speed_need"):
                car_steer_need = rospy.get_param("/parking_planning/car_steer_need")
                car_speed_need = rospy.get_param("/parking_planning/car_speed_need")

                # print('speed need ',car_speed_need,' steer need ',car_steer_need)
                ackerman_control=0
                if (ackerman_control):
                    carla_control = AckermannDrive()
                    carla_control.steering_angle = car_steer_need
                    carla_control.speed = car_speed_need

                    # print('update info v =', carla_control.speed, ' w = ', carla_control.steering_angle, ' end = ',
                    #       self.MAIN_THREAD_END_FLAG == 0)

                    # 发布消息
                    self.laneline_info_pub.publish(carla_control)
                else:
                    self.carla_control(car_speed_need,car_steer_need)

        self.carla_control(0,0)

    def get_car_speed(self):
        # data=rospy.wait_for_message('/carla/ego_vehicle/vehicle_status', CarlaEgoVehicleStatus,10)
        # self.v = data.velocity

        data = rospy.wait_for_message('/carla/ego_vehicle/speedometer', Float32, 10)
        self.v=data.data
        return self.v



    def listener(self):
        t = threading.Thread(target=self.send_msg,args=())
        t.daemon = True
        t.start()
        # self.get_car_position()
        print("speed ",self.get_car_speed())




if __name__ == '__main__':
    print('start set carla parm info')

    info=info_set()
    # info.listener()
    # info.carla_control(0,0)