import rospy

class ParkingPlanningNode():
    def __init__(self):
        self.role_name = rospy.get_param("role_name", 'ego_vehicle')
        rospy.init_node('/carla/{}/parking_planning_node'.format(self.role_name))
        rospy.Subscriber('/carla/{}/vehicle_info',)