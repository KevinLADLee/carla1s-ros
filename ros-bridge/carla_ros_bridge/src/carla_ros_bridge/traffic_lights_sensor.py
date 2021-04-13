#!/usr/bin/env python
#
# Copyright (c) 2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
a sensor that reports the state of all traffic lights
"""
import geometry_msgs.msg
import std_msgs.msg
from carla_msgs.msg import (
    CarlaTrafficLightStatusList,
    CarlaTrafficLightInfoList
)
from carla_ros_bridge.traffic import TrafficLight
from carla_ros_bridge.pseudo_actor import PseudoActor
from ros_compatibility import QoSProfile, latch_on

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import carla
import carla_common.transforms as trans

class TrafficLightsSensor(PseudoActor):
    """
    a sensor that reports the state of all traffic lights
    """

    def __init__(self, uid, name, parent, node, actor_list):
        """
        Constructor

        :param uid: unique identifier for this object
        :type uid: int
        :param name: name identiying the sensor
        :type name: string
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param node: node-handle
        :type node: CompatibleNode
        :param actor_list: current list of actors
        :type actor_list: map(carla-actor-id -> python-actor-object)
        """

        super(TrafficLightsSensor, self).__init__(uid=uid,
                                                  name=name,
                                                  parent=parent,
                                                  node=node)

        self.actor_list = actor_list
        self.traffic_light_status = CarlaTrafficLightStatusList()
        self.traffic_light_actors = []

        self.traffic_lights_info_publisher = node.new_publisher(
            CarlaTrafficLightInfoList,
            self.get_topic_prefix() + "/info", qos_profile=QoSProfile(depth=10, durability=latch_on))
        self.traffic_lights_status_publisher = node.new_publisher(
            CarlaTrafficLightStatusList,
            self.get_topic_prefix() + "/status",
            qos_profile=QoSProfile(depth=10, durability=latch_on))
        self.traffic_lights_markers_publisher = node.new_publisher(
            MarkerArray,
            self.get_topic_prefix() + "/markers",
            qos_profile=QoSProfile(depth=10, durability=latch_on))

        self.mark_array = MarkerArray()
        self.id = 0

    def destroy(self):
        """
        Function to destroy this object.
        :return:
        """
        super(TrafficLightsSensor, self).destroy()
        self.actor_list = None
        self.node.destroy_publisher(self.traffic_lights_info_publisher)
        self.node.destroy_publisher(self.traffic_lights_status_publisher)
        self.node.destroy_publisher(self.traffic_lights_markers_publisher)

    @staticmethod
    def get_blueprint_name():
        """
        Get the blueprint identifier for the pseudo sensor
        :return: name
        """
        return "sensor.pseudo.traffic_lights"

    def update(self, frame, timestamp):
        """
        Get the state of all known traffic lights
        """
        traffic_light_status = CarlaTrafficLightStatusList()
        traffic_light_actors = []
        for actor_id in self.actor_list:
            actor = self.actor_list[actor_id]
            if isinstance(actor, TrafficLight):
                traffic_light_actors.append(actor)
                traffic_light_status.traffic_lights.append(actor.get_status())

        if traffic_light_actors != self.traffic_light_actors:
            self.traffic_light_actors = traffic_light_actors
            traffic_light_info_list = CarlaTrafficLightInfoList()

            for traffic_light in traffic_light_actors:
                traffic_light_info_list.traffic_lights.append(traffic_light.get_info())

            self.traffic_lights_info_publisher.publish(traffic_light_info_list)

        if traffic_light_status != self.traffic_light_status:
            self.traffic_light_status = traffic_light_status
            self.traffic_lights_status_publisher.publish(traffic_light_status)

        self.mark_array.markers.clear()
        for traffic_light in traffic_light_actors:
            self.create_traffic_light_marker(traffic_light)
        self.traffic_lights_markers_publisher.publish(self.mark_array)

    def create_traffic_light_marker(self, traffic_light):
        info = traffic_light.get_info()
        status = traffic_light.get_status()

        tl = traffic_light.carla_actor
        tl_t = tl.get_transform()
        transformed_tv = tl_t.transform(tl.trigger_volume.location)

        marker = Marker()
        marker.type = Marker.CUBE
        marker.header.frame_id = "map"
        marker.id = info.id
        marker.pose.position = trans.carla_location_to_ros_vector3(transformed_tv)
        marker.scale.x = info.trigger_volume.size.x
        marker.scale.y = info.trigger_volume.size.y
        marker.scale.z = info.trigger_volume.size.z

        if status.state == 0:
            marker.color = std_msgs.msg.ColorRGBA(1,0,0,0.5)
        elif status.state == 1:
            marker.color = std_msgs.msg.ColorRGBA(1,1,0,0.5)
        elif status.state == 2:
            marker.color = std_msgs.msg.ColorRGBA(0,1,0,0.5)
        else:
            marker.color = std_msgs.msg.ColorRGBA(0,0,0,0)
        self.mark_array.markers.append(marker)
        return marker