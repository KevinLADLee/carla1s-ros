#!/usr/bin/env python
#
# Copyright (c) 2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
handle a opendrive sensor
"""

import carla
import rospy
from carla_ros_bridge.pseudo_actor import PseudoActor

from std_msgs.msg import String
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point

from ros_compatibility import QoSProfile, latch_on

COLOR_ALUMINIUM_2 = ColorRGBA(186.0 / 255.0, 189.0 / 255.0, 182.0 / 255.0, 1)
COLOR_SKY_BLUE_0 = ColorRGBA(114.0 / 255.0, 159.0 / 255.0, 207.0 / 255.0, 1)
COLOR_CHAMELEON_0 = ColorRGBA(138.0 / 255.0, 226.0 / 255.0, 52.0 / 255.0, 1)
COLOR_SCARLET_RED_0 = ColorRGBA(239.0 / 255.0, 41.0 / 255.0, 41.0 / 255.0, 1)
COLOR_ORANGE_0 = ColorRGBA(252.0 / 255.0, 175.0 / 255.0, 62.0 / 255.0, 1)


class OpenDriveSensor(PseudoActor):
    """
    Pseudo opendrive sensor
    """

    def __init__(self, uid, name, parent, node, carla_map):
        """
        Constructor

        :param uid: unique identifier for this object
        :type uid: int
        :param name: name identiying this object
        :type name: string
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param node: node-handle
        :type node: carla_ros_bridge.CarlaRosBridge
        :param carla_map: carla map object
        :type carla_map: carla.Map
        """
        super(OpenDriveSensor, self).__init__(uid=uid,
                                              name=name,
                                              parent=parent,
                                              node=node)
        self.carla_map = carla_map
        self._map_published = False
        self.map_publisher = node.new_publisher(String, self.get_topic_prefix(),
                                                qos_profile=QoSProfile(depth=10, durability=latch_on))
        self.map_viz_publisher = node.new_publisher(MarkerArray, self.get_topic_prefix() + '_visualization',
                                                    qos_profile=QoSProfile(depth=10, durability=latch_on))

        self.id = 0
        self.marker_array = MarkerArray()

    def destroy(self):
        super(OpenDriveSensor, self).destroy()
        self.node.destroy_publisher(self.map_publisher)
        self.node.destroy_publisher(self.map_viz_publisher)

    @staticmethod
    def get_blueprint_name():
        """
        Get the blueprint identifier for the pseudo sensor
        :return: name
        """
        return "sensor.pseudo.opendrive_map"

    def update(self, frame, timestamp):
        """
        Function (override) to update this object.
        """
        if not self._map_published:
            self.map_publisher.publish(String(data=self.carla_map.to_opendrive()))
            self.draw_map()
            self.map_viz_publisher.publish(self.marker_array)
            self._map_published = True

    @staticmethod
    def lateral_shift(transform, shift):
        """Makes a lateral shift of the forward vector of a transform"""
        transform.rotation.yaw += 90
        return transform.location + shift * transform.get_forward_vector()

    def set_marker_id(self):
        self.id += 1
        return self.id - 1

    def add_arrow_line_marker(self, transform):
        arrow_marker = Marker()
        arrow_marker.type = Marker.LINE_LIST
        arrow_marker.header.frame_id = "map"
        arrow_marker.id = self.set_marker_id()
        arrow_marker.color = ColorRGBA(0.8, 0.8, 0.8, 1)
        arrow_marker.scale.x = 0.2
        arrow_marker.pose.orientation.w = 1
        transform.rotation.yaw += 180
        forward = transform.get_forward_vector()
        transform.rotation.yaw += 90
        right_dir = transform.get_forward_vector()
        end = transform.location
        start = end - 2.0 * forward
        right = start + 0.8 * forward + 0.4 * right_dir
        left = start + 0.8 * forward - 0.4 * right_dir
        points = [start, end, start, left, start, right]
        for p in points:
            point = Point()
            point.x = p.x
            point.y = -p.y
            point.z = p.z
            arrow_marker.points.append(point)
        self.marker_array.markers.append(arrow_marker)

    def add_line_strip_marker(self, color=None, points=None):
        marker = Marker()
        marker.id = self.set_marker_id()
        marker.type = Marker.LINE_STRIP
        # marker.type = Marker.SPHERE_LIST
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = "map"

        if color is None:
            marker.color = ColorRGBA(1, 1, 1, 1)
        else:
            marker.color = color

        marker.scale.x = 0.1
        marker.pose.orientation.w = 1

        if points is not None:
            for p in points:
                point = Point()
                point.x = p.x
                point.y = -p.y
                point.z = p.z
                marker.points.append(point)
        self.marker_array.markers.append(marker)
        return marker

    def draw_map(self):
        precision = 0.05
        topology = self.carla_map.get_topology()
        topology = [x[0] for x in topology]
        topology = sorted(topology, key=lambda w: w.transform.location.z)
        set_waypoints = []
        for waypoint in topology:
            waypoints = [waypoint]
            nxt = waypoint.next(precision)
            if len(nxt) > 0:
                nxt = nxt[0]
                while nxt.road_id == waypoint.road_id:
                    waypoints.append(nxt)
                    nxt = nxt.next(precision)
                    if len(nxt) > 0:
                        nxt = nxt[0]
                    else:
                        break
            if waypoint.lane_type != carla.LaneType.Stop:
                print(waypoint.lane_type)
                set_waypoints.append(waypoints)


        for waypoints in set_waypoints:
            waypoint = waypoints[0]
            road_left_side = [self.lateral_shift(w.transform, -w.lane_width * 0.5) for w in waypoints]
            road_right_side = [self.lateral_shift(w.transform, w.lane_width * 0.5) for w in waypoints]
            road_points = road_left_side + [x for x in reversed(road_right_side)]
            self.add_line_strip_marker(points=road_points)

            if not waypoint.is_junction:
                for n, wp in enumerate(waypoints):
                    if ((n + 1) % 400) == 0:
                        self.add_arrow_line_marker(wp.transform)
