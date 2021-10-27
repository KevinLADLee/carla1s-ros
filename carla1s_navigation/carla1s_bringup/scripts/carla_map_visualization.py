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
import xml.etree.ElementTree as ET
import numpy as np
import math

import carla

import carla_common.transforms as trans
# from carla1s_msgs import ParkingSpotArray, ParkingSpot


import rospy
from rospy import ROSException

from std_msgs.msg import String
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
from carla_msgs.msg import CarlaWorldInfo

COLOR_ALUMINIUM_2 = ColorRGBA(186.0 / 255.0, 189.0 / 255.0, 182.0 / 255.0, 1)
COLOR_SKY_BLUE_0 = ColorRGBA(114.0 / 255.0, 159.0 / 255.0, 207.0 / 255.0, 1)
COLOR_CHAMELEON_0 = ColorRGBA(138.0 / 255.0, 226.0 / 255.0, 52.0 / 255.0, 1)
COLOR_SCARLET_RED_0 = ColorRGBA(239.0 / 255.0, 41.0 / 255.0, 41.0 / 255.0, 1)
COLOR_ORANGE_0 = ColorRGBA(252.0 / 255.0, 175.0 / 255.0, 62.0 / 255.0, 1)

class CarlaParkingSpot:
    def __init__(self, x, y, yaw, width, length) -> None:
        self.trans = carla.Transform(carla.Location(x, y, 0.0), carla.Rotation(yaw=yaw))
        self.width = width
        self.length = length
        self.cornor_points = []
        self.update_cornor_points()
        pass

    def update_cornor_points(self):
        w = 0.5 * self.width
        l = 0.5 * self.length
        x = self.trans.location.x
        y = self.trans.location.y
        self.cornor_points.append(self.trans.transform(carla.Location( l, w)))
        self.cornor_points.append(self.trans.transform(carla.Location( l, -w)))
        self.cornor_points.append(self.trans.transform(carla.Location(-l, -w)))
        self.cornor_points.append(self.trans.transform(carla.Location(-l, w)))
        # Add twice for viz
        self.cornor_points.append(self.trans.transform(carla.Location( l, w)))        


class CarlaMapVisualization:
    """
    Pseudo opendrive sensor
    """

    def __init__(self):
        self.world = None
        self.connect_to_carla()
        self.map = self.world.get_map()
        self.map_name = self.map.name
        rospy.loginfo("Map Visualization Node: Loading {} map!".format(self.map_name))
        self.map_viz_publisher = rospy.Publisher('/carla/map_visualization', MarkerArray, latch=True, queue_size=1)

        self.id = 0
        self.marker_array = MarkerArray()

    def connect_to_carla(self):

        rospy.loginfo("Map Visualization Node: Waiting for CARLA world (topic: /carla/world_info)...")
        try:
            rospy.wait_for_message("/carla/world_info", CarlaWorldInfo, timeout=rospy.Duration(secs=15))
        except ROSException as e:
            rospy.logerr("Map Visualization Node: Error while waiting for world info: {}".format(e))
            raise e

        host = rospy.get_param("~"+"host", "127.0.0.1")
        port = rospy.get_param("~"+"port", 2000)
        timeout = rospy.get_param("timeout", 10)
        rospy.loginfo("Map Visualization Node: CARLA world available. Trying to connect to {host}:{port}".format(
            host=host, port=port))

        carla_client = carla.Client(host=host, port=port)
        carla_client.set_timeout(timeout)

        try:
            self.world = carla_client.get_world()
        except RuntimeError as e:
            rospy.logerr("Error while connecting to Carla: {}".format(e))
            raise e

        rospy.loginfo("Connected to Carla.")

    def publish_msgs(self):
        """
        Function (override) to update this object.
        """
        self.draw_map()

        self.draw_parking_spot()

        rospy.loginfo(
            "Map Visualization Node: Got {} markers for carla map visualization".format(len(self.marker_array.markers)))

        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.map_viz_publisher.publish(self.marker_array)
            r.sleep()

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
        arrow_marker.ns = "map_visulization"
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
            # point.z = 0
            arrow_marker.points.append(point)
        self.marker_array.markers.append(arrow_marker)

    def add_line_strip_marker(self, color=None, points=None):
        marker = Marker()
        marker.id = self.set_marker_id()
        marker.type = Marker.LINE_STRIP
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = "map"
        marker.ns = "map_visulization"

        if color is None:
            marker.color = ColorRGBA(1, 1, 1, 1)
        else:
            marker.color = color

        marker.scale.x = 0.25
        marker.pose.orientation.w = 1

        if points is not None:
            for p in points:
                point = Point()
                point.x = p.x
                point.y = -p.y
                point.z = p.z
                point.z = 0
                marker.points.append(point)
        self.marker_array.markers.append(marker)
        return marker

    def draw_map(self):
        precision = 0.1
        topology = self.map.get_topology()
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
            set_waypoints.append(waypoints)

        for waypoints in set_waypoints:
            waypoint = waypoints[0]
            road_left_side = [self.lateral_shift(w.transform, -w.lane_width * 0.5) for w in waypoints]
            road_right_side = [self.lateral_shift(w.transform, w.lane_width * 0.5) for w in waypoints]
            # road_points = road_left_side + [x for x in reversed(road_right_side)]
            # self.add_line_strip_marker(points=road_points)

            if len(road_left_side) > 2:
                self.add_line_strip_marker(points=road_left_side)
            if len(road_right_side) > 2:
                self.add_line_strip_marker(points=road_right_side)

            if not waypoint.is_junction:
                for n, wp in enumerate(waypoints):
                    if ((n + 1) % 400) == 0:
                        self.add_arrow_line_marker(wp.transform)

    def draw_parking_spot(self):
        rospy.loginfo("Map Visualization Node: Drawing parking spot")

        self.opendrive_xml_root = ET.fromstring(self.map.to_opendrive().encode())

        parking_spot_carla_trans = []
        l_x, l_y, l_yaw = None, None, None
        for road_element in self.opendrive_xml_root.findall('road'):
            for objects_element in road_element.findall('objects'):
                for plan_view_element in road_element.findall('planView'):
                    for geometry_element in plan_view_element:
                        if len(geometry_element.findall('line')) == 1:
                            l_x = float(geometry_element.attrib['x'])
                            l_y = float(geometry_element.attrib['y'])
                            l_yaw = float(geometry_element.attrib['hdg'])
                for object_element in objects_element:
                    if object_element.attrib['type'] == 'parking':
                        s = float(object_element.attrib['s'])
                        t = float(object_element.attrib['t'])
                        yaw2 = float(object_element.attrib['hdg'])
                        width = float(object_element.attrib['width'])
                        length = float(object_element.attrib['length'])
                        # print("x: {}, y: {}, yaw: {}, s: {}, t: {}, yaw2: {}, ".format(x, y, yaw, s, t, yaw2))
                        spot_x = l_x + s * math.cos(l_yaw) - t * math.sin(l_yaw)
                        spot_y = -(l_y + s * math.sin(l_yaw) + t * math.cos(l_yaw))
                        spot_yaw = -(l_yaw - yaw2) * 180.0 / math.pi
                        spot = CarlaParkingSpot(spot_x, spot_y, spot_yaw, width, length)
                        parking_spot_carla_trans.append(spot)
                        
        print(len(parking_spot_carla_trans))                
        for spot in parking_spot_carla_trans:
            self.add_line_strip_marker(color=COLOR_SCARLET_RED_0, points=spot.cornor_points)


def main(args=None):
    """
    main function
    """
    rospy.init_node("carla_map_visualization", args)

    carla_map_visualization = None
    try:
        carla_map_visualization = CarlaMapVisualization()
        carla_map_visualization.publish_msgs()

        rospy.spin()
    except (RuntimeError, ROSException):
        pass
    except KeyboardInterrupt:
        print("User requested shut down.")
    finally:
        print("Shutting down.")


if __name__ == "__main__":
    main()
