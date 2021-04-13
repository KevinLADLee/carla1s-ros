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
            self.draw_roads()
            self._map_published = True

    @staticmethod
    def lateral_shift(transform, shift):
        """Makes a lateral shift of the forward vector of a transform"""
        transform.rotation.yaw += 90
        return transform.location + shift * transform.get_forward_vector()

    def set_marker_id(self):
        self.id += 1
        return self.id - 1

    def draw_arrow(self, transform):
        arrow_marker = Marker()
        arrow_marker.type = Marker.LINE_LIST
        arrow_marker.header.frame_id = "map"
        arrow_marker.id = self.set_marker_id()
        arrow_marker.color.r = 0.8
        arrow_marker.color.g = 0.8
        arrow_marker.color.b = 0.8
        arrow_marker.color.a = 1
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

    def draw_line_strip(self, points):
        line_marker = Marker()
        line_marker.header.frame_id = "map"
        line_marker.id = self.set_marker_id()
        line_marker.type = Marker.LINE_STRIP
        line_marker.color.r = 1
        line_marker.color.g = 1
        line_marker.color.b = 1
        line_marker.color.a = 1
        line_marker.scale.x = 0.2
        line_marker.pose.orientation.w = 1
        for p in points:
            point = Point()
            point.x = p.x
            point.y = -p.y
            point.z = p.z
            line_marker.points.append(point)
        self.marker_array.markers.append(line_marker)

    def get_lane_markings(self, lane_marking_type, lane_marking_color, waypoints, sign):
        """For multiple lane marking types (SolidSolid, BrokenSolid, SolidBroken and BrokenBroken), it converts them
         as a combination of Broken and Solid lines"""
        margin = 0.25
        # margin = 2
        marking_1 = [self.lateral_shift(w.transform, sign * w.lane_width * 0.5) for w in waypoints]
        if lane_marking_type == carla.LaneMarkingType.Broken or (lane_marking_type == carla.LaneMarkingType.Solid):
            return [(lane_marking_type, lane_marking_color, marking_1)]
        else:
            marking_2 = [self.lateral_shift(w.transform, sign * (w.lane_width * 0.5 + margin * 2)) for w in waypoints]
            if lane_marking_type == carla.LaneMarkingType.SolidBroken:
                return [(carla.LaneMarkingType.Broken, lane_marking_color, marking_1),
                        (carla.LaneMarkingType.Solid, lane_marking_color, marking_2)]
            elif lane_marking_type == carla.LaneMarkingType.BrokenSolid:
                return [(carla.LaneMarkingType.Solid, lane_marking_color, marking_1),
                        (carla.LaneMarkingType.Broken, lane_marking_color, marking_2)]
            elif lane_marking_type == carla.LaneMarkingType.BrokenBroken:
                return [(carla.LaneMarkingType.Broken, lane_marking_color, marking_1),
                        (carla.LaneMarkingType.Broken, lane_marking_color, marking_2)]
            elif lane_marking_type == carla.LaneMarkingType.SolidSolid:
                return [(carla.LaneMarkingType.Solid, lane_marking_color, marking_1),
                        (carla.LaneMarkingType.Solid, lane_marking_color, marking_2)]

        return [(carla.LaneMarkingType.NONE, carla.LaneMarkingColor.Other, [])]

    def draw_lane_marking(self, waypoints):
        """Draws the left and right side of lane markings"""
        # Left Side
        self.draw_lane_marking_single_side(waypoints[0], -1)

        # Right Side
        self.draw_lane_marking_single_side(waypoints[1], 1)

    def draw_lane_marking_single_side(self, waypoints, sign):
        lane_marking = None

        marking_type = carla.LaneMarkingType.NONE
        previous_marking_type = carla.LaneMarkingType.NONE

        marking_color = carla.LaneMarkingColor.Other
        previous_marking_color = carla.LaneMarkingColor.Other

        markings_list = []
        temp_waypoints = []
        current_lane_marking = carla.LaneMarkingType.NONE
        for sample in waypoints:
            lane_marking = sample.left_lane_marking if sign < 0 else sample.right_lane_marking

            if lane_marking is None:
                continue

            marking_type = lane_marking.type
            marking_color = lane_marking.color

            if current_lane_marking != marking_type:
                # Get the list of lane markings to draw
                markings = self.get_lane_markings(
                    previous_marking_type,
                    previous_marking_color,
                    temp_waypoints,
                    sign)
                current_lane_marking = marking_type

                # Append each lane marking in the list
                for marking in markings:
                    markings_list.append(marking)

                temp_waypoints = temp_waypoints[-1:]

            else:
                temp_waypoints.append((sample))
                previous_marking_type = marking_type
                previous_marking_color = marking_color

        # Add last marking
        last_markings = self.get_lane_markings(
            previous_marking_type,
            previous_marking_color,
            temp_waypoints,
            sign)
        for marking in last_markings:
            markings_list.append(marking)

        # Once the lane markings have been simplified to Solid or Broken lines, we draw them
        for markings in markings_list:
            if markings[0] == carla.LaneMarkingType.Solid:
                self.draw_solid_line(markings[1], False, markings[2], 2)
            elif markings[0] == carla.LaneMarkingType.Broken:
                self.draw_broken_line(markings[1], False, markings[2], 2)

    def draw_solid_line(self, lane_marking_color, closed, points, width):
        if len(points) >= 2:
            color = self.lane_marking_color_to_ros(lane_marking_color)
            solid_line_marker = self.create_line_marker(color, points)
            solid_line_marker.scale.x = width / 10.0
            self.marker_array.markers.append(solid_line_marker)

    def draw_broken_line(self, lane_marking_color, closed, points, width):
        if len(points) >= 2:
            color = self.lane_marking_color_to_ros(lane_marking_color)
            broken_line_marker = self.create_line_marker(color, points)
            broken_line_marker.scale.x = width / 25.0
            broken_line_marker.type = Marker.LINE_LIST
            self.marker_array.markers.append(broken_line_marker)

    def create_line_marker(self, color=None, points=None):
        marker = Marker()
        marker.id = self.set_marker_id()
        marker.type = Marker.LINE_STRIP
        marker.header.frame_id = "map"
        if color is None:
            marker.color = ColorRGBA(1, 1, 1, 1)
        else:
            marker.color = color
        marker.scale.x = 0.2
        marker.pose.orientation.w = 1

        if points is not None:
            for p in points:
                point = Point()
                point.x = p.x
                point.y = -p.y
                point.z = p.z
                marker.points.append(point)

        return marker

    def lane_marking_color_to_ros(self, lane_marking_color):
        color = ColorRGBA(0, 0, 0, 1)
        if lane_marking_color == carla.LaneMarkingColor.White:
            color = COLOR_ALUMINIUM_2

        elif lane_marking_color == carla.LaneMarkingColor.Blue:
            color = COLOR_SKY_BLUE_0

        elif lane_marking_color == carla.LaneMarkingColor.Green:
            color = COLOR_CHAMELEON_0

        elif lane_marking_color == carla.LaneMarkingColor.Red:
            color = COLOR_SCARLET_RED_0

        elif lane_marking_color == carla.LaneMarkingColor.Yellow:
            color = COLOR_ORANGE_0

        return color

    def draw_roads(self):
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
            set_waypoints.append(waypoints)

        for waypoints in set_waypoints:
            waypoint = waypoints[0]
            # waypoints_junction = [x for x in waypoints if x.is_junction]
            road_left_side = [self.lateral_shift(w.transform, -w.lane_width * 0.5) for w in waypoints]
            road_right_side = [self.lateral_shift(w.transform, w.lane_width * 0.5) for w in waypoints]
            road_points = road_left_side + [x for x in reversed(road_right_side)]
            self.draw_line_strip(road_points)
            # self.draw_lane_marking([waypoints, waypoints])

            if not waypoint.is_junction:
                for n, wp in enumerate(waypoints):
                    if ((n + 1) % 400) == 0:
                        self.draw_arrow(wp.transform)

        self.map_viz_publisher.publish(self.marker_array)
