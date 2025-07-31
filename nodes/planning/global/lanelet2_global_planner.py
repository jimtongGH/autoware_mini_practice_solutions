#!/usr/bin/env python3
import math

# All these imports from lanelet2 library should be sufficient
import shapely
import numpy as np
import rospy
import lanelet2
import lanelet2.traffic_rules
import lanelet2.routing
from lanelet2.io import Origin, load
from lanelet2.projection import UtmProjector
from lanelet2.core import BasicPoint2d
from lanelet2.geometry import findNearest

from autoware_mini.msg import Waypoint, Path, VehicleCmd
from geometry_msgs.msg import PoseStamped

class Lanelet2GlobalPlanner:
    def __init__(self):

        # get parameters
        self.graph = None
        self.current_location = None
        self.goal_point = None
        self.coordinate_transformer = rospy.get_param("/localization/coordinate_transformer")
        self.use_custom_origin = rospy.get_param("/localization/use_custom_origin")
        self.utm_origin_lat = rospy.get_param("/localization/utm_origin_lat")
        self.utm_origin_lon = rospy.get_param("/localization/utm_origin_lon")
        self.lanelet2_map_path = rospy.get_param("~lanelet2_map_path")
        self.speed_limit = rospy.get_param("~speed_limit", 40.0)
        self.output_frame = rospy.get_param("/planning/waypoint_loader/output_frame")
        self.distance_to_goal_limit = rospy.get_param("/planning/lanelet2_global_planner/distance_to_goal_limit")

        # Publishers
        self.waypoints_pub = rospy.Publisher('/planning/global_path', Path, queue_size=1, latch=True)
        self.vehicle_cmd_pub = rospy.Publisher('/control/vehicle_cmd', VehicleCmd, queue_size=10)

        # Subscribers
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_point_callback, queue_size=1)
        rospy.Subscriber('/localization/current_pose', PoseStamped, self.current_pose_callback, queue_size=10)

    def goal_point_callback(self, msg):
        self.goal_point = BasicPoint2d(msg.pose.position.x, msg.pose.position.y)
        # loginfo message about receiving the goal point
        rospy.loginfo("%s - goal position (%f, %f, %f) orientation (%f, %f, %f, %f) in %s frame", rospy.get_name(),
                      msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
                      msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z,
                      msg.pose.orientation.w, msg.header.frame_id)

        # Load the map using Lanelet2
        if self.coordinate_transformer == "utm":
            projector = UtmProjector(Origin(self.utm_origin_lat, self.utm_origin_lon), self.use_custom_origin, False)
        else:
            raise ValueError(
                'Unknown coordinate_transformer for loading the Lanelet2 map ("utm" should be used): ' + self.coordinate_transformer)

        self.lanelet2_map = load(self.lanelet2_map_path, projector)

        # traffic rules
        traffic_rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                                      lanelet2.traffic_rules.Participants.VehicleTaxi)
        # routing graph
        self.graph = lanelet2.routing.RoutingGraph(self.lanelet2_map, traffic_rules)

        # get start and end lanelets
        start_lanelet = findNearest(self.lanelet2_map.laneletLayer, self.current_location, 1)[0][1]
        goal_lanelet = findNearest(self.lanelet2_map.laneletLayer, self.goal_point, 1)[0][1]
        # find routing graph
        route = self.graph.getRoute(start_lanelet, goal_lanelet, 0, True)

        if route is None:
            rospy.logwarn("%s - No route found", rospy.get_name())
            return

        # find shortest path
        path = route.shortestPath()
        # This returns LaneletSequence to a point where a lane change would be necessary to continue
        path_no_lane_change = path.getRemainingLane(start_lanelet)

        self.publish_path(path_no_lane_change)

    def publish_path(self, lanelet_sequence):
        waypoints = []
        for i, lanelet in enumerate(lanelet_sequence):
            if 'speed_ref' in lanelet.attributes:
                speed_kph = float(lanelet.attributes['speed_ref'])
                speed = min(speed_kph / 3.6 , self.speed_limit)
            else:
                speed = self.speed_limit

            for j, point in enumerate(lanelet.centerline):
                if i > 0 and j == 0:
                   continue
                waypoint = Waypoint()
                waypoint.position.x = point.x
                waypoint.position.y = point.y
                waypoint.position.z = point.z
                waypoint.speed = speed
                waypoints.append(waypoint)

        if waypoints:
            waypoint_end = waypoints[-1]
            waypoint_end.position.x = self.goal_point.x
            waypoint_end.position.y = self.goal_point.y

        path = Path()
        path.header.frame_id = self.output_frame
        path.header.stamp = rospy.Time.now()
        path.waypoints = waypoints
        self.waypoints_pub.publish(path)

    def empty_publish_path(self):
        # stop the vehicle
        vehicle_cmd = VehicleCmd()
        vehicle_cmd.ctrl_cmd.steering_angle = 0.0
        vehicle_cmd.ctrl_cmd.linear_velocity = 0.0
        self.vehicle_cmd_pub.publish(vehicle_cmd)

        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = self.output_frame
        path.waypoints = []
        self.waypoints_pub.publish(path)



    def current_pose_callback(self, msg):
        self.current_location = BasicPoint2d(msg.pose.position.x, msg.pose.position.y)
        if self.goal_point:
            distance = math.hypot(self.goal_point.x - self.current_location.x,
                                  self.goal_point.y - self.current_location.y)
            if distance < self.distance_to_goal_limit:

                self.empty_publish_path()
                rospy.loginfo("%s - goal reached, clear path", rospy.get_name())




    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('lanelet2_global_planner')
    node = Lanelet2GlobalPlanner()
    node.run()