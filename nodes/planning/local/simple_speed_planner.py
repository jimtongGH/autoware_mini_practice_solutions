#!/usr/bin/env python3

import rospy
import math
import message_filters
import traceback
import shapely
import numpy as np
import threading
from numpy.lib.recfunctions import structured_to_unstructured
from ros_numpy import numpify
from autoware_mini.msg import Path, Log
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3
from autoware_mini.geometry import project_vector_to_heading, get_distance_between_two_points_2d
from shapely.geometry import LineString, Point


class SpeedPlanner:

    def __init__(self):

        # parameters
        self.default_deceleration = rospy.get_param("default_deceleration")
        self.braking_reaction_time = rospy.get_param("braking_reaction_time")
        synchronization_queue_size = rospy.get_param("~synchronization_queue_size")
        synchronization_slop = rospy.get_param("~synchronization_slop")
        self.distance_to_car_front = rospy.get_param("distance_to_car_front")

        # variables
        self.collision_points = None
        self.current_position = None
        self.current_speed = None

        # Lock for thread safety
        self.lock = threading.Lock()

        # publishers
        self.local_path_pub = rospy.Publisher('local_path', Path, queue_size=1, tcp_nodelay=True)

        # subscribers
        rospy.Subscriber('/localization/current_pose', PoseStamped, self.current_pose_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/localization/current_velocity', TwistStamped, self.current_velocity_callback, queue_size=1, tcp_nodelay=True)

        collision_points_sub = message_filters.Subscriber('collision_points', PointCloud2, tcp_nodelay=True)
        local_path_sub = message_filters.Subscriber('extracted_local_path', Path, tcp_nodelay=True)

        ts = message_filters.ApproximateTimeSynchronizer([collision_points_sub, local_path_sub], queue_size=synchronization_queue_size, slop=synchronization_slop)

        ts.registerCallback(self.collision_points_and_path_callback)

    def current_velocity_callback(self, msg):
        self.current_speed = msg.twist.linear.x

    def current_pose_callback(self, msg):
        self.current_position = shapely.Point(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

    def collision_points_and_path_callback(self, collision_points_msg, local_path_msg):
        try:
            with self.lock:
                collision_points = numpify(collision_points_msg) if len(collision_points_msg.data) > 0 else np.array([])
                current_position = self.current_position
                current_speed = self.current_speed

            if current_speed is None or current_position is None:
                rospy.logwarn("%s - There is No current_speed or current_position here!", rospy.get_name())
                return

            # no local path or no collision points
            if  len(local_path_msg.waypoints) == 0 or len(collision_points) == 0:
                self.local_path_pub.publish(local_path_msg)
                return
            

            local_path_coordination = [(wp.position.x, wp.position.y) for wp in local_path_msg.waypoints]
            local_path_linestring = LineString(local_path_coordination)

            # min_target_velocity = float('inf')
            # closest_object_distance = float('inf')
            # collision_point_category = 0

            collision_point_distances = []
            collision_point_velocities = []
            collision_point_categories = []

            for collision_point in collision_points:
                collision_point_position = Point(collision_point['x'], collision_point['y'])
                distance_to_cp = local_path_linestring.project(collision_point_position)

                corrected_distance_to_cp = distance_to_cp - self.distance_to_car_front - collision_point['distance_to_stop']

                heading = self.get_heading_at_distance(local_path_linestring, distance_to_cp)
                obj_velocity_vector = Vector3(x=collision_point['vx'], y=collision_point['vy'], z=collision_point['vz'])
                obj_speed = np.linalg.norm([collision_point['vx'], collision_point['vy'], collision_point['vz']])
                rel_speed = self.project_vector_to_heading(heading, obj_velocity_vector)

                rospy.loginfo("Object's actual speed: %.3f m/s, relative speed: %.3f m/s", obj_speed, rel_speed)

                collision_point_distances.append(corrected_distance_to_cp)
                collision_point_velocities.append(rel_speed)
                collision_point_categories.append(collision_point['category'])
            
            collision_point_distances = np.array(collision_point_distances)
            collision_point_velocities = np.array(collision_point_velocities)
            collision_point_categories = np.array(collision_point_categories)

            # the distance
            target_distances = collision_point_distances - self.braking_reaction_time * np.abs(collision_point_velocities)
            # keep choosing max value
            target_distances = np.maximum(0, target_distances)

            positive_velocities = np.maximum(0, collision_point_velocities)

            # the target_velocity
            target_velocities = np.sqrt(np.square(positive_velocities) + 2 * self.default_deceleration * target_distances)

            min_velocity_index = np.argmin(target_velocities)
            min_target_velocity = target_velocities[min_velocity_index]

            closest_object_distance = collision_point_distances[min_velocity_index] + self.distance_to_car_front
            closest_object_velocity = collision_point_velocities[min_velocity_index]
            collision_point_category = collision_point_categories[min_velocity_index]
            stopping_point_distance = closest_object_distance

            for i, wp in enumerate(local_path_msg.waypoints):
                wp.speed = min(min_target_velocity, wp.speed)

            # Update the lane message with the calculated values
            path = Path()
            path.header = local_path_msg.header
            path.waypoints = local_path_msg.waypoints
            path.closest_object_distance = closest_object_distance
            path.closest_object_velocity = closest_object_velocity
            path.is_blocked = True
            path.stopping_point_distance = stopping_point_distance
            path.collision_point_category = collision_point_category
            self.local_path_pub.publish(path)

        except Exception as e:
            rospy.logerr_throttle(10, "%s - Exception in callback: %s", rospy.get_name(), traceback.format_exc())


    def get_heading_at_distance(self, linestring, distance):
        """
        Get heading of the path at a given distance
        :param distance: distance along the path
        :param linestring: shapely linestring
        :return: heading angle in radians
        """

        point_after_object = linestring.interpolate(distance + 0.1)
        # if distance is negative it is measured from the end of the linestring in reverse direction
        point_before_object = linestring.interpolate(max(0, distance - 0.1))

        # get heading between two points
        return math.atan2(point_after_object.y - point_before_object.y, point_after_object.x - point_before_object.x)


    def project_vector_to_heading(self, heading_angle, vector):
        """
        Project vector to heading
        :param heading_angle: heading angle in radians
        :param vector: vector
        :return: projected vector
        """

        return vector.x * math.cos(heading_angle) + vector.y * math.sin(heading_angle)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('speed_planner')
    node = SpeedPlanner()
    node.run()
