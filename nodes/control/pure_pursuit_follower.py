#!/usr/bin/env python3

import rospy
import numpy as np
import shapely

from autoware_mini.msg import Path, VehicleCmd
from geometry_msgs.msg import PoseStamped
from pygments.lexers.sql import lookahead

from shapely.geometry import LineString, Point
from shapely import prepare, distance
from tf.transformations import euler_from_quaternion
from scipy.interpolate import interp1d

class PurePursuitFollower:
    def __init__(self):

        # Parameters
        self.path_linestring = None
        self.distance_to_velocity_interpolator = None
        self.lookahead_distance = rospy.get_param("~lookahead_distance")
        self.wheel_base = rospy.get_param("/vehicle/wheel_base")
        self.end_track = False

        # Publishers
        self.vehicle_cmd_pub = rospy.Publisher('/control/vehicle_cmd', VehicleCmd, queue_size=10)

        # Subscribers
        rospy.Subscriber('path', Path, self.path_callback, queue_size=1)
        rospy.Subscriber('/localization/current_pose', PoseStamped, self.current_pose_callback, queue_size=1)

    def path_callback(self, msg):
        # convert waypoints to shapely linestring
        path_linestring = LineString([(w.position.x, w.position.y) for w in msg.waypoints])
        # prepare path - creates spatial tree, making the spatial queries more efficient
        prepare(path_linestring)
        self.path_linestring = path_linestring

        self.end_track = False

        if msg.waypoints:
            # Create a distance-to-velocity interpolator for the path
            # collect waypoint x and y coordinates
            waypoints_xy = np.array([(w.position.x, w.position.y) for w in msg.waypoints])
            # Calculate distances between points
            distances = np.cumsum(np.sqrt(np.sum(np.diff(waypoints_xy, axis=0) ** 2, axis=1)))
            # add 0 distance in the beginning
            distances = np.insert(distances, 0, 0)
            # Extract velocity values at waypoints
            velocities = np.array([w.speed for w in msg.waypoints])

            self.distance_to_velocity_interpolator = interp1d(distances, velocities, kind='linear')
            self.distance_to_velocity_interpolator.bounds_error = True
            self.distance_to_velocity_interpolator.fill_value = 0.0
        else:
            self.end_track = True

    def current_pose_callback(self, msg):
        if self.path_linestring is None or self.distance_to_velocity_interpolator is None:
            return

        if self.end_track is False:
            # convert the ego vehicle location to a Shapely Point
            current_pose = Point([msg.pose.position.x, msg.pose.position.y])
            # find the distance using the project function from Shapely
            d_ego_from_path_start = self.path_linestring.project(current_pose)
            lookahead_point = self.path_linestring.interpolate(d_ego_from_path_start + self.lookahead_distance)
            # using euler_from_quaternion to get the heading angle
            _, _, heading = euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
            # lookahead point heading calculation
            lookahead_heading = np.arctan2(lookahead_point.y - current_pose.y, lookahead_point.x - current_pose.x)
            real_lookahead_distance = shapely.distance(current_pose, lookahead_point)
            steering_angle = np.arctan((2 * self.wheel_base * np.sin(lookahead_heading - heading))/real_lookahead_distance)

            vehicle_cmd = VehicleCmd()
            vehicle_cmd.ctrl_cmd.steering_angle = steering_angle
            vehicle_cmd.ctrl_cmd.linear_velocity = self.distance_to_velocity_interpolator(d_ego_from_path_start)
            vehicle_cmd.header.stamp = msg.header.stamp
            vehicle_cmd.header.frame_id = "base_link"
            self.vehicle_cmd_pub.publish(vehicle_cmd)
        else:
            # stop the vehicle
            vehicle_cmd = VehicleCmd()
            vehicle_cmd.ctrl_cmd.steering_angle = 0.0
            vehicle_cmd.ctrl_cmd.linear_velocity = 0.0
            vehicle_cmd.header.stamp = msg.header.stamp
            vehicle_cmd.header.frame_id = "base_link"
            self.vehicle_cmd_pub.publish(vehicle_cmd)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('pure_pursuit_follower')
    node = PurePursuitFollower()
    node.run()