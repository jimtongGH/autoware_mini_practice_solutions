#!/usr/bin/env python3

import rospy
import shapely
import math
import numpy as np
import threading
from ros_numpy import msgify
from autoware_mini.msg import Path, DetectedObjectArray
from sensor_msgs.msg import PointCloud2

DTYPE = np.dtype([
    ('x', np.float32),
    ('y', np.float32),
    ('z', np.float32),
    ('vx', np.float32),
    ('vy', np.float32),
    ('vz', np.float32),
    ('distance_to_stop', np.float32),
    ('deceleration_limit', np.float32),
    ('category', np.int32)
])

class CollisionPointsManager:

    def __init__(self):

        # parameters
        self.safety_box_width = rospy.get_param("safety_box_width")
        self.stopped_speed_limit = rospy.get_param("stopped_speed_limit")
        self.braking_safety_distance_obstacle = rospy.get_param("~braking_safety_distance_obstacle")
        self.braking_safety_distance_goal = rospy.get_param("~braking_safety_distance_goal")

        # variables
        self.detected_objects = None
        self.goal_waypoint = None

        # Lock for thread safety
        self.lock = threading.Lock()

        # publishers
        self.local_path_collision_pub = rospy.Publisher('collision_points', PointCloud2, queue_size=1, tcp_nodelay=True)

        # subscribers
        rospy.Subscriber('extracted_local_path', Path, self.path_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/detection/final_objects', DetectedObjectArray, self.detected_objects_callback, queue_size=1, buff_size=2**20, tcp_nodelay=True)
        rospy.Subscriber('extracted_global_path', Path, self.global_path_callback, queue_size=1, tcp_nodelay=True)

    def detected_objects_callback(self, msg):
        self.detected_objects = msg.objects

    def global_path_callback(self, msg):
        if len(msg.waypoints) == 0:
            print('No waypoints')
            return

        with self.lock:
            self.goal_waypoint = msg.waypoints[-1]

    def path_callback(self, msg):
        with self.lock:
            detected_objects = self.detected_objects
        collision_points = np.array([], dtype=DTYPE)

        if len(msg.waypoints) > 0:

            if not msg.waypoints:
                collision_points_msg = msgify(PointCloud2, collision_points)
                collision_points_msg.header = msg.header
                self.local_path_collision_pub.publish(collision_points_msg)
                return

            if detected_objects is None or len(detected_objects) == 0:
                collision_points_msg = msgify(PointCloud2, collision_points)
                collision_points_msg.header = msg.header
                self.local_path_collision_pub.publish(collision_points_msg)
                return

            path_coordinates = [(waypoint.position.x, waypoint.position.y) for waypoint in msg.waypoints]
            local_path_linestring = shapely.LineString(path_coordinates)

            local_path_buffer = local_path_linestring.buffer(self.safety_box_width / 2, cap_style='flat')
            shapely.prepare(local_path_buffer)

            for obj in detected_objects:
                if hasattr(obj, 'convex_hull') and len(obj.convex_hull) >= 6:
                    hull_points = []
                    for i in range(0, len(obj.convex_hull), 3):
                        if i + 1 < len(obj.convex_hull):
                            hull_points.append((obj.convex_hull[i], obj.convex_hull[i + 1]))

                    if len(hull_points) >= 3:
                        object_polygon = shapely.Polygon(hull_points)

                        if local_path_buffer.intersects(object_polygon):
                            intersection = local_path_buffer.intersection(object_polygon)

                            intersection_points = shapely.get_coordinates(intersection)

                            object_speed = math.sqrt(obj.velocity.x ** 2 + obj.velocity.y ** 2)

                            for x, y in intersection_points:
                                collision_points = np.append(collision_points, np.array(
                                    [(x, y, obj.centroid.z, obj.velocity.x, obj.velocity.y, obj.velocity.z,
                                      self.braking_safety_distance_obstacle, np.inf,
                                      3 if object_speed < self.stopped_speed_limit else 4)], dtype=DTYPE))

            # print("Collision points found:", len(collision_points))
            if self.goal_waypoint is not None:
                goal_point = shapely.Point(self.goal_waypoint.position.x, self.goal_waypoint.position.y)
                goal_buffer = goal_point.buffer(0.5)
                if goal_buffer.intersects(local_path_buffer):
                    collision_points = np.append(collision_points, np.array(
                        [(goal_point.x, goal_point.y, self.goal_waypoint.position.z,
                          0.0, 0.0, 0.0,
                          self.braking_safety_distance_goal, np.inf,
                          1)], dtype=DTYPE))


            collision_points_msg = msgify(PointCloud2, collision_points)
            collision_points_msg.header = msg.header
            self.local_path_collision_pub.publish(collision_points_msg)
            return

        empty_msg = PointCloud2()
        empty_msg.header = msg.header
        self.local_path_collision_pub.publish(empty_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('collision_points_manager')
    node = CollisionPointsManager()
    node.run()