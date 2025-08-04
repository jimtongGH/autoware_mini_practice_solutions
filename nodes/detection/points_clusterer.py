#!/usr/bin/env python3

import rospy
import numpy as np

from autoware_mini.msg import Path, VehicleCmd
from sensor_msgs.msg import PointCloud2
from ros_numpy import numpify, msgify
from numpy.lib.recfunctions import structured_to_unstructured, unstructured_to_structured

from sklearn.cluster import DBSCAN

class PointsClusterer:
    def __init__(self):

        # Parameters
        self.cluster_epsilon = rospy.get_param("~cluster_epsilon")
        self.cluster_min_size = rospy.get_param("~cluster_min_size")
        self.clusterer = DBSCAN(eps=self.cluster_epsilon, min_samples=self.cluster_min_size)

        # Publishers


        # Subscribers
        rospy.Subscriber('points_filtered', PointCloud2, self.points_callback, queue_size=1, buff_size=2 ** 24,
                         tcp_nodelay=True)

    def points_callback(self, msg):
        data = numpify(msg)
        points = structured_to_unstructured(data[['x', 'y', 'z']], dtype=np.float32)
        labels = self.clusterer.fit_predict(points)

        print('points shape:', points.shape)
        print('labels shape:', labels.shape)

        assert points.shape[0] == labels.shape[0], 'points and labels not same count'

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('points_clusterer')
    node = PointsClusterer()
    node.run()