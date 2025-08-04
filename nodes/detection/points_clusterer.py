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
        self.points_pub = rospy.Publisher('points_clustered', PointCloud2, queue_size=1, tcp_nodelay=True)

        # Subscribers
        rospy.Subscriber('points_filtered', PointCloud2, self.points_callback, queue_size=1, buff_size=2 ** 24,
                         tcp_nodelay=True)

    def points_callback(self, msg):
        data = numpify(msg)
        points = structured_to_unstructured(data[['x', 'y', 'z']], dtype=np.float32)
        labels = self.clusterer.fit_predict(points)

        # print('points shape:', points.shape)
        # print('labels shape:', labels.shape)

        assert points.shape[0] == labels.shape[0], 'points and labels not same count'

        points_labeled = np.hstack((points, labels.reshape(-1, 1)))
        mask = labels != -1
        points_labeled = points_labeled[mask]

        # convert labelled points to PointCloud2 format
        data = unstructured_to_structured(points_labeled, dtype=np.dtype([
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('label', np.int32)
        ]))

        # publish clustered points message
        cluster_msg = msgify(PointCloud2, data)
        cluster_msg.header = msg.header
        # self.points_pub.publish(cluster_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('points_clusterer')
    node = PointsClusterer()
    node.run()