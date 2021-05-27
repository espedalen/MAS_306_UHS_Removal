#!/usr/bin/env python
import rospy
import rosnode
import dynamic_reconfigure.client
import pcl
from zivid_camera.srv import *
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2, PointCloud
from sensor_msgs import point_cloud2 as pc2
# from sensor_msgs.msg import PointCloud
import pcl_ros
import os
import tf
import tf.transformations as tft
import time
from geometry_msgs.msg import Point
import numpy as np
import matplotlib.pyplot as plt
import pcl_helper
import config


def rot_mat():
    tf_list = tf.TransformListener()

    tf_list.waitForTransform('/world', '/zivid_optical_frame', rospy.Time(0), rospy.Duration(4.0))
    (trans, quats) = tf_list.lookupTransform('/world', '/zivid_optical_frame', rospy.Time(0))
    R = tft.quaternion_matrix(quats)
    T = tft.translation_matrix(trans)
    TR = R + T - np.identity(4)
    return TR


if __name__ == '__main__':
    rospy.init_node("for_bachelor", anonymous=True)
    rotasjon = rot_mat()
    print(rotasjon)
