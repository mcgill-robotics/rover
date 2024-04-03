#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg._ModelStates import ModelStates
from geometry_msgs.msg import Pose, PolygonStamped, Polygon, Point32, PoseWithCovariance, TwistWithCovariance
from visualization_msgs.msg import Marker
# from autonomy_config import FIXED_FRAME_ID, CAMERA_POSITION_OFFSET, ROUNDING_COEF, ROVER_MODEL_NAME, PointsFilters
import numpy as np
from typing import Set, Tuple, List
from scipy.spatial import ConvexHull
# from bounding_boxes import get_all_hulls_vertices
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg 
from nav_msgs.msg import Odometry


def publish_odo_msg():
    pub = rospy.Publisher('/test_optim_node/odom', Odometry, queue_size=1)
    rospy.init_node("odometry_node")

    odo_msg = Odometry()
    odo_msg.header.stamp = rospy.Time.now()
    odo_msg.child_frame_id = "rover"

    current_pos = PoseWithCovariance()

    current_pos.pose.position.x = 1.0
    current_pos.pose.position.y = 2.0
    current_pos.pose.position.z = 0.0

    # Set orientation
    current_pos.pose.orientation.x = 0.0
    current_pos.pose.orientation.y = 0.0
    current_pos.pose.orientation.z = 0.0
    current_pos.pose.orientation.w = 1.0

    # Set covariance values
    # This is a 6x6 covariance matrix, but typically only the diagonal elements are used
    current_pos.covariance = [
        0.1, 0, 0, 0, 0, 0,
        0, 0.1, 0, 0, 0, 0,
        0, 0, 0.1, 0, 0, 0,
        0, 0, 0, 0.1, 0, 0,
        0, 0, 0, 0, 0.1, 0,
        0, 0, 0, 0, 0, 0.1
        ]

    odo_msg.pose = current_pos


    velo = TwistWithCovariance()
    velo.twist.linear.x = 1.0
    velo.twist.linear.y = 1.0
    velo.twist.linear.z = 0.0

    velo.twist.angular.x = 0
    velo.twist.angular.y = 0
    velo.twist.angular.z = 0

    velo.covariance = [
        0.1, 0, 0, 0, 0, 0,
        0, 0.1, 0, 0, 0, 0,
        0, 0, 0.1, 0, 0, 0,
        0, 0, 0, 0.1, 0, 0,
        0, 0, 0, 0, 0.1, 0,
        0, 0, 0, 0, 0, 0.1
        ]

    odo_msg.twist = velo
    

    # odo_msg.twist = velo




    r = rospy.Rate(5) # 10hz
    t = 0.0
    while not rospy.is_shutdown():
            
        pub.publish(odo_msg)
        
        r.sleep()


if __name__ == '__main__': 
    try:
        publish_odo_msg()
    except rospy.ROSInterruptException:
        pass
