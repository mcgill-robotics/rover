#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pc2
from gazebo_msgs.msg._ModelStates import ModelStates
from geometry_msgs.msg import Pose, PolygonStamped, Polygon, Point32
from visualization_msgs.msg import Marker
from autonomy_config import FIXED_FRAME_ID, CAMERA_POSITION_OFFSET, ROUNDING_COEF, ROVER_MODEL_NAME, PointsFilters
import numpy as np
from typing import Set, Tuple, List
from scipy.spatial import ConvexHull
from bounding_boxes import get_all_hulls_vertices
import json
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg 

class PointCloudTracker:
    def __init__(self) -> None:
        self.obstacle_points: Set[Tuple[float, float, float]] = set() 
        self.ground_points: Set[Tuple[float, float, float]] = set()
        self.convex_hulls: List[ConvexHull] = []
        self.rviz_pc2_pub = rospy.Publisher('parsed_point_cloud', PointCloud2, queue_size=10)
        self.rviz_marker_pub = rospy.Publisher('rover_position', Marker, queue_size=10)
        self.obstacle_pub = rospy.Publisher('obstacles', ObstacleArrayMsg)
        self.rviz_polygon_pub_lst = {}  #Dictionary Containing n publishers
        self.map_grid = {}
    def listener(self) -> None:
        rospy.init_node('pc2_publisher_and_listener', anonymous=True)
        rospy.Subscriber("camera/depth/points", PointCloud2, self.parse_pointcloud2_message)
        rospy.spin()

    def update_json(self):
        location = self.get_rover_pose()[0] # Rover coordinates
        location_output = (round(location[0], 1), round(location[1], 1))
        print("Updating Map")
        with open ('prm/obstacles.json', "w") as jsonfile:
            json.dump({"map":self.map_grid, "location": location_output}, jsonfile)        

    def get_rover_pose(self) -> tuple:
        data = rospy.wait_for_message("/gazebo/model_states", ModelStates)
        model_names, model_poses = data.name, data.pose
        if ROVER_MODEL_NAME not in model_names:
            raise Exception(f"Rover model name not found: '{ROVER_MODEL_NAME}'")
        rover_pose_quat = model_poses[model_names.index(ROVER_MODEL_NAME)].orientation 
        rover_pose_obj = model_poses[model_names.index(ROVER_MODEL_NAME)].position
        return (
            rover_pose_obj.x, rover_pose_obj.y, rover_pose_obj.z
        ), (rover_pose_quat.x, rover_pose_quat.y, rover_pose_quat.z, rover_pose_quat.w)
        
    def apply_camera_pose_transform(self, points_tuple: tuple, rover_position_tuple: tuple, camera_orientation_tuple: tuple) -> np.array:

        Drw = np.reshape(np.array(rover_position_tuple), (3, 1))

        Rrw = self.quaternion_rotation_matrix(camera_orientation_tuple)

        Trw = np.vstack((
            np.hstack((Rrw, Drw)), 
            np.hstack((np.zeros((1, 3)), np.array([[1]])))
        ))

        Rcr = np.array(
            [[0 ,0 ,1],
            [0 ,1 ,0],
            [-1 ,0 ,0]]
        ) @ np.array(
            [[0 ,1 ,0],
            [-1,0 ,0],
            [0 ,0 ,1]]
        )

        Dcr = np.reshape(CAMERA_POSITION_OFFSET, (3, 1))

        Tcr = np.vstack((
            np.hstack((Rcr, Dcr)),
            np.hstack((np.zeros((1, 3)), np.array([[1]])))
        ))
        
        P = np.vstack((
            points_tuple.T,
            np.ones((points_tuple.shape[0],))
        ))

        return Trw @ Tcr @ P

    def parse_pointcloud2_message(self, msg: PointCloud2) -> None:
        print(f'{len(self.obstacle_points)=}')
        rover_position_tuple, rover_orientation_tuple = self.get_rover_pose()
        points_np = np.array(list(pc2.read_points(msg, field_names=['x', 'y', 'z'], skip_nans=True)))
        points_transformed_np = np.round(self.apply_camera_pose_transform(points_np, rover_position_tuple, rover_orientation_tuple).T, decimals=ROUNDING_COEF)
        
        new_obstacle_points = points_transformed_np[
            (points_transformed_np[:, 2] < PointsFilters.GROUND_LOWER_LIMIT) |
            (points_transformed_np[:, 2] > PointsFilters.GROUND_UPPER_LIMIT)
        ]

        # ground_points = points_transformed_np[
        #     (points_transformed_np[:, 2] >= PointsFilters.GROUND_LOWER_LIMIT) &
        #     (points_transformed_np[:, 2] <= PointsFilters.GROUND_UPPER_LIMIT)
        # ]

        for p in new_obstacle_points:
            point = str(f"{round(p[0], 1)} {round(p[1], 1)}")
            if point not in self.map_grid:
                self.map_grid[point] = True
            self.obstacle_points.add(tuple((p[0], p[1], p[2])))
        self.update_json()
        
        self.publish_rover_position_to_rviz(rover_position_tuple)
        if (len(self.obstacle_points) > 0):
            self.publish_pc2_to_rviz()
            self.publish_bounding_boxes_to_rviz()

    def publish_pc2_to_rviz(self) -> None:
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = FIXED_FRAME_ID
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
          PointField('y', 4, PointField.FLOAT32, 1),
          PointField('z', 8, PointField.FLOAT32, 1)]
        points = np.array(list(self.obstacle_points), dtype=np.float32) 
        data = points.tobytes()
        cloud_msg = PointCloud2(
            header=header,
            height=1,
            width = points.shape[0],
            fields=fields,
            is_dense=False,
            is_bigendian=False,
            point_step=12,
            row_step=12 * points.shape[0],
            data=data)
        self.rviz_pc2_pub.publish(cloud_msg)

    def publish_rover_position_to_rviz(self, camera_pose_tuple: tuple) -> None:
        marker = Marker()
        marker.header.frame_id = FIXED_FRAME_ID
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = camera_pose_tuple[0]
        marker.pose.position.y = camera_pose_tuple[1]
        marker.pose.position.z = camera_pose_tuple[2]
        self.rviz_marker_pub.publish(marker)
        # print(f'Published {tuple(round(n, 2) for n in camera_pose_tuple)} ')

    def publish_bounding_boxes_to_rviz(self) -> None:
        X = np.array([(t[0], t[1]) for t in self.obstacle_points])
        obstacle_msg = ObstacleArrayMsg() # This and 2 lines under creates an Obstacle array
        obstacle_msg.header.stamp = rospy.Time.now()
        obstacle_msg.header.frame_id = FIXED_FRAME_ID
        for e, hull in enumerate(get_all_hulls_vertices(X)):
            individual_obstacle = ObstacleMsg()
            individual_obstacle.id = e
            individual_obstacle.polygon.points = [Point32(x=x, y=y) for x, y in hull]
            obstacle_msg.obstacles.append(individual_obstacle)
            # Setting up the polygonStamped
            polygon_stamped_msg = PolygonStamped()
            polygon_stamped_msg.header.stamp = rospy.Time.now()
            polygon_stamped_msg.header.frame_id = FIXED_FRAME_ID
            polygon_stamped_msg.polygon.points = [Point32(x=x, y=y, z=0.5) for x, y in hull]
            polygon_stamped_msg.header.seq = e
            '''---------------------------------'''
            

            if e+1 > len(self.rviz_polygon_pub_lst):
                for _ in range(e+1-len(self.rviz_polygon_pub_lst)):
                    self.rviz_polygon_pub_lst[e] = rospy.Publisher(f'obstacle_polygons/{e}', PolygonStamped, queue_size=10)
            self.rviz_polygon_pub_lst[e].publish(polygon_stamped_msg)
            print(f"Polygon count: {len(self.rviz_polygon_pub_lst)}")
            '''
            else:
                print(f'WARNING: {e=} is out of range of {len(self.rviz_polygon_pub_lst)=}')'''
        self.obstacle_pub.publish(obstacle_msg)

    def quaternion_rotation_matrix(self, Q: tuple):
        """
        Covert a quaternion into a full three-dimensional rotation matrix.
    
        Input
        :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3) 
    
        Output
        :return: A 3x3 element matrix representing the full 3D rotation matrix. 
                This rotation matrix converts a point in the local reference 
                frame to a point in the global reference frame.
        """
        # Extract the values from Q
        q0 = Q[3]
        q1 = Q[0]
        q2 = Q[1]
        q3 = Q[2]
        
        # First row of the rotation matrix
        r00 = 2 * (q0 * q0 + q1 * q1) - 1
        r01 = 2 * (q1 * q2 - q0 * q3)
        r02 = 2 * (q1 * q3 + q0 * q2)
        
        # Second row of the rotation matrix
        r10 = 2 * (q1 * q2 + q0 * q3)
        r11 = 2 * (q0 * q0 + q2 * q2) - 1
        r12 = 2 * (q2 * q3 - q0 * q1)
        
        # Third row of the rotation matrix
        r20 = 2 * (q1 * q3 - q0 * q2)
        r21 = 2 * (q2 * q3 + q0 * q1)
        r22 = 2 * (q0 * q0 + q3 * q3) - 1
        
        # 3x3 rotation matrix
        rot_matrix = np.array([[r00, r01, r02],
                            [r10, r11, r12],
                            [r20, r21, r22]])
                                
        return rot_matrix

if __name__ == '__main__':
    trcker = PointCloudTracker()
    trcker.listener()