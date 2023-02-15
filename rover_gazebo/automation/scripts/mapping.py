import rospy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pc2
from gazebo_msgs.msg._ModelStates import ModelStates
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker
import numpy as np

class PointCloudTracker:
    def __init__(self) -> None:
        self.existing_points = set() 
        self.fixed_frame_id = 'parsed_point_cloud_world'
        self.camera_height_offset = 1.2
        self.rounding_coef = 5
        self.rviz_pc2_pub = rospy.Publisher('parsed_point_cloud', PointCloud2, queue_size=10)
        self.rviz_marker_pub = rospy.Publisher('rover_position', Marker, queue_size=10)

    def listener(self) -> None:
        rospy.init_node('pc2_publisher_and_listener', anonymous=True)
        rospy.Subscriber("camera/depth/points", PointCloud2, self.parse_pointcloud2_message)
        rospy.spin()

    def get_camera_pose(self) -> tuple:
        data = rospy.wait_for_message("/gazebo/model_states", ModelStates)
        rover_model_name = '/' 
        model_names = data.name
        model_poses = data.pose
        if rover_model_name not in model_names:
            raise Exception(f"Rover model name not found: '{rover_model_name}'")
        rover_pose_quat = model_poses[model_names.index(rover_model_name)].orientation 
        rover_pose_obj = model_poses[model_names.index(rover_model_name)].position
        return (
            rover_pose_obj.x, rover_pose_obj.y, rover_pose_obj.z + self.camera_height_offset
        ), (rover_pose_quat.x, rover_pose_quat.y, rover_pose_quat.z, rover_pose_quat.w)
        
    def apply_camera_pose_transform(self, points_tuple: tuple, camera_position_tuple: tuple, camera_orientation_tuple: tuple) -> np.array:
        Rxy = np.array(
            [[0 ,-1 ,0],
             [1,0 ,0],
             [0 ,0 ,1]]) 

        Rxz = np.array(
            [[0 ,0 ,-1],
             [0 ,1 ,0],
             [1 ,0 ,0]]
        )  
        return (points_tuple @ (Rxy @ Rxz)) @ self.quaternion_rotation_matrix(camera_orientation_tuple)

    def parse_pointcloud2_message(self, msg: PointCloud2) -> None:
        print(f'{len(self.existing_points)=}')
        camera_position_tuple, camera_orientation_tuple = self.get_camera_pose()
        points = pc2.read_points(msg, field_names=['x', 'y', 'z'], skip_nans=True)
        points_np = np.array(list(points))
        points_transformed_np = self.apply_camera_pose_transform(points_np, camera_position_tuple, camera_orientation_tuple)
        for p in points_transformed_np:
            p_xyz_rounded = (
                round(p[0] + camera_position_tuple[0], self.rounding_coef),
                round(p[1] + camera_position_tuple[1], self.rounding_coef),
                round(p[2] + camera_position_tuple[2], self.rounding_coef)
            )
            self.existing_points.add(p_xyz_rounded)
        self.publish_rover_position_to_rviz(camera_position_tuple)
        self.publish_pc2_to_rviz()

    def publish_pc2_to_rviz(self) -> None:
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = self.fixed_frame_id
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
          PointField('y', 4, PointField.FLOAT32, 1),
          PointField('z', 8, PointField.FLOAT32, 1)]
        points = np.array(list(self.existing_points), dtype=np.float32) 
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
        marker.header.frame_id = self.fixed_frame_id
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
        q3 = -Q[2]
        
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