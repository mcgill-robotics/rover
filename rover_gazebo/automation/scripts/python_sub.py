import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
"""
Point cloud messages contain x, y, z, and RBG coordinates.
The camera will map every pixel to whichever object is in the FOV and corresponds to said pixel.


"""
def parse_pointcloud2_message(msg):
    points = pc2.read_points_list(msg, skip_nans=True)
    print([points[:3]])
    
def listener():
    rclpy.init()
    node = rclpy.create_node('listener', anonymous=True)
    node.create_subscription(PointCloud2, "camera/depth/points", parse_pointcloud2_message)
    rospy.spin()

if __name__ == '__main__':
    listener()