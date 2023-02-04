import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2


def parse_pointcloud2_message(msg):
    points = pc2.read_points_list(msg, skip_nans=True)
    print([points[:3]])
    
def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("camera/depth/points", PointCloud2, parse_pointcloud2_message)
    rospy.spin()

if __name__ == '__main__':
    listener()