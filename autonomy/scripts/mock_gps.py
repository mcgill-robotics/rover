import rospy
from std_msgs.msg import Float32MultiArray
rospy.init_node('publisher_and_listener', anonymous=True)
gps_pub = rospy.Publisher("fake/gps", Float32MultiArray, queue_size=100)
for n in range(0,110):
    f = Float32MultiArray()
    f.data = [45.536490, -73.630132]
    while (gps_pub.get_num_connections() < 1):
        pass
    gps_pub.publish(f)