
import rospy
from std_msgs.msg import Float32MultiArray

def talker():
    pub = rospy.Publisher("test_topic", Float32MultiArray, queue_size=10)
    rospy.init_node("antennaData", anonymous=True)
    rate = rospy.Rate(0.3)
    while not rospy.is_shutdown():
        hello_str = "Hellos woolrd %s" %rospy.get_time()
        rospy.loginfo(hello_str)
        # pub.publish(hello_str)
        pub.publish(Float32MultiArray(data=[1,2,3, 4, 5, 6, 7, 8, 9, 10, 11, 12 ]))
        pub.publish(Float32MultiArray(data=[5,7, 1, 2, 2, 2, 7, 2, 9, 2, 1, 2 ]))
        rate.sleep()


if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

