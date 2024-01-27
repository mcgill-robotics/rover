
import rospy
from std_msgs.msg import Float32MultiArray

def talker():
    pub = rospy.Publisher("test_topic", Float32MultiArray, queue_size=10)
    rospy.init_node("antennaData", anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        hello_str = "Hellos woolrd %s" %rospy.get_time()
        rospy.loginfo(hello_str)
        # pub.publish(hello_str)
        pub.publish(Float32MultiArray(data=[1,1]))
        rate.sleep()


if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

