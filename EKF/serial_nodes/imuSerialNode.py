#!/usr/bin/env python3

import rospy
import serial 
from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray

class ImuNode():
    """Serial ROS Node for exchanging data with the IMU

    Attributes
    --------

    """
    def __init__(self):
        # Initialize ROS
        rospy.init_node("UM7", anonymous=False)
        self.imu_publisher = rospy.Publisher("um7_data", Int32MultiArray, queue_size=1)

        port = '/dev/ttyACM0'
        try:
            self.imu = serial.Serial(port, baudrate=115200, timeout=0.1)
        except serial.SerialException:
            rospy.logerr("Could not connect to port")
            exit()
        # Start Node
        self.run()

    def run(self):
        """Runs the node loop for getting and updating the Gamepad information for user control
        """
        while not rospy.is_shutdown():
            if rospy.is_shutdown():
                exit()
            try:
                imu_msg = Int32MultiArray() # array type for topic

                data = self.pollImu()

                if data != b'': # check if data is not empty
                    imuData = [] 
                    data = str(data, 'utf-8') # converts to proper string format

                    for num in data.split(','): # splits values at comma
                        imuData.append(int(num))


                    imu_msg.data = imuData

                    self.imu_publisher.publish(imu_msg)

            except Exception as error:
                rospy.logerr(str(error))
        exit()

    def pollImu(self): # reads lines from serial monitor(nums separated by commas)
        return self.imu.readline()

if __name__ == "__main__":
    driver = ImuNode()
    rospy.spin()