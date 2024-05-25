import os, sys
currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(currentdir)
import rospy
from steering import Steering
from drive_control.msg import WheelSpeed
from geometry_msgs.msg import Twist

class Node_DriveControl():

    def __init__(self):
        """
        Hard-coded the simulation rover measurement, need to adapt it towards the real model.
        Script takes care of converting twist velocities from controller into rover wheel velocities.
        Specifically, velocities are published to create an accelerating filter (local average filter for math geeks).
        
        """
        self.wheel_radius = 0.04688 # In meters.
        self.wheel_base_length = 0.28 # In meters.
        self.wheel_speed = [0, 0] # Array elements: Left wheel speed, right wheel speed.
        self.steering = Steering(self.wheel_radius, self.wheel_base_length)

        rospy.init_node('drive_controller')
        self.angular_velocity_publisher = rospy.Publisher('/wheel_velocity_cmd', WheelSpeed, queue_size=1)
        self.robot_twist_subscriber = rospy.Subscriber("rover_velocity_controller/cmd_vel", Twist, self.twist_to_velocity)

        # Control Frequency of the drive controller
        self.rate = rospy.Rate(50)

        self.run()
    
    # Function to receive twist velocities and call the steering function to get the rover wheel velocities.
    def twist_to_velocity(self, robot_twist):
        vR = robot_twist.linear.x
        wR = robot_twist.angular.z
        self.wheel_speed = self.steering.steering_control(vR, wR)

    def run(self):
        while not rospy.is_shutdown():

            cmd = WheelSpeed() # Create the wheel speed message.        
            print(f"Speed: {self.wheel_speed}")
            # Populate the message with the steering values.
            cmd.left[0], cmd.left[1] = self.wheel_speed[0], self.wheel_speed[0]
            cmd.right[0], cmd.right[1] = self.wheel_speed[1], self.wheel_speed[1]

            self.angular_velocity_publisher.publish(cmd) # Send the angular speeds.

            self.rate.sleep()
            print(cmd)     

# ROS runtime main entry point
if __name__ == "__main__":
    driver = Node_DriveControl()
    rospy.spin()
