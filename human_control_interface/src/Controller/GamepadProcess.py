import rospy
from human_control_interface.msg import Gamepad_input
from geometry_msgs.msg import Twist

class Node_GamepadProcessing:
    def __init__(self, wMax, rW, baseLength):
        # initialize ROS node
        rospy.init_node("gamepad_process_node")

        # initialize variables for velocity
        self.roverLinearVelocity = 10
        self.roverAngularVelocity = 10

        # change these later, once you get actual values
        self.wMax = wMax
        self.rW = rW
        self.baseLength = baseLength

        # initialize variables for gamepad positioning
        self.xVal = 0
        self.yVal = 0
        self.zVal = 0

        # initialize variables for rover's max linear and angular velocities
        self.maxLinearVelocity = 0
        self.maxAngularVelocity = 0

        # initialize a subscriber for grabbing data from gamepad
        self.processSub = rospy.Subscriber("gamepad_data", Gamepad_input, gamepadProcessCall)
        self.processPub = rospy.Publisher("processGamepad_data", Twist, queue_size=1)

    def gamepadProcessCall(self, msg):
        # assign axis values
        self.xVal = msg.A2
        self.yVal = msg.A1
        self.zVal = msg.A6

        # calc. for linear velocity
        self.roverLinearVelocity = self.wMax * self.rW * self.zVal * self.xVal

        # calc. for angular velocity
        self.roverAngularVelocity = (2 / self.baseLength) * ((self.wMax * self.rW) - self.roverLinearVelocity) * self.zVal * self.yVal

        # assigns values to a Twist msg, then publish it to ROS
        roverTwist = Twist()
        roverTwist.linear.x = self.roverLinearVelocity
        roverTwist.angular.z = self.roverAngularVelocity

        self.processPub.publish(roverTwist)

if __name__ == "__main__":
    gamepadProcess = Node_GamepadProcessing(10, 10, 5)