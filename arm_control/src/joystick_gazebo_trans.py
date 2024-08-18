#!/usr/bin/env python3


import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray, Float32, Float64
from sensor_msgs.msg import JointState

class Joystick_Gazebo_Trans():

    def __init__(self):

        #Initialize fields for cmds of each joint of the arm.
        self.tumorCmd = Float64()
        self.shoulderCmd = Float64()
        self.elbowCmd = Float64()
        self.wristCmd = Float64()
        self.wristRollCmd = Float64()
        self.clawCmd = Float64()

        #Initialize fields for FB of each joint of the arm.
        self.tumorFB = Float64()
        self.shoulderFB = Float64()
        self.elbowFB = Float64()
        self.wristFB = Float64()
        self.wristRollFB = Float64()
        self.clawFB = Float64()

        #Initialize fields for Brushed and Brushless FB arrays.
        self.brushedFB = Float32MultiArray()
        self.brushlessFB = Float32MultiArray()
        
        
        #Initialize ROS
        rospy.init_node("joyst_gaz_trans", anonymous=False)

        #Declare subscribers for arm controller cmd topics.
        self.BrushedSubscriber = rospy.Subscriber("armBrushedCmd", Float32MultiArray, self.BrushedProcess)
        self.BrushlessSubscriber = rospy.Subscriber("armBrushlessCmd", Float32MultiArray, self.BrushlessProcess)

        #Declare subscriber for Gazebo joint states topic.
        self.jointStSubscriber = rospy.Subscriber("joint_states", JointState, self.StateProcess)

        #Declare publishers for arm controller FB topics.
        self.BrushedPublisher = rospy.Publisher("armBrushedFB", Float32MultiArray, queue_size=10)
        self.BrushlessPublisher = rospy.Publisher("armBrushlessFB", Float32MultiArray, queue_size=10)

        #Declare publishers for all arm links.
        self.joint1Publisher = rospy.Publisher("joint_1_controller/command", Float64, queue_size=10)
        self.joint2Publisher = rospy.Publisher("joint_2_controller/command", Float64, queue_size=10)
        self.joint3Publisher = rospy.Publisher("joint_3_controller/command", Float64, queue_size=10)
        self.joint4Publisher = rospy.Publisher("joint_4_controller/command", Float64, queue_size=10)
        self.joint5Publisher = rospy.Publisher("joint_5_controller/command", Float64, queue_size=10)
        self.joint6Publisher = rospy.Publisher("joint_6_controller/command", Float64, queue_size=10)
        self.joint7Publisher = rospy.Publisher("joint_7_controller/command", Float64, queue_size=10)

        #Declare rate
        self.rate = rospy.Rate(100)

        self.run()

    
    def run (self):
        #Declare arrays for arm FB.
        brushedFB = Float32MultiArray()
        brushlessFB = Float32MultiArray()
        
        while not rospy.is_shutdown():

            #Put all of the FB data in the corresponding array.
            brushlessFB.data = [self.elbowFB.data, self.shoulderFB.data, self.tumorFB.data]
            brushedFB.data = [self.clawFB.data, self.wristRollFB.data, self.wristFB.data]


            #Publish msgs to FB topics.
            self.BrushedPublisher.publish(brushedFB)
            self.BrushlessPublisher.publish(brushlessFB)

            #Publish msgs to each arm joint.
            self.joint1Publisher.publish(self.tumorCmd)
            self.joint2Publisher.publish(self.shoulderCmd)
            self.joint3Publisher.publish(self.elbowCmd)
            self.joint4Publisher.publish(self.wristCmd)
            self.joint5Publisher.publish(self.wristRollCmd)
            self.joint6Publisher.publish(self.clawCmd)
            self.joint7Publisher.publish(self.clawCmd)

            self.rate.sleep()




    
    def BrushedProcess (self, brushedInp: Float32MultiArray):
        #Distribute entries of brushedInp into the corresponding topics.
        self.wristRollCmd.data = brushedInp.data[1] * (np.pi/180) 
        self.wristCmd.data = brushedInp.data[2] * (np.pi/180)   

        #If the input for the claw is positive, close it
        if (brushedInp.data[0] > 0):
            self.clawCmd.data = 0.118

        
        #if negative, open the claw.
        elif (brushedInp.data[0] < 0):
            self.clawCmd.data = -0.3

    def BrushlessProcess (self, brushlessInp: Float32MultiArray):
        #Distribute entries of brushlessInp into the corresponding topics.
        self.tumorCmd.data = brushlessInp.data[2] * (np.pi/180) 
        self.shoulderCmd.data = brushlessInp.data[1] * (np.pi/180)
        self.elbowCmd.data = brushlessInp.data[0] * (np.pi/180)
         
    def StateProcess (self, jState: JointState):
        self.tumorFB.data = jState.position[0] * (180/np.pi)
        self.shoulderFB.data = jState.position[1] * (180/np.pi)
        self.elbowFB.data = jState.position[2] * (180/np.pi)
        self.wristFB.data = jState.position[3] * (180/np.pi)
        self.wristRollFB.data = jState.position[4] * (180/np.pi)
        self.clawFB.data = jState.position[5] * (180/np.pi)

if __name__ == "__main__":
  driver = Joystick_Gazebo_Trans()
  rospy.spin()