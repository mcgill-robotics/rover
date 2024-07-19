#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Header
import numpy as np


class ArmVisualizer:
    def __init__(self):
        rospy.init_node("arm_visualizer", anonymous=False)

        self.joint_state_publisher = rospy.Publisher(
            "/joint_states", JointState, queue_size=10
        )
        self.rate = rospy.Rate(1000)  # 50 Hz

        self.joint_state = JointState()
        self.joint_state.header = Header()
        self.joint_state.name = [
            "joint_1",
            "joint_2",
            "joint_3",
            "joint_4",
            "joint_5",
            "joint_6",
            "joint_7",
        ]

        # Initial joint positions
        self.joint_positions = np.zeros(len(self.joint_state.name))

        # Subscribers for command positions
        rospy.Subscriber("armBrushedCmd", Float32MultiArray, self.updateArmBrushedCmd)
        rospy.Subscriber(
            "armBrushlessCmd", Float32MultiArray, self.updateArmBrushlessCmd
        )

        self.run()

    def updateArmBrushedCmd(self, data):
        print("Received brushed command data:", data.data)
        # self.joint_positions[4], self.joint_positions[3] = tuple(
        #     x * (np.pi / 180) for x in data.data[1:]
        # )
        # self.joint_positions[5] = np.clip(
        #     self.joint_positions[5] + data.data[0], -0.3, 0.11
        # )
        # self.joint_positions[6] = self.joint_positions[5]
        self.joint_positions[4] = np.deg2rad(data.data[0])  # Joint 5
        self.joint_positions[3] = np.deg2rad(data.data[1])  # Joint 4
        self.joint_positions[5] = np.deg2rad(data.data[2])  # Joint 6
        self.joint_positions[6] = np.deg2rad(
            data.data[2]
        )  # Joint 7 (assuming same as Joint 6)
        self.publish_joint_states()

    def updateArmBrushlessCmd(self, data):
        print("Received brushless command data:", data.data)
        # self.joint_positions[2], self.joint_positions[1], self.joint_positions[0] = (
        #     tuple(x * (np.pi / 180) for x in data.data)
        # )
        self.joint_positions[2] = np.deg2rad(data.data[0])  # Joint 3
        self.joint_positions[1] = np.deg2rad(data.data[1])  # Joint 2
        self.joint_positions[0] = np.deg2rad(data.data[2])  # Joint 1
        self.publish_joint_states()

    def publish_joint_states(self):
        self.joint_state.header.stamp = rospy.Time.now()
        self.joint_state.position = self.joint_positions
        self.joint_state_publisher.publish(self.joint_state)
        # print(self.joint_positions)

    def run(self):
        while not rospy.is_shutdown():
            self.publish_joint_states()
            self.rate.sleep()


if __name__ == "__main__":
    try:
        visualizer = ArmVisualizer()
    except rospy.ROSInterruptException:
        pass
