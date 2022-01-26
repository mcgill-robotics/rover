#!/usr/bin/env python3

import sys
sys.path.append("..") 
from science_module.msg import ScienceCmd, ScienceFeedback
import numpy as np
from matplotlib import pyplot as plt
import rospy

class Node_ScienceSim():

  def __init__(self):

    rospy.init_node("science_sim", anonymous=False)
    self.science_cmd_subscriber = rospy.Subscriber("science_control_data", ScienceCmd, self.update_sim)
    self.science_state_publisher = rospy.Publisher("science_state_data", ScienceFeedback, queue_size=1)

    # Carousel dimensions and info
    self.filter_disk_center = (105,0)
    self.tube_disk_center = (0,0)
    self.filter_disk_radius = 60
    self.tube_disk_radius = 60
    self.tube_neighbor_offset = 14.4 * np.pi / 180
    self.group_offset = np.pi/2

    # Control signals
    self.led_state = False
    self.laser_state = False
    self.gripper_state = False
    self.cooler_state = False
    self.snapshot_state = False
    self.shutdown_state = False
    self.motor_speed = 0
    self.filter_target_angle = 0
    self.tube_target_angle = 0

    # Control Light locations
    self.led_light_pos      = (-60, 110)
    self.laser_light_pos    = (-20, 110)
    self.gripper_light_pos  = (25, 110)
    self.snapshot_light_pos = (70, 110)
    self.shutdown_light_pos = (115, 110)

    # Simulation plot boundaries
    self.x_max = 170
    self.x_min = -70
    self.y_max = 120
    self.y_min = -120

    # Text displacement
    self.text_x_adjust = 5
    self.text_y_adjust = -5

    self.run()

  def update_sim(self, cmd):
    self.led_state = cmd.LedState
    self.laser_state = cmd.LaserState
    self.gripper_state = cmd.GripperState
    self.cooler_state = cmd.PeltierState
    self.snapshot_state = cmd.CcdSensorSnap
    self.shutdown_state = cmd.Shutdown
    self.motor_speed = cmd.MotorSpeed
    self.filter_target_angle = self.filter_target_angle + cmd.Stepper1IncAng
    self.tube_target_angle = self.tube_target_angle + cmd.Stepper2IncAng

  
  def run(self):

    plt.ion()
    fig = plt.figure()
    ax = fig.add_subplot(111)
    plt.show()

    while not rospy.is_shutdown():

      state_msg = ScienceFeedback()

      # Plot circle
      filter_disk = plt.Circle(self.filter_disk_center, self.filter_disk_radius, color='blue', alpha=0.2)
      tube_disk = plt.Circle(self.tube_disk_center, self.tube_disk_radius, color='green', alpha=0.2)

      # Plot angle indicators for
      for i in range(4):
        x = self.filter_disk_radius*np.cos(self.filter_target_angle + i*self.group_offset) + self.filter_disk_center[0]
        y = self.filter_disk_radius*np.sin(self.filter_target_angle + i*self.group_offset) + self.filter_disk_center[1]
        ax.plot(x, y, color='blue' if i == 0 else 'black', marker='o', markersize=10, alpha=0.5)
      
      for i in range(4):
        for j in [-1, 0, 1]:
          x = self.filter_disk_radius*np.cos(self.tube_target_angle + i*self.group_offset + j*self.tube_neighbor_offset) + self.tube_disk_center[0]
          y = self.filter_disk_radius*np.sin(self.tube_target_angle + i*self.group_offset + j*self.tube_neighbor_offset) + self.tube_disk_center[1]
          ax.plot(x, y, color='green' if (i == 0 and j == 0) else 'black', marker='o', markersize=10, alpha=0.5)

      # Add carousels
      ax.add_patch(filter_disk)
      ax.add_patch(tube_disk)

      # Add boolean signals
      # LED
      ax.plot(self.led_light_pos[0], self.led_light_pos[1], color= 'green' if self.led_state else 'red', marker='o', markersize=10)
      ax.text(self.led_light_pos[0] + self.text_x_adjust, self.led_light_pos[1] + self.text_y_adjust, "LED")

      # Laser
      ax.plot(self.laser_light_pos[0], self.laser_light_pos[1], color= 'green' if self.laser_state else 'red', marker='o', markersize=10)
      ax.text(self.laser_light_pos[0] + self.text_x_adjust, self.laser_light_pos[1] + self.text_y_adjust, "Laser")

      # Gripper
      ax.plot(self.gripper_light_pos[0], self.gripper_light_pos[1], color= 'green' if self.gripper_state else 'red', marker='o', markersize=10)
      ax.text(self.gripper_light_pos[0] + self.text_x_adjust, self.gripper_light_pos[1] + self.text_y_adjust, "Gripper")

      # Snapshot
      ax.plot(self.snapshot_light_pos[0], self.snapshot_light_pos[1], color= 'green' if self.snapshot_state else 'red', marker='o', markersize=10)
      ax.text(self.snapshot_light_pos[0] + self.text_x_adjust, self.snapshot_light_pos[1] + self.text_y_adjust, "Snapshot")

      # Shutdown
      ax.plot(self.shutdown_light_pos[0], self.shutdown_light_pos[1], color= 'green' if self.shutdown_state else 'red', marker='o', markersize=10)
      ax.text(self.shutdown_light_pos[0] + self.text_x_adjust, self.shutdown_light_pos[1] + self.text_y_adjust, "Shutdown")


      # Draw new visual
      ax.set_xlim(self.x_min, self.x_max)
      ax.set_ylim(self.y_min, self.y_max)
      fig.canvas.draw()
      plt.pause(0.01)
      ax.cla()

      # Send Feedback
      state_msg.LedState = self.led_state
      state_msg.LaserState = self.laser_state
      state_msg.GripperState = self.gripper_state
      state_msg.PeltierState = self.cooler_state
      state_msg.Stepper1Fault = False
      state_msg.Stepper2Fault = False
      
      self.science_state_publisher.publish(state_msg)


if __name__ == "__main__":
    driver = Node_ScienceSim()
    rospy.spin()
