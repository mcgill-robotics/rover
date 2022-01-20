#!/usr/bin/env python3

import sys
sys.path.append("..") 
from science_module.msg import ScienceCmd, ScienceFeedback, CcdData
import numpy as np
from matplotlib import pyplot as plt
import rospy

class Node_ScienceSim():

  def __init__(self):

    rospy.init_node("science_sim", anonymous=False)
    self.scienceCmdSubscriber = rospy.Subscriber("science_control_data", ScienceCmd, self.updateSim)
    self.scienceStatePublisher = rospy.Publisher("science_state_data", ScienceFeedback, queue_size=1)

    # Filter carousel
    self.filter_disk_center = (0,0)
    self.tube_disk_center = (2,2)
    self.filter_disk_radius = 1
    self.tube_disk_radius = 1

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
    self.led_light_pos      = (-1.5, 3)
    self.laser_light_pos    = (-1.5, 2.6)
    self.gripper_light_pos  = (-1.5, 2.2)
    self.snapshot_light_pos = (-1.5, 1.8)
    self.shutdown_light_pos = (-1.5, 1.4)

    # Simulation plot boundaries
    self.x_max = 4
    self.x_min = -2
    self.y_max = 4
    self.y_min = -2

    # Text displacement
    self.text_x_adjust = 0.1
    self.text_y_adjust = -0.1

    self.run()

  def updateSim(self, cmd):
    self.led_state = cmd.LedState
    self.laser_state = cmd.LaserState
    self.gripper_state = cmd.GripperState
    self.cooler_state = cmd.PeltierState
    self.snapshot_state = cmd.CcdSensorSnap
    self.shutdown_state = cmd.Shutdown
    self.motor_speed = cmd.MotorSpeed
    self.filter_target_angle = cmd.Stepper1IncAng
    self.tube_target_angle = cmd.Stepper2IncAng

  
  def run(self):

    plt.ion()
    fig = plt.figure()
    ax = fig.add_subplot(111)
    plt.show()

    while not rospy.is_shutdown():

      state_msg = ScienceFeedback()

      # Plot circle
      filter_disk = plt.Circle(self.filter_disk_center, self.filter_disk_radius, color='blue')
      tube_disk = plt.Circle(self.tube_disk_center, self.tube_disk_radius, color='green')

      # Plot angle indicator
      x_filter = self.filter_disk_radius*np.cos(self.filter_target_angle)
      y_filter = self.filter_disk_radius*np.sin(self.filter_target_angle)

      x_tube = self.filter_disk_radius*np.cos(self.tube_target_angle)
      y_tube = self.filter_disk_radius*np.sin(self.tube_target_angle)

      ax.plot(x_filter, y_filter, color='black', marker='o', markersize=10)
      ax.plot(x_tube, y_tube, color='black', marker='o', markersize=10)

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
      
      self.scienceStatePublisher.publish(state_msg)


if __name__ == "__main__":
    driver = Node_ScienceSim()
    rospy.spin()
