#!/usr/bin/env python3

import sys
sys.path.append("..") 
import numpy as np
from matplotlib import pyplot as plt
import rospy
from std_msgs.msg import Float32MultiArray

class Node_ScienceSim():

  def __init__(self):

    rospy.init_node("science_sim", anonymous=False)
    self.science_cmd_subscriber = rospy.Subscriber("science_cmd", Float32MultiArray, self.update_sim)
    self.science_state_publisher = rospy.Publisher("science_fb", Float32MultiArray, queue_size=1)

    # Carousel dimensions and info
    self.carousel_disk_center = (105,0)
    self.carousel_disk_radius = 60
    self.group_offset = np.pi/2

    # Control signals
    self.auger_speed = 0
    self.linear_guide_speed = 0
    self.carousel_angle = 0
    
    # Control Brushed Motor Feaadback locations
    self.auger_speed_label_pos      = (-60, 110)
    self.linear_guide_label_pos    = (20, 110)

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
    self.carousel_angle = self.carousel_angle + cmd.data[0] * np.pi/180
    self.auger_speed = cmd.data[1]
    self.linear_guide_speed = cmd.data[2]
  
  def run(self):

    plt.ion()
    fig = plt.figure()
    ax = fig.add_subplot(111)
    plt.show()
    tick = 0

    while not rospy.is_shutdown():

      state_msg = Float32MultiArray()

      # Plot circle
      carousel_disk = plt.Circle(self.carousel_disk_center, self.carousel_disk_radius, color='blue', alpha=0.2)

      # Plot angle indicators for
      for i in range(4):
        x = self.carousel_disk_radius*np.cos(self.carousel_angle + i*self.group_offset) + self.carousel_disk_center[0]
        y = self.carousel_disk_radius*np.sin(self.carousel_angle + i*self.group_offset) + self.carousel_disk_center[1]
        ax.plot(x, y, color='blue' if i == 0 else 'black', marker='o', markersize=10, alpha=0.5)

      # Add carousels
      ax.add_patch(carousel_disk)
     
      # Add Brushed Motors
      # Auger
      ax.plot(self.auger_speed_label_pos[0], self.auger_speed_label_pos[1], marker='o', markersize=10)
      ax.text(self.auger_speed_label_pos[0] + self.text_x_adjust, self.auger_speed_label_pos[1] + self.text_y_adjust, f"Auger Speed: {self.auger_speed}")

      # Laser
      ax.plot(self.linear_guide_label_pos[0], self.linear_guide_label_pos[1], marker='o', markersize=10)
      ax.text(self.linear_guide_label_pos[0] + self.text_x_adjust, self.linear_guide_label_pos[1] + self.text_y_adjust, f"Linear Guide Speed: {self.linear_guide_speed}")
              
      # Draw new visual
      ax.set_xlim(self.x_min, self.x_max)
      ax.set_ylim(self.y_min, self.y_max)
      fig.canvas.draw()
      plt.pause(0.01)
      ax.cla()

      state_msg.data.append(np.sin(tick / (32 * np.pi)))
      state_msg.data.append(np.cos(tick / (32 * np.pi)))
      state_msg.data.append(np.sin(tick / (128 * np.pi)))
      state_msg.data.append(np.arctan(tick / (32 * np.pi)))
      
      self.science_state_publisher.publish(state_msg)
      tick += 1


if __name__ == "__main__":
    driver = Node_ScienceSim()
    rospy.spin()
