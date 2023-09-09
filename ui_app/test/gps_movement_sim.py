from pygame.locals import *
import pygame
import rospy
from std_msgs.msg import Float32MultiArray
import sys
from time import sleep


window_height = 100
window_width = 100

window = pygame.display.set_mode((window_width, window_height))
# This is the code to check if a player is pushing the arrows

# ROS node
robot_gps_publisher = rospy.Publisher("/roverGPSData", Float32MultiArray, queue_size=0)
antenna_params_publisher = rospy.Publisher("/antennaData", Float32MultiArray, queue_size=0)
rospy.init_node('gps_publisher')

# GPS position
gps_coords = [51.467437, -112.706696]
step = 0.0001

# Antenna location
antenna_coords = [51.467239, -112.706449, gps_coords[0], gps_coords[1]]

antenna_params_publisher.publish(Float32MultiArray(data=antenna_coords))

while not rospy.is_shutdown():
    for evenement in pygame.event.get():
        if evenement.type == QUIT or (evenement.type == KEYDOWN and 
                                      evenement.key == K_ESCAPE):
            print('QUIT')
            pygame.quit()
            sys.exit()

        if evenement.type == KEYDOWN and evenement.key == K_RIGHT:
            # print("Clicked on the right arrow")
            gps_coords[1] += step
        if evenement.type == KEYDOWN and evenement.key == K_LEFT:
            # print("Clicked on the left arrow")
            gps_coords[1] -= step
        if evenement.type == KEYDOWN and evenement.key == K_UP:
            # print("Clicked on the up arrow")
            gps_coords[0] += step
        if evenement.type == KEYDOWN and evenement.key == K_DOWN:
            # print("Clicked on the down arrow")
            gps_coords[0] -= step

    robot_gps_publisher.publish(Float32MultiArray(data=gps_coords))
    print(f"gps_coords: {gps_coords}", end='\r')
    sleep(0.1)