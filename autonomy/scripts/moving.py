#!/usr/bin/env python
from time import sleep
import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
import math
from geometry_msgs.msg import Vector3

# Works in three steps 
# 1. Robot goes forward to be able to discern its current direction
# 2. Robot turns to face the right direction
# 3. The robot goes mostly forward, and correct its path until reaching b

class No_obstacles:
    # Current Path
    cur_path = []

    # Turning needed
    turn_need = True

    # When you want the robot to be stopped
    stopped = False

    # Newest Position and Orientation
    pos = None
    ori = None

    # distance tolerance
    tol_d = 0.15

    # angular tolerance (in degrees)
    tol_a = 1
    
    def __init__(self):
        rospy.init_node('a_to_b')
        self.vel_pub = rospy.Publisher('/rover_velocity_controller/cmd_vel', Twist, queue_size=10)
        self.pose_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.navigation)
        self.path_sub = rospy.Subscriber("/path", Float32MultiArray, self.update_path)
        rospy.spin()

    # Happens when a new path is transfered
    def update_path(self, msg):
        array = msg.data
        if (len(array) == 0):
            self.vel_pub.publish(Twist(Vector3(0,0,0), Vector3(0,0,0)))
            self.stopped = True
            return

        self.stopped = False
        h = int(len(array)/2)

        new_path = []
        for n in range (0,h):
            x = array[2*n]
            y = array[2*n+1]
            point = [x, y]
            new_path.append(point)
        
        self.cur_path = new_path
        self.turn_need = True
        
    
    def turn(self):
        dx1, dy1 = self.ori
        dx2, dy2 = self.required_ori()

        dist_left = math.sqrt(dx2*dx2 + dy2*dy2)
        # Find angle between both vectors
        angle = math.acos((dx1*dx2 + dy1*dy2)/(math.sqrt(dx1*dx1 + dy1*dy1)*dist_left))*180/math.pi
        # Find sign of angle using cross product
        cross = dx1*dy2 - dx2*dy1
        
        if cross >= 0:
            sign = 1
        else: 
            sign = -1

        if (angle > self.tol_a):
            self.vel_pub.publish(Twist(Vector3(0,0,0), Vector3(0,0,sign*math.sqrt(angle/25))))
        else:
            self.turn_need = False

    # returns the 2D angle of orientation
    def quart_2D(self, q):
        return (q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z), 2.0*(q.x*q.y + q.w*q.z)

    
    # returns the 2D angle of required orientation
    def required_ori(self):
        cur_dest = self.cur_path[-1]
        dx = cur_dest[0] - self.pos.x
        dy = cur_dest[1] - self.pos.y
        return dx, dy
    
    def global_dist(self):
        cur_dest = self.cur_path[0]
        dx = cur_dest[0] - self.pos.x
        dy = cur_dest[1] - self.pos.y
        return math.sqrt(dx*dx + dy*dy)
    
    def navigation(self, msg):
        # If its empty you just return
        if (len(self.cur_path) == 0):
            return
        
        # When it needs to be stopped
        if self.stopped:
            self.vel_pub.publish(Twist(Vector3(0,0,0), Vector3(0,0,0)))
            return

        # Update Position and Orientation, Keep the previous one
        self.pos = msg.pose[1].position
        self.ori = self.quart_2D(msg.pose[1].orientation)
        
        # Check if turn needed
        if self.turn_need:
            self.turn()
            return

        # Check if rover arrived (sub)
        if abs(self.pos.x - self.cur_path[-1][0]) < self.tol_d and abs(self.pos.y - self.cur_path[-1][1]) < self.tol_d:
            self.cur_path.pop()
            return
        
        dx1, dy1 = self.ori
        dx2, dy2 = self.required_ori()

        dist_left = math.sqrt(dx2*dx2 + dy2*dy2)
        # Find angle between both vectors
        angle = math.acos((dx1*dx2 + dy1*dy2)/(math.sqrt(dx1*dx1 + dy1*dy1)*dist_left))*180/math.pi
        if angle >= 90:
            self.turn_need = True
            return

        slow = 1
        turn = 1
        if (angle > 15):
            slow = angle/15
            turn = 1 - (angle/200)

        
        # Find sign of angle using cross product
        cross = dx1*dy2 - dx2*dy1
        
        if cross >= 0:
            sign = 1
        else: 
            sign = -1
        
        dist_left_global = self.global_dist()
        # Thighness of turn is higher for smaller distance and higher angle
        speed_factor = math.sqrt(dist_left_global)*0.1
        if (speed_factor > 0.1):
            speed_factor = 0.1
        self.vel_pub.publish(Twist(Vector3(speed_factor/slow,0,0), Vector3(0,0,sign*math.sqrt(angle/(70*turn)))))
            
if __name__ == '__main__':
    No_obstacles()