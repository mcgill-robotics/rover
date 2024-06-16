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
    stop_need = False

    # Newest Position and Orientation
    pos = None
    ori = None

    # distance tolerance
    tol_d = 0.15

    # angular tolerance (in degrees)
    tol_a = 3
    
    def __init__(self):
        rospy.init_node('a_to_b')
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pose_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.navigation)
        self.path_sub = rospy.Subscriber("/path", Float32MultiArray, self.update_path)
        rospy.spin()

    # Happens when a new path is transfered
    def update_path(self, msg):
        array = msg.data
        if (len(array) == 0):
            self.stop_need = True
            return
        self.tol_d = 0.3 # The first point is too uncertain make sure to nuke it
        self.stop_need = False
        h = int(len(array)/2)

        new_path = []
        for n in range (0,h):
            x = array[2*n]
            y = array[2*n+1]
            point = [x, y]
            new_path.append(point)
        
        self.cur_path = new_path
        self.turn_need = True
        
    # returns the 2D angle of orientation
    def quart_2D(self, q):
        return (q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z), 2.0*(q.x*q.y + q.w*q.z)

    def get_pitch(self, q):
        return math.asin(2.0*(q.x*q.z - q.w*q.y))*180/math.pi
    
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
        # Update Position and Orientation, Keep the previous one
        self.pos = msg.pose[15].position
        self.ori = self.quart_2D(msg.pose[15].orientation)
        pitch = self.get_pitch(msg.pose[15].orientation)

        # If its empty you just return
        if (len(self.cur_path) == 0 and not self.stop_need):
            self.stop_need = True
            return
        
        # Check if rover arrived (sub)
        if not (len(self.cur_path)) == 0 and (abs(self.pos.x - self.cur_path[-1][0]) < self.tol_d and abs(self.pos.y - self.cur_path[-1][1]) < self.tol_d):
            self.cur_path.pop()
            self.tol_d = 0.15
            return
        
        if self.stop_need:
            self.vel_pub.publish(Twist(Vector3((pitch/45), 0, 0), Vector3(0, 0, 0)))
            return

        dx1, dy1 = self.ori
        dx2, dy2 = self.required_ori()
        dist_left = math.sqrt(dx2*dx2 + dy2*dy2)
        # Find angle between both vectors
        post = (dx1*dx2 + dy1*dy2)/(math.sqrt(dx1*dx1 + dy1*dy1)*dist_left)
        if post > 1:
            post = 1
        if post < -1:
            post = -1
        angle = math.acos(post)*180/math.pi
        if angle >= 90:
            self.turn_need = True

        # Find sign of angle using cross product
        cross = dx1*dy2 - dx2*dy1
        if cross >= 0:
            sign = 1
        else: 
            sign = -1

        if self.turn_need:
            if (angle > self.tol_a):
                self.vel_pub.publish(Twist(Vector3(pitch/85,0,0), Vector3(0,0,sign*math.sqrt(angle/35))))
            else:
                self.turn_need = False
        else:
            slow = 1
            turn = 1
            if (angle > 15):
                slow = angle/15
                turn = 1 - (angle/200)
            dist_left_global = self.global_dist()
            # Thighness of turn is higher for smaller distance and higher angle
            speed_factor = 100*math.sqrt(dist_left_global)
            if (speed_factor > 0.05):
                speed_factor = 0.05
            if pitch < 0: # Negative pitch will make the robot go backwards if its too big
                pitch = pitch/100
                if pitch > -7:
                    pitch = 0
            self.vel_pub.publish(Twist(Vector3((speed_factor/slow + pitch/48),0,0), Vector3(0,0,sign*math.sqrt(angle/(50*turn)))))

if __name__ == '__main__':
    No_obstacles()