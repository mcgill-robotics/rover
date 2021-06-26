#!/usr/bin/env python

from filterpy.kalman import ExtendedKalmanFilter as EKF
from numpy import dot, array, sqrt
from numpy.random import randn
from math import atan2, sqrt

import sympy
from sympy import symbols, Matrix
from sympy.abc import x

import rospy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

import sys
import copy
from std_msgs.msg import Float32

# Declare distance variable
distance = 0

# Goal marker topic
topic = 'visualization_marker'
publisher = rospy.Publisher(topic, Marker)

# Ekf track topic
ekf_topic = 'visualization_marker_array'
ekfPublisher = rospy.Publisher(ekf_topic, MarkerArray)

ekfArray = MarkerArray()

# Initialize ekf node to appear in rosnode list
rospy.init_node("ekf_node")

#Callback function for extracting data from the ultrasonic sensor and assigning it to distance variable
def ultraCall(distData):
    global distance 
    distance = distData.data

    # Testing rospy status
    """
    if rospy.is_shutdown == True:
	print("shut")
    else:
	print("running")
    """

# Subscribes to distReader topic (ultrasonic sensor)
def ultraSub():
    distanceRead = rospy.Subscriber("distReader", Float32, ultraCall)

class RobotEKF(EKF):
    # initializes motion model with standard dev. of velocity and time step
    def __init__(self, dt, std_vel):
	# uses filterpy.kalman to create a one-dimensional state with 1 measurement (for position)
	EKF.__init__(self, 1, 1, 2)
	self.dt = dt
	self.std_vel = std_vel
	
	# defines symbols of x pos., velocity and time
	x, v, time = symbols('x, v, t')
	d = v * time
	
	# motion model (focusing only on x position and distance travelled)
	self.fxu = Matrix([[x + d]])
	self.F_j = self.fxu.jacobian(Matrix([x]))
	self.V_j = self.fxu.jacobian(Matrix([v]))
	
	# save dictionary and its variables for later use
	self.subs = {x: 0, v:0, time:dt}
        self.pos_x = x
        self.v = v
	
    def predict(self, u=0):
	# uses velocity as control input
	self.subs[self.v] = u[0]
	
	# converts arrays to floating-point approx.
	F = array(self.F_j.evalf(subs=self.subs)).astype(float)
        V = array(self.V_j.evalf(subs=self.subs)).astype(float)

	# covariance of motion noise in control space
        M = array([[self.std_vel**2]])

	# total motion noise (take noise related to position and adds it to noise related to control input)
        self.P = dot(F, self.P).dot(F.T) + dot(V, M).dot(V.T)

# jacobian of sensor model (assumes measurements are exact)
def Hj(x):
    """ compute Jacobian of H matrix where h(x) computes 
    the range and bearing to a landmark for state x """
    H = array([[1]])
    
    return H

# sensor/measurement model (returns the state variable)	 
def Hx(x):
    """ takes a state variable and returns the measurement
    that would correspond to that state. """
    Hx = array([x[0]])
    
    return Hx

# what was actually measured minus what the measurement was supposed to be
def residual(a, b):
	# no rotation
	y = a - b
	return y

# takes distance given by ultrasonic sensor and converts to position
def get_sensor_reading():
    # obtains current x position; distance measured is the same value
    z = [distance]
    return z

# verify whether wall/goal is reached (*tested, works*)
def reach_goal(current_state, goal_state):
    """ check whether goal state is reached """
    if current_state[0] - goal_state[0] < 10:
      return True
    else:
      return False

from math import sqrt, tan, cos, sin, atan2
import numpy as np

# sets time step to 0.1s
dt = 0.1

def run_navigation(start_state, goal_state, init_var, std_vel, std_range):
    ekf = RobotEKF(dt, std_vel=std_vel)		# initializes new RobotEKF with given time step and std. of velocity
    ekf.x = start_state         # x position

    # initialize state variance P and measurement variance R
    ekf.P = np.diag([init_var[0]])
    ekf.R = np.diag([std_range**2])

    # array containing the positions of the robot	
    ekf_track = []

    markerCount = 0
    id = 0

    while not rospy.is_shutdown():
	u = array([0.1]) # steering command (constant velocity of 0.1m/s)
        ekf.predict(u=u)	# predict function with assigned steering commandz
        z = get_sensor_reading() # measurement function that gives position from sensor
        ekf.update(z, HJacobian = Hj, Hx=Hx, residual=residual)		# update function
        
	# keep track of the EKF path
	ekf_track.append(ekf.x)
	# rospy.loginfo(ekf.x) # for debugging purposes 

	# calls subscriber to get distance readings
        ultraSub()
	
	# RVIZ sim 
	#Define values for the visualized goal state
	goal = Marker()
	goal.header.frame_id = "/ekf"
	goal.type = goal.CUBE	
	goal.action = goal.ADD

	goal.scale.x = 0.7
	goal.scale.y = 0.7
	goal.scale.z = 0.65

	goal.pose.position.x = 0.0
	goal.pose.position.y = 0.0
	goal.pose.position.z = 0.0
	
	goal.color.a = 1.0
	goal.color.r = 1.0
	goal.color.g = 1.0
	goal.color.b = 0.0

   	goal.pose.orientation.x = 0.0
    	goal.pose.orientation.y = 0.0
    	goal.pose.orientation.z = 0.0
   	goal.pose.orientation.w = 1.0

	publisher.publish(goal)

	# Define trajectory for ekf track
	ekfMarker = Marker()
	ekfMarker.header.frame_id = "/ekf"
	ekfMarker.ns = "ekf_space"
	ekfMarker.type = ekfMarker.SPHERE
	ekfMarker.action = ekfMarker.ADD
	ekfMarker.scale.x = 0.7
	ekfMarker.scale.y = 0.7
	ekfMarker.scale.z = 0.4
	ekfMarker.color.a = 1
	ekfMarker.color.r = 0.5
	ekfMarker.color.g = 0
	ekfMarker.color.b = 0
	ekfMarker.pose.orientation.w = 1.0
				
	ekfMarker.pose.position.x = ekf.x[0]
	ekfMarker.pose.position.y = 0
	ekfMarker.pose.position.z = 0
	
	# Should remove the first entry in array after count reaches past 5
	if (markerCount > 10):
	  ekfArray.markers.pop(0)
	
	ekfArray.markers.append(ekfMarker)
	
	# Renumber id to not have old markers
	id = 0
	for e in ekfArray.markers:
	    e.id = id
            id += 1	

	ekfPublisher.publish(ekfArray)
	markerCount += 1	

	rospy.sleep(0.02)

start_state = np.array([100.0]) # start state of robot is defined at the initial distance measured (pre-set to 100cm/1m)
goal_state = np.array([0.0]) # goal state is set as the origin in the frame
init_var = [0.1]   # initial std for state variable x
	
ekf = run_navigation(start_state=start_state, goal_state=goal_state, init_var = init_var, std_vel = 0.3, std_range = 0.5) # starts navigation with set values

