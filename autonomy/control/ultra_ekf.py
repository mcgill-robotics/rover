#!/usr/bin/env python

from filterpy.kalman import ExtendedKalmanFilter as EKF
from numpy import dot, array, sqrt
from numpy.random import randn
from math import atan2, sqrt
import sympy
from sympy import symbols, Matrix
from sympy.abc import beta, x, y, v, R, theta

import rospy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

import sys
import copy
from std_msgs.msg import String, Float32

# Initialize ekf node to appear in rosnode list
rospy.init_node("ekf_node")

# Goal marker topic
goal_topic = 'visualization_marker'
goalPublisher = rospy.Publisher(goal_topic, Marker)

# Ideal track topic
ideal_topic = 'visualization_marker_array'
idealPublisher = rospy.Publisher(ideal_topic, MarkerArray)

ideal = MarkerArray()

# Ekf track topic
ekf_topic = 'visualization_marker_array'
ekfPublisher = rospy.Publisher(ekf_topic, MarkerArray)

ekfArray = MarkerArray()

# Declare distance variable
distance = 0

#Callback function for extracting data from the ultrasonic sensor and assigning it to distance variable
def callback(data):
    global distance 
    distance = data.data

# Subscribes to distReader topic (ultrasonic sensor)
distanceRead = rospy.Subscriber("distReader", Float32, callback)
rospy.spin()

#initial starting point for robot (goal/wall/obstacle as reference)
init_distance = distance

class RobotEKF(EKF):
    def __init__(self, dt, std_vel, std_steer):
        EKF.__init__(self, 3, 2, 2)
        self.dt = dt
        self.std_vel = std_vel
        self.std_steer = std_steer

        beta, x, y, v, theta, time = symbols(
            'beta, x, y, v, theta, t')
        d = v*time
    
        self.fxu = Matrix([[x + d*sympy.sin(theta+beta)],
                           [y + d*sympy.cos(theta+beta)],
                           [theta+beta]])

        self.F_j = self.fxu.jacobian(Matrix([x, y, theta]))
        self.V_j = self.fxu.jacobian(Matrix([v, beta]))

        # save dictionary and its variables for later use
        self.subs = {x: 0, y: 0, v:0, beta:0, 
                     time:dt, theta:0}
        self.x_x, self.x_y, = x, y 
        self.v, self.beta, self.theta = v, beta, theta

    def predict(self, u=0):
        self.subs[self.theta] = self.x[2, 0]
        self.subs[self.v] = u[0]
        self.subs[self.beta] = u[1]

        F = array(self.F_j.evalf(subs=self.subs)).astype(float)
        V = array(self.V_j.evalf(subs=self.subs)).astype(float)

        # covariance of motion noise in control space
        M = array([[self.std_vel**2, 0], 
                   [0, self.std_steer**2]])

        self.P = dot(F, self.P).dot(F.T) + dot(V, M).dot(V.T)


def H_of(x):
    """ compute Jacobian of H matrix where h(x) computes 
    the range and bearing to a landmark for state x """
    H = array(
        [[1, 0, 0],
         [0, 1, 0],
         [0, 0, 1],])
    
    return H

def Hx(x):
    """ takes a state variable and returns the measurement
    that would correspond to that state. """
    Hx = array([[x[0, 0]],
                [x[1, 0]],
                [x[2, 0]]])
    
    return Hx

def residual(a, b):
    """ compute residual (a-b) between computed measurement and 
    machine measurement containing [range, bearing]. 
    Bearing is normalized to [-pi, pi)"""
    y = a - b
    y[2] = y[2] % (2 * np.pi)    # force in range [0, 2 pi)
    if y[2] > np.pi:             # move to [-pi, pi)
        y[2] -= 2 * np.pi
    return y

def get_measurement(sim_pos, std_rng, std_brg):
    """ simulate measurement by adding random noise within variance range """
    #sim_pos[0,0] = distance - init_distance
    x, y, theta = sim_pos[0, 0], sim_pos[1, 0], sim_pos[2, 0]
    noised_theta = (theta + randn()*std_brg) % (2 * np.pi)
    if noised_theta > np.pi:  # move to [-pi, pi)
        noised_theta -= 2 * np.pi
    z = np.array([[x + randn()*std_rng],
                  [y + randn()*std_rng],
                  [noised_theta]])
    return z

def reach_goal(current_state, goal_state, tol=.1):
    """ check whether goal state is reached """
    if np.absolute(current_state[0,0] - goal_state[0,0]) < tol and np.absolute(current_state[1,0] - goal_state[1,0]) < tol:
      return True
    else:
      return False

from math import sqrt, tan, cos, sin, atan2
import matplotlib.pyplot as plt
import numpy as np

dt = 0.1

def run_navigation(start_state, goal_state, init_var, tol, std_vel, std_steer, 
                     std_range, std_bearing,
                     step=10, ellipse_step=50, ylim=None):
    ekf = RobotEKF(dt, std_vel=std_vel, 
                   std_steer=std_steer)
    ekf.x = start_state         # x, y, steer angle
    # initialize state variance P and measurement variance R
    ekf.P = np.diag([init_var[0], init_var[1], init_var[2]])
    ekf.R = np.diag([std_range**2, std_range**2, std_bearing**2])

    ideal_state = ekf.x.copy() # simulated position
    
    ideal_track = []
    ekf_track = []

    # simulated movement, i.e. half circle
    while not reach_goal(ekf.x, goal_state, tol=tol):
        # get control
        u = array([1.0, .02])  # steering command (vel, steering angle change)
        # u = get_control(current_state, goal_state)

        # ideal movement of robot
        #ideal_state = ekf.move(ideal_state, u, dt)
        # keep track of the ideal path
        #ideal_track.append(ideal_state)

        # apply noise to control and measurement with EKF
        ekf.predict(u=u)
        z = get_measurement(ekf.x, std_range, std_bearing)
        ekf.update(z, HJacobian=H_of, Hx=Hx, residual=residual)
        # keep track of the EKF path
        ekf_track.append(ekf.x)

    ideal_track = np.array(ideal_track)
    ekf_track = np.array(ekf_track)
    
    while not rospy.is_shutdown():
	#Define values for the visualized goal state
	goal_marker = Marker()
	goal_marker.header.frame_id = "/ekf1"
	goal_marker.type = goal_marker.CUBE	
	goal_marker.action = goal_marker.ADD
	goal_marker.scale.x = 0.3
	goal_marker.scale.y = 0.3
	goal_marker.scale.z = 0.3
	goal_marker.pose.position.x = goal_state[0, 0]
	goal_marker.pose.position.y = goal_state[1, 0]
	goal_marker.pose.position.z = 0
	goal_marker.color.a = 1
	goal_marker.color.r = 0.5
	goal_marker.color.g = 1
	goal_marker.color.b = 0
	goal_marker.pose.orientation.w = 1.0
	goalPublisher.publish(goal_marker)

	#Define values for the ideal trajectory
	idealMarker = Marker()
	idealMarker.header.frame_id = "/ekf1" #sets frame as ekf1
	idealMarker.ns = "ideal_space"
	idealMarker.type = idealMarker.SPHERE
	idealMarker.action = idealMarker.ADD
	idealMarker.scale.x = 0.1
	idealMarker.scale.y = 0.1
	idealMarker.scale.z = 0.05
	idealMarker.color.a = 1
	idealMarker.color.r = 0
	idealMarker.color.g = 0
	idealMarker.color.b = 0.5
	idealMarker.pose.orientation.w = 1.0

	id = 0
	for arr in ideal_track:				
		idealMarker.pose.position.x = arr[0]
		idealMarker.pose.position.y = arr[1]
		idealMarker.pose.position.z = 0

		ideal.markers.append(idealMarker)
		#print(idealMarker.pose.position.x)

		for i in ideal.markers:
      		 	i.id = id
       	         	id += 1		

		idealPublisher.publish(ideal)
		rospy.sleep(0.02)

	# Define trajectory for ekf track
	ekfMarker = Marker()
	ekfMarker.header.frame_id = "/ekf1"
	ekfMarker.ns = "ekf_space"
	ekfMarker.type = ekfMarker.SPHERE
	ekfMarker.action = ekfMarker.ADD
	ekfMarker.scale.x = 0.1
	ekfMarker.scale.y = 0.1
	ekfMarker.scale.z = 0.05
	ekfMarker.color.a = 1
	ekfMarker.color.r = 0.5
	ekfMarker.color.g = 0
	ekfMarker.color.b = 0
	ekfMarker.pose.orientation.w = 1.0

	id = 0
	for arr in ekf_track:				
		ekfMarker.pose.position.x = arr[0]
		ekfMarker.pose.position.y = arr[1]
		ekfMarker.pose.position.z = 0

		ekfArray.markers.append(ekfMarker)
		#print(ekfMarker.pose.position.x)

		for e in ekfArray.markers:
      		 	e.id = id
       	         	id += 1	

		ekfPublisher.publish(ekfArray)
		rospy.sleep(0.02)

    plt.figure()
    plt.scatter(goal_state[0, 0], goal_state[1, 0], marker='s', s=60)
    plt.plot(ideal_track[:,0], ideal_track[:,1], color='b', lw=2)
    plt.plot(ekf_track[:, 0], ekf_track[:,1], color='r', lw=2)
    plt.axis('equal')
    plt.title("EKF Robot localization")
    if ylim is not None: plt.ylim(*ylim)
    plt.show()
    return ekf

start_state = np.array([[2.0, 0.0, 0.0]]).T
goal_state = np.array([[12.0,0.0]]).T
init_var = [0.1, 0.1, 0.01]   # initial std for state variables x, y and theta
	
ekf = run_navigation(
    start_state=start_state, goal_state=goal_state, init_var=init_var, tol=.3,
    std_vel=0.1, std_steer=0.01,
    std_range=0.1, std_bearing=0.01)
print('Legend')
print('Blue: ideal path')
print('Red: EKF path')
print('Square: goal')
print('Final P:', ekf.P.diagonal())
