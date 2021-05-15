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
from geometry_msgs.msg import Pose

from tf.transformations import euler_from_quaternion

# Declare distance and pose variables
distance = 0
pose = Pose() # automatically sets all values to zero

# Create variables for roll, pitch, yaw; initialize in callback
roll = 0
pitch = 0
yaw = 0

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
    
# Callback for simulated IMU sensor (pose contains position for x y z, and orientation for x y z w)
def imuCall(poseData):
    global pose
    pose = poseData
    
    # converts quaternion values into euler angles using tf.transformations
    global roll, pitch, yaw
    quaternion_list = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    (roll, pitch, yaw) = euler_from_quaternion(quaternion_list)
    

# Subscribes to rover_pose topic (simulated IMU sensor)
def imuSub():
    poseRead = rospy.Subscriber("rover_pose", Pose, imuCall)
    
class RobotEKF(EKF):
    # initializes motion model with standard dev. of velocity and time step
    def __init__(self, dt, std_vel):
	# uses filterpy.kalman to create a three-dimensional state with 3 measurements (for x and y; obtain yaw with pose orientation?)
	EKF.__init__(self, 3, 3, 1)
	self.dt = dt
	self.std_vel = std_vel
	
	# defines symbols of x pos., y pos., theta (bearing angle), velocity,  and time 
	x, y, theta, v, time = symbols('x, y, theta, v, t')
	d = v * time
	
	# motion model (focusing only on x position, y position distance travelled in component form, and pose angle about z-axis?) ***
	self.fxu = Matrix([[x + d*sympy.cos(theta)],
			   [y + d*sympy.sin(theta)],
                           [theta]])
	self.F_j = self.fxu.jacobian(Matrix([x, y, theta]))
	self.V_j = self.fxu.jacobian(Matrix([v]))
	
	# save dictionary and its variables for later use
	self.subs = {x: 0, y: 0, theta: 0, v:0, time:dt}

        self.pos_x = x
	self.pos_y = y
	self.theta = theta
        self.v = v
	
    def predict(self, u=0):
	# uses angular velocity as control input
	self.subs[self.v] = u[0]
	
	# converts arrays to floating-point approx.
	F = array(self.F_j.evalf(subs=self.subs)).astype(float)
        V = array(self.V_j.evalf(subs=self.subs)).astype(float)

	# covariance of motion noise in control space
        M = array([[self.std_vel**2]])

	# total motion noise (take noise related to position/orientation and adds it to noise related to control input)
        self.P = dot(F, self.P).dot(F.T) + dot(V, M).dot(V.T)

# jacobian of sensor model (assumes measurements are exact)
def Hj(x):
    """ compute Jacobian of H matrix where h(x) computes 
    the range and bearing to a landmark for state x """
    H = array([[1, 0, 0],
	       [0, 1, 0],
               [0, 0, 1]])
    
    return H

# sensor/measurement model (returns the state variable)	 
def Hx(x):
    """ takes a state variable and returns the measurement
    that would correspond to that state. """
    Hx = array([[x[0, 0]],
                [x[1, 0]],
	        [x[2, 0]]])

    return Hx

# what was actually measured minus what the measurement was supposed to be (***)
def residual(a, b):
	# rotation added, should not change
	y = a - b
	return y

# pose variable contains Pose() given by IMU sensor, takes x and y positions; *** 
def get_sensor_reading():
    # obtains current x position; distance measured is the same value
    z = [[pose.position.x],
         [pose.position.y],
	 [yaw]]
    rospy.loginfo("Measurement")
    rospy.loginfo(z)
    return z

# verify whether wall/goal is reached (*tested, works*); *** will need to change for IMU (talk to Kieran about how it should look)
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


def run_navigation(start_state, init_var, std_vel, std_range, std_bearing):
    ekf = RobotEKF(dt, std_vel=std_vel)		# initializes new RobotEKF with given time step, std. of velocity
    ekf.x = start_state         # x position, y position, bearing angle (starting pos.)

    # initialize state variance P and measurement variance R
    ekf.P = np.diag([init_var[0], init_var[1], init_var[2]])
    ekf.R = np.diag([std_range**2, std_range**2, std_bearing**2]) # variance in range of sensor and bearing angle 

    # array containing the positions of the robot	
    ekf_track = []

    markerCount = 0

    while not rospy.is_shutdown():
	u = array([5.0]) # steering command (constant velocity of ? m/s)
        ekf.predict(u=u)	# predict function with assigned steering command
        z = get_sensor_reading() # measurement function that gives position from sensor
        ekf.update(z, HJacobian = Hj, Hx=Hx, residual=residual)		# update function
        
	# keep track of the EKF path
	ekf_track.append(ekf.x)
	rospy.loginfo("State")
	rospy.loginfo(ekf.x) # for debugging purposes 

	# calls subscriber to get distance readings
        ultraSub()
	imuSub()

	
	# RVIZ sim 
	#Define values for the visualized goal state (potentially unneeded for sim)
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
	ekfMarker.pose.position.y = ekf.x[1]
	ekfMarker.pose.position.z = 0
	
	# Should remove the first entry in array after count reaches past 5
	if (markerCount > 10):
	   ekfArray.markers.pop(0)

	ekfArray.markers.append(ekfMarker)
	
	# Renumber id to remove old markers
	id = 0
	for e in ekfArray.markers:
	 	e.id = id
         	id += 1	

	ekfPublisher.publish(ekfArray)
	markerCount += 1	

	rospy.sleep(0.02)

start_state = np.array([[0, 0, 0]]).T # start state of robot is defined at the initial distance measured (according to sim)
# goal_state = np.array([0.0]) # goal state is set as the origin in the frame
init_var = [0.001, 0.001, 0.002]   # initial std for state variable x, y and theta
	
ekf = run_navigation(start_state=start_state, init_var = init_var, std_vel = 0.001, std_range = 0.002, std_bearing = 0.002) # starts navigation with set values

