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
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import String

import pynmea2
from geodesy import utm

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance
from tf.transformations import euler_from_quaternion


''' BU353-S4 SENSOR DECLARATIONS '''
# Variables for UTM point (easting and northing in meters)
gps_easting = 0
gps_northing = 0

''' UM7 SENSOR DECLARATIONS '''
# Create variables for roll, pitch, yaw; initialize in callback
um7_roll = 0
um7_pitch = 0
um7_yaw = 0

''' TRACKING CAM T265 DECLARATIONS '''
# Variables for cam's roll, pitch, yaw, position?
t265_pose = PoseWithCovariance()
t265_roll = 0
t265_pitch = 0
t265_yaw = 0

''' RVIZ STUFF '''
# Goal marker topic
topic = 'visualization_marker'
publisher = rospy.Publisher(topic, Marker)

# Ekf track topic
ekf_topic = 'visualization_marker_array'
ekfPublisher = rospy.Publisher(ekf_topic, MarkerArray)

ekfArray = MarkerArray()

''' INIT ROS NODE AND CALLBACKS '''
# Initialize ekf node to appear in rosnode list
rospy.init_node("ekf_node")

### (UM7 IMU) ###   
# Callback for UM7 IMU sensor (pose contains position for x y z, and orientation for x y z w)
def imuCall(um7Data):
	imu_data = um7Data.data

    # assigns array values to um7 roll, pitch and yaw variables
	global um7_roll, um7_pitch, um7_yaw
	um7_roll, um7_pitch, um7_yaw = imu_data[0], imu_data[1], imu_data[2]

# Subscribes to um7_data topic (UM7 IMU sensor)
def imuSub():
    um7Read = rospy.Subscriber("um7_data", Int32MultiArray, imuCall)

### (BU353-S4 GPS) ###
# Callback for GPS (String parsed and converted to NMEA sentence)
def gpsCall(buData):
	gps_msg = pynmea2.parse(buData.data) # stores data in a message

	utmPt = utm.fromLatLong(gps_msg.latitude, gps_msg.longitude, gps_msg.altitude)

	# assigns easting and northing values from UTMPoint object
	global gps_easting, gps_northing
	gps_easting, gps_northing = utmPt.easting, utmPt.northing
	
def gpsSub():
	buRead = rospy.Subscriber("bu353_data", String, gpsCall)	

### (Tracking Camera T265) ###
# Callback for tracking cam (Odometry message published by RealSense)
# Pose contains position for x y z, and orientation for x y z w; might have to change frame
def camCall(t265Data):
	global t265_pose
	t265_pose = t265Data.pose # pose with covariance object (has a float64 array and Pose() object)

	# potentially add stuff for position here

	# assign roll, pitch and yaw given
	global t265_roll, t265_pitch, t265_yaw
	quaternion_list = [t265_pose.pose.orientation.x, t265_pose.pose.orientation.y, t265_pose.pose.orientation.z, t265_pose.pose.orientation.w]
	(t265_roll, t265_pitch, t265_yaw) = euler_from_quaternion(quaternion_list) # convert to euler angles

def camSub():
	t265Read = rospy.Subscriber("camera/odom/sample", Odometry, camCall)

class RobotEKF(EKF):
    # initializes motion model with standard dev. of velocity and time step
    def __init__(self, dt, std_vel):
	# uses filterpy.kalman to create a three-dimensional state with 3 measurements (for x and y; obtain yaw with pose orientation?)
	EKF.__init__(self, 3, 3, 2)
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
	# uses velocity as control input
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
	# rotation added, angle must stay within 0 and 180 degrees (-pi and pi radians)
	y = a - b
	return y

# pose variable contains Pose() given by IMU sensor, takes x and y positions; *** 
def get_sensor_reading():
    # obtains current x position; distance measured is the same value
    z = [[um7_roll],
         [um7_pitch],
	 [um7_yaw]]
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
	u = array([0.1, 0.2]) # steering command (constant velocity of 0.1m/s)
        ekf.predict(u=u)	# predict function with assigned steering command
        z = get_sensor_reading() # measurement function that gives position from sensor
        ekf.update(z, HJacobian = Hj, Hx=Hx, residual=residual)		# update function
        
	# keep track of the EKF path
	# ekf_track.append(ekf.x)
	# rospy.loginfo(ekf.x) # for debugging purposes 
	print(t265_pitch)

	# calls subscriber to get imu, gps and tracking cam readings
	imuSub()
	gpsSub()
	camSub()

	# RVIZ sim (needs improvement, still displays old markers)
	#Define values for the visualized goal state
	goal = Marker()
	goal.header.frame_id = "/ekf"
	goal.type = goal.SPHERE	
	goal.action = goal.ADD

	goal.scale.x = 0.55
	goal.scale.y = 0.55
	goal.scale.z = 0.55

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
	ekfMarker.scale.x = 0.2
	ekfMarker.scale.y = 0.2
	ekfMarker.scale.z = 0.05
	ekfMarker.color.a = 1
	ekfMarker.color.r = 0.5
	ekfMarker.color.g = 0
	ekfMarker.color.b = 0
	ekfMarker.pose.orientation.w = 1.0

	id = 0				
	ekfMarker.pose.position.x = ekf.x[0]
	ekfMarker.pose.position.y = 0
	ekfMarker.pose.position.z = 0
	
	# Should remove the first entry in array after count reaches past 5
	if (markerCount > 5):
	   ekfArray.markers.pop(0)
	   markerCount = 0

	ekfArray.markers.append(ekfMarker)
	# print(markerCount)

	for e in ekfArray.markers:
	 	e.id = id
         	id += 1	

	ekfPublisher.publish(ekfArray)
	markerCount += 1	

	rospy.sleep(0.02)

start_state = np.array([[-0.0662, -0.0706, 0.2571]]).T # start state of robot is defined at the initial distance measured (according to sim)
# goal_state = np.array([0.0]) # goal state is set as the origin in the frame
init_var = [0.1, 0.1, 0.1]   # initial std for state variable x, y and theta
	
ekf = run_navigation(start_state=start_state, init_var = init_var, std_vel = 0.01, std_range = 0.108, std_bearing = 0.02) # starts navigation with set values

