#!/usr/bin/env python

import numpy as np
from numpy.linalg import inv
import math

import rospy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import String

import pynmea2
import time
from geodesy import utm

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance

''' BU353-S4 SENSOR DECLARATIONS '''
# Variables for UTM point (easting and northing in meters)
gps_easting = 0
gps_northing = 0

# initial values for easting/northing to get displacement
init_easting = 0
init_northing = 0

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

t265_x = 0
t265_y = 0
t265_z = 0

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
    um7_roll, um7_pitch, um7_yaw = imu_data[0]/91.02222, imu_data[1]/91.02222, imu_data[2]/91.02222

    # sets them within range of 0 to 360 deg.
    if um7_yaw < 0: um7_yaw += 360.0
    if um7_roll < 0: um7_roll += 360.0
    if um7_pitch < 0: um7_pitch += 360.0

# Subscribes to um7_data topic (UM7 IMU sensor)
def imuSub():
    um7Read = rospy.Subscriber("um7_data", Int32MultiArray, imuCall)

### (BU353-S4 GPS) ###
# Callback for GPS (String parsed and converted to NMEA sentence)
def gpsCall(buData):
    gps_msg = pynmea2.parse(buData.data) # stores data in a message

    utmPt = utm.fromLatLong(gps_msg.latitude, gps_msg.longitude, gps_msg.altitude)

    # assigns easting and northing values from UTMPoint object
    global gps_easting, gps_northing, init_easting, init_northing

    # assigns initial east/north to decrement
    if init_easting == 0 and init_northing == 0:
        init_easting = utmPt.easting
        init_northing = utmPt.northing

    gps_easting, gps_northing = utmPt.easting, utmPt.northing # obtain current easting and northing values...
    gps_easting -= init_easting # ...then decrement them to zero values
    gps_northing -= init_northing
	
def gpsSub():
	buRead = rospy.Subscriber("bu353_data", String, gpsCall)	

### (Tracking Camera T265) ###
# Callback for tracking cam (Odometry message published by RealSense)
# Pose contains position for x y z, and orientation for x y z w; might have to change frame
def camCall(t265Data):
    global t265_pose
    t265_pose = t265Data.pose # pose with covariance object (has a float64 array and Pose() object)

    # potentially add stuff for position here
    global t265_x, t265_y, t265_z
    t265_x, t265_y, t265_z = t265_pose.pose.position.x, t265_pose.pose.position.y, t265_pose.pose.position.z

    # assign roll, pitch and yaw given
    global t265_roll, t265_pitch, t265_yaw

    # simplifies work by assigning x, y, z, w to their respective orientation values
    w = t265_pose.pose.orientation.w
    x = t265_pose.pose.orientation.x
    y = t265_pose.pose.orientation.y
    z = t265_pose.pose.orientation.z

    # calculates pitch, roll and yaw based on cam's reference frame (obtained from t265_rpy.py)
    t265_pitch = -math.asin(2.0 * (x*z - w*y)) * 180.0 / math.pi
    t265_roll = math.atan2(2.0 * (w*x + y*z), w*w - x*x - y*y + z*z) * 180.0 / math.pi
    t265_yaw = math.atan2(2.0 * (w*z + x*y), w*w + x*x - y*y - z*z) * 180.0 / math.pi

    # sets them within range of 0 to 360 deg.
    if t265_yaw < 0: t265_yaw += 360.0
    if t265_roll < 0: t265_roll += 360.0
    if t265_pitch < 0: t265_pitch += 360.0

    # print("RPY [deg]: Roll: {0:.7f}, Pitch: {1:.7f}, Yaw: {2:.7f}".format(t265_roll, t265_pitch, t265_yaw))


def camSub():
	t265Read = rospy.Subscriber("camera/odom/sample", Odometry, camCall)


class RobotEKF():
    # initializes motion model with standard dev. of velocity and time step
    def __init__(self, state_init, std_vel, std_rot): 
        # create a three-dimensional state with 3 measurements (for x, y and theta; yaw) => state_init
        self.std_vel = std_vel # standard dev. of velocity and rotation
        self.std_rot = std_rot
        
        self.R_cam = np.array([[0.4,0,0], # covariance matrix for measurement noise of the tracking camera
                             [0,0.4,0],
                             [0,0,0.9]])
        
        self.R_gpIMU = np.array([[1.5, 0, 0], # covariance matrix for measurement noise of the gps and IMU readings
                                 [0, 1.5, 0],
                                 [0, 0, 0.4]])

        self.Q = np.array([[self.std_vel**2,0,0], # covariance for process noise (i.e. states)
                           [0,self.std_vel**2,0],
                           [0,0,self.std_rot**2]])
        self.P = self.Q
        
        self.state_est = state_init # initialize estimated state with parameter
        self.time_prev = time.time() # current time
        
    def get_F(self, theta_prev, dt, v): 
        """
        fxu = Matrix([[x + d*sympy.sin(theta)],
                        [y + d*sympy.cos(theta)],
                        [theta +d*omega]])
        return = fxu.jacobian(Matrix([x, y, theta])) 
        """
        # jacobian of get_predict_state ^
        F = np.array([[1,0,-dt*v*math.sin(theta_prev)],
                         [0,1,dt*v*math.cos(theta_prev)],
                         [0,0,1]])
        
        return F
    
    def get_H(self): # jacobian of sensor/measurement model (identity since they are exact)
        H = np.array([[1, 0, 0],
               [0, 1, 0],
               [0, 0, 1]])
        
        return H
    
    def get_predict_state(self, x_prev, y_prev, theta_prev, dt, v, omega): # motion model
        #Omega = angular velocity
        #v = forward velocity
        state_pred = np.array([x_prev + dt*v*math.cos(theta_prev),
                        y_prev + dt*v*math.sin(theta_prev),
                        theta_prev + dt*omega])
        
        return state_pred
    
    # sensor readings (GPS, tracking cam and IMU; x, y, theta)
    # for the first update step
    def get_cam_measurement(self):
        z = np.array([[t265_x],
             [t265_y],
             [t265_yaw]])
        return z

    # for the second update step
    def get_gpsIMU_measurement(self):
        z = np.array([[gps_easting],
                     [gps_northing],
                     [um7_yaw]])
        return z

    def predict_update(self, u=[0,0]):
        #Predict step
        dt = time.time() - self.time_prev # calculate time step
        self.time_prev = time.time() # resets current time
        state_pred = self.get_predict_state(x_prev = self.state_est[0], y_prev = self.state_est[1], theta_prev = self.state_est[2], dt = dt, v = u[0], omega = u[1]) # predicted state estimate

        F = self.get_F(theta_prev = self.state_est[2], dt = dt, v = u[0]) # state transition matrix
        H = self.get_H() # observation matrix
        
        P_pred = np.matmul(np.matmul(F,self.P),F.T) + self.Q # predicted covariance estimate (used for first update)

        ################################
        #First update step
        first_state_meas = self.get_cam_measurement() # get camera readings
        first_y_res = first_state_meas - np.matmul(H,state_pred) # first measurement residual (for cam)
        
        firstS_k = np.matmul(np.matmul(H,P_pred),H.T) + self.R_cam # first residual covariance
        firstS_k = firstS_k.astype('float')
        firstK_k = np.matmul(np.matmul(P_pred,H.T),inv(firstS_k)) # initial Kalman gain
        
        init_state_est = state_pred + np.matmul(firstK_k, first_y_res) # first updated state estimate (ekf.x)
        init_P = np.matmul((np.eye(3) - np.matmul(firstK_k, H)),P_pred) # first updated covariance estimate

        ################################
        #Second (and final) update step
        second_state_meas = self.get_gpsIMU_measurement() # get gps and imu readings
        second_y_res = second_state_meas - np.matmul(H, init_state_est) # second measurement resid. (takes prev. state estimate)

        secondS_k = np.matmul(np.matmul(H, init_P), H.T) + self.R_gpIMU # uses first covariance estimate init_P ^^^
        secondS_k = secondS_k.astype('float')
        secondK_k = np.matmul(np.matmul(init_P,H.T),inv(secondS_k)) # final Kalman gain

        self.state_est = init_state_est + np.matmul(secondK_k, second_y_res) # updated + final state estimate
        self.P = np.matmul((np.eye(3) - np.matmul(secondK_k, H)), init_P) # updated + final covariance estimate



def run_localization(start_state, std_vel, std_rot):
    ekf = RobotEKF(start_state, std_vel, std_rot) # creates new EKF object with defined variables

    markerCount = 0

    # calls subscribers to get measurements from sensor nodes
    imuSub()
    gpsSub()
    camSub()

    while not rospy.is_shutdown():
        u = np.array([0.00, 0.00]) # steering commands (linear x and angular velocity; v and omega)
        ekf.predict_update(u=u)

        rospy.loginfo(ekf.state_est)

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
                    
        ekfMarker.pose.position.x = ekf.state_est[0]
        ekfMarker.pose.position.y = ekf.state_est[1]
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

start_state = np.array([[0, 0, 0]]).T
ekf = run_localization(start_state = start_state, std_vel = 0.99, std_rot = 0.99)

        

