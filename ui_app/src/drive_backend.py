from drive_layout import Ui_DriveTab
from std_msgs.msg import Float32MultiArray
import rospy
import math


class Drive_Backend():

    def __init__(self, drive_tab):
        self.ui = drive_tab

        self.latitude = 0.0
        self.longitude = 0.0
        self.heading = 0.0

        self.antenna_message = Float32MultiArray()
        self.antenna_message.data.append(self.latitude)
        self.antenna_message.data.append(self.longitude)
        self.antenna_message.data.append(self.heading)

        self.antenna_publisher = rospy.Publisher("/antennaData", Float32MultiArray, queue_size=1)

    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians


    def update_wheel_velocities(self, wheel_velocity):
        self.ui.left_speed_data.display(wheel_velocity.left[0])
        self.ui.right_speed_data.display(wheel_velocity.right[0])
        self.ui.rear_left_speed_data.display(wheel_velocity.left[1])
        self.ui.rear_right_speed_data.display(wheel_velocity.right[1])

    def update_twist_data(self, robot_twist):
        self.ui.twist_angular_velocity.display(robot_twist.angular.z)
        self.ui.twist_linear_velocity.display(robot_twist.linear.x)

    def update_robot_location(self, location):
        self.ui.robot_location_x.display(location.position.x)
        self.ui.robot_location_y.display(location.position.y)
        self.ui.robot_location_z.display(location.position.z)
        qx = location.orientation.x
        qy = location.orientation.y
        qz = location.orientation.z
        qw  =location.orientation.w
        roll,pitch,yaw = self.euler_from_quaternion(qx, qy, qz, qw)
        self.ui.robot_location_roll.display(roll)
        self.ui.robot_location_pitch.display(pitch)
        self.ui.robot_location_yaw.display(yaw)
    
    def on_send(self):
        latitude = self.ui.latitude_val.text()
        latitude = float(latitude)

        longitude = self.ui.longitude_val.text()
        longitude = float(longitude)

        heading = self.ui.heading_val.text()
        heading = float(heading)

        self.antenna_message.data[0] = latitude
        self.antenna_message.data[1] = longitude
        self.antenna_message.data[2] = heading
        self.antenna_publisher.publish(self.antenna_message)



