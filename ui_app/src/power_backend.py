# from embedded_bridge.msg import PowerFeedback
from std_msgs.msg import Float32MultiArray
import rospy


class Power_Backend():

    def __init__(self, power_tab):
        self.ui = power_tab

        self.drive_on = 0.0
        self.science_on = 0.0
        self.lower_arm_on = 0.0
        self.upper_arm_on = 0.0

        self.system_publisher = Float32MultiArray()
        self.system_publisher.data.append(self.drive_on)
        self.system_publisher.data.append(self.science_on)
        self.system_publisher.data.append(self.lower_arm_on)
        self.system_publisher.data.append(self.upper_arm_on)

        self.power_on_publisher = rospy.Publisher("/powerCmd", Float32MultiArray, queue_size=1)

    @staticmethod
    def update_float_value(value, label):
        label.display("%.2f" % float(value))
    
    def get_drive_enabled(self):
        if self.ui.drive_enabled.isChecked():
            self.drive_on = 1.0
        else:
            self.drive_on = 0.0
        
        self.system_publisher.data = [self.drive_on, self.science_on, self.lower_arm_on, self.upper_arm_on]
        self.power_on_publisher.publish(self.system_publisher)
        
    def get_science_enabled(self):
        if self.ui.science_enabled.isChecked():
            self.science_on = 1.0
        else:
            self.science_on = 0.0

        self.system_publisher.data = [self.drive_on, self.science_on, self.lower_arm_on, self.upper_arm_on]
        self.power_on_publisher.publish(self.system_publisher)
    
    def get_lower_arm_enabled(self):
        if self.ui.lower_arm_enabled.isChecked():
            self.lower_arm_on = 1.0
        else:
            self.lower_arm_on = 0.0

        self.system_publisher.data = [self.drive_on, self.science_on, self.lower_arm_on, self.upper_arm_on]
        self.power_on_publisher.publish(self.system_publisher)
    
    def get_upper_arm_enabled(self):
        if self.ui.upper_arm_enabled.isChecked():
            self.upper_arm_on = 1.0
        else:
            self.upper_arm_on = 0.0

        self.system_publisher.data = [self.drive_on, self.science_on, self.lower_arm_on, self.upper_arm_on]
        self.power_on_publisher.publish(self.system_publisher)

    # Callback for battery currents.
    def power_feedback(self, msg):

        self.update_float_value(msg.data[0], self.ui.current_value_1)
        self.update_float_value(msg.data[1], self.ui.current_value_2)

    # Callback for system feedbacks.
    def system_feedback(self, msg):

        self.update_float_value(msg.data[0], self.ui.wheel_left_front_value)
        self.update_float_value(msg.data[1], self.ui.wheel_left_back_value)
        self.update_float_value(msg.data[2], self.ui.wheel_right_front_value)
        self.update_float_value(msg.data[3], self.ui.wheel_right_back_value)
        self.update_float_value(msg.data[4], self.ui.lower_arm_value)
        self.update_float_value(msg.data[5], self.ui.upper_arm_value)
        self.update_float_value(msg.data[6], self.ui.science_motors_value)
        self.update_float_value(msg.data[7], self.ui.science_steppers_value)
        