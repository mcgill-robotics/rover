# from embedded_bridge.msg import PowerFeedback


class Power_Backend():

    def __init__(self, power_tab):
        self.ui = power_tab
        

    @staticmethod
    def update_float_value(value, label):
        label.display("%.2f" % float(value))

    # Callback for battery currents.
    def power_feedback(self, msg):

        self.update_float_value(msg.data[0], self.ui.current_value_1)
        self.update_float_value(msg.data[1], self.ui.current_value_2)

    # Callback for system feedbacks.
    def on_kill_power(self, msg):

        self.update_float_value(msg.data[0], self.ui.wheel_left_front_value)
        self.update_float_value(msg.data[1], self.ui.wheel_left_back_value)
        self.update_float_value(msg.data[2], self.ui.wheel_right_front_value)
        self.update_float_value(msg.data[3], self.ui.wheel_right_back_value)
        self.update_float_value(msg.data[4], self.ui.lower_arm_value)
        self.update_float_value(msg.data[5], self.ui.upper_arm_value)
        self.update_float_value(msg.data[6], self.ui.science_motors_value)
        self.update_float_value(msg.data[7], self.ui.science_steppers_value)
        