from std_msgs.msg import Float32MultiArray


class Science_Backend():

    def __init__(self, science_tab):
        self.ui = science_tab
        self.num_moisture_sensors = 4
        self.moisture_samples = [[] for _ in range(self.num_moisture_sensors)]
        self.ui.auger_speed_slider.valueChanged.connect(self.update_dynamic_value(self.ui.auger_speed_slider, self.ui.auger_speed_fb_lcd))
        self.ui.linear_guide_slider.valueChanged.connect(self.update_dynamic_value(self.ui.linear_guide_slider, self.ui.linear_guide_fb_lcd))

    ## READER FUNCTIONS
    @staticmethod
    def get_boolean_value(button):
        return button.isChecked()

    @staticmethod
    def get_double_spin_box_value(double_spin_box):
        return float(double_spin_box.value())

    @staticmethod
    def get_uint_spin_box_value(spin_box):
        return int(spin_box.value())

    ## UPDATER FUNCTIONS
    @staticmethod
    def update_boolean_value(condition, label):
        label.setText("Enabled" if condition else "Disabled")

    @staticmethod
    def update_float_value(value, label):
        label.display("%.2f" % float(value))

    @staticmethod
    def update_dynamic_value(dynamic_input, dynamic_output):
        def update_value():
            dynamic_output.display(f"{int(dynamic_input.value())}")
        return update_value


    def on_science_feedback(self, msg):
        """
        Callback function when receiving a science message from messaging broker

        :param msg: Float32MultiArray
        """
        for i in range(self.num_moisture_sensors):
            self.moisture_samples[i].append(msg.data[i])
        # Add code to update plot
    
    def set_science_cmd(self):
        """
        Function creating a Float32MultiArray message and sending request through messaging broker
        """
        msg = Float32MultiArray()

        # Float 32
        msg.data.append(self.get_double_spin_box_value(self.ui.carousel_position_inc_spinbox))
        msg.data.append(self.get_double_spin_box_value(self.ui.auger_speed_slider))
        msg.data.append(self.get_double_spin_box_value(self.ui.linear_guide_slider))

        return msg

    