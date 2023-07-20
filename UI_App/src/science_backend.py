import datetime
import matplotlib.pyplot as plt
import numpy as np
import os
from std_msgs.msg import Float32MultiArray
import time


class Science_Backend():

    def __init__(self, science_tab):
        self.ui = science_tab
        
        # Moisture Sensor sample variables
        self.num_moisture_sensors = 4
        self.num_moisture_samples = 10000
        self.moisture_samples = [[] for _ in range(self.num_moisture_sensors)]
        self.ui.moisture_sampleCnt_spinbox.valueChanged.connect(self.on_moisture_sample_display_change)

        # Logger for sensor data
        self.science_log_path = os.path.dirname(__file__) + f"/../../rover_out/{datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}"
        os.makedirs(self.science_log_path)
        self.science_log_filename = self.science_log_path + "/science_moisture_data.log"

        # Motor variables
        self.carousel_angle_position = 0
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

    def on_moisture_sample_display_change(self):
        self.num_moisture_samples = self.ui.moisture_sampleCnt_spinbox.value()

    def on_science_feedback(self, msg):
        """
        Callback function when receiving a science message from messaging broker

        :param msg: Float32MultiArray
        """
        # Log new data feedback
        log_file = open(self.science_log_filename, "a")
        log_file.write(f"{time.time()},{msg.data[0]},{msg.data[1]},{msg.data[2]},{msg.data[3]}\n")
        log_file.close()

        # Plot new data sample
        self.ui.moisture_data_fig.clear()
        ax = self.ui.moisture_data_fig.add_subplot(111)

        for i in range(self.num_moisture_sensors):
            # pop oldest sample
            while not (len(self.moisture_samples[i]) < self.num_moisture_samples):
                self.moisture_samples[i].pop(0)
            # add new sample
            self.moisture_samples[i].append(msg.data[i])
            # Update plot
            N = len(self.moisture_samples[i])
            ax.plot(np.linspace(0, N-1, N), self.moisture_samples[i], label=f"Moisture {i+1}")
        
        ax.set_ylabel("Moisture \%")
        ax.set_xlabel("time")
        ax.legend()
        self.ui.moisture_plotter.draw()


    def set_science_cmd(self):
        """
        Function creating a Float32MultiArray message and sending request through messaging broker
        """
        msg = Float32MultiArray()
        self.carousel_angle_position += self.ui.carousel_position_inc_spinbox.value()
        self.update_float_value(self.carousel_angle_position, self.ui.carousel_position_fb_lcd)

        # Float 32
        msg.data.append(self.get_double_spin_box_value(self.ui.carousel_position_inc_spinbox))
        msg.data.append(self.get_double_spin_box_value(self.ui.auger_speed_slider))
        msg.data.append(self.get_double_spin_box_value(self.ui.linear_guide_slider))

        return msg
