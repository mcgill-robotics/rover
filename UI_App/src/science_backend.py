from science_module.msg import SciencePilot



class Science_Backend():

    def __init__(self, science_tab):
        self.ui = science_tab
        self.ui.error_label.clear()



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


    ## SCIENCE SECTION
    # Subscribers
    def on_science_feedback(self, msg):
        """
        Callback function when receiving a science message from messaging broker

        :param msg: ScienceFeedback
        """
        self.update_boolean_value(msg.Stepper1Fault, self.ui.stepper1Fault_bool)
        self.update_boolean_value(msg.Stepper2Fault, self.ui.stepper2Fault_bool)
        self.update_boolean_value(msg.PeltierState, self.ui.coolerState_bool)
        self.update_boolean_value(msg.LedState, self.ui.ledState_state_label)
        self.update_boolean_value(msg.LaserState, self.ui.laserState_state_label)
        self.update_boolean_value(msg.GripperState, self.ui.gripperState_state_label)

    # Helpers
    def send_science_shutdown(self):
        """
        Function creating a SciencePilot message for shutdown, setting it to true and sending it through messaging broker
        """
        msg = SciencePilot()
        msg.Shutdown = True
        self.science_module_publisher.publish(msg)

    def send_science_pilot(self):
        """
        Function creating a SciencePilot message and sending request through messaging broker
        """
        msg = SciencePilot()

        self.ui.laserState_toggle.isChecked()

        # Booleans
        msg.LedState = self.get_boolean_value(self.ui.ledState_toggle)
        msg.LaserState = self.get_boolean_value(self.ui.laserState_toggle)
        msg.GripperState = self.get_boolean_value(self.ui.gripperState_toggle)
        msg.PeltierState = self.get_boolean_value(self.ui.peltierState_toggle)
        msg.CcdSensorSnap = self.get_boolean_value(self.ui.ccdSensorSnap_toggle)

        # Float 32
        msg.ContMotorSpeed = self.get_double_spin_box_value(self.ui.contMotorSpeed_doubleSpinBox)
        msg.StepperMotor1Pos = self.get_double_spin_box_value(self.ui.stepper1Pos_doubleSpinBox)
        msg.StepperMotor2Pos = self.get_double_spin_box_value(self.ui.stepper1Pos_doubleSpinBox)
        msg.StepperMotor1Speed = self.get_double_spin_box_value(self.ui.stepper1Speed_doubleSpinBox)
        msg.StepperMotor2Speed = self.get_double_spin_box_value(self.ui.stepper2Speed_doubleSpinBox)

        # UInt 32
        msg.Stepper1ControlMode = self.get_uint_spin_box_value(self.ui.stepper1ControlMode_spinBox)
        msg.Stepper2ControlMode = self.get_uint_spin_box_value(self.ui.stepper2ControlMode_spinBox)

        self.science_module_publisher.publish(msg)
