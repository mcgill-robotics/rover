# from embedded_bridge.msg import PowerFeedback


class Power_Backend():

    def __init__(self, power_tab):
        self.ui = power_tab
        

    @staticmethod
    def update_float_value(value, label):
        label.display("%.2f" % float(value))

    
    def on_power_feedback(self, msg):
        self.update_float_value(msg.VoltageBattery1, self.ui.voltage_value_1)
        self.update_float_value(msg.CurrentBattery1, self.ui.current_value_1)
        self.update_float_value((float(msg.CurrentBattery1) * float(msg.VoltageBattery1)), self.ui.current_value_1)

        self.update_float_value(msg.VoltageBattery2, self.ui.voltage_value_2)
        self.update_float_value(msg.CurrentBattery2, self.ui.current_value_2)
        self.update_float_value((float(msg.CurrentBattery2) * float(msg.VoltageBattery2)), self.ui.current_value_2)
        # TODO: Battery lifetime, System enables, kill switch enabled local var?


    def on_kill_power(self):
        self.power_kill_toggle(True)
        # TODO: send kill power


    def power_kill_toggle(self, signal):
        self.power_killed = signal
        self.ui.kill_switch_bool.setText("Emergency Stop" if signal else "Active")
        # TODO: Change system enabled?