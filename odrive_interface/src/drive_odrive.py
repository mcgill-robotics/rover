import os, sys
currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(currentdir)

# TODO: Figure out why catkin on the Jetson isn't playing nice with this import. It worked on my PC -Eren
# import init_functions
import rospy
import odrive_utils
from drive_control.msg import WheelSpeed
from odrive_interface.msg import MotorError, MotorState
import odrive
from odrive.enums import AxisState, ProcedureResult

"""
Placeholders for now. In enumerate_motors(), these motors will be returned in this order. Serial numbers are also strings,
because ODrive API requires them to be that way. Note: correct serial numbers are hex values, while odrivetool returns
decimal. So before putting the numbers in here, you should first convert them to hex.

drive_ids and arm_ids exist to make reverse searching easier. Compared to iterating through the original array values to
find keys, this is an ugly but efficient solution.
"""

# TODO: Once drive is working well, expand this node to include the three arm motors
class Node_OD_Drive():
    def __init__(self):
        self.drive_lb = None
        self.drive_rb = None
        self.lb_speed_cmd = 0.0
        self.rb_speed_cmd = 0.0
        self.lf_speed_cmd = 0.0
        self.rf_speed_cmd = 0.0

        self.active_errors = {"DRIVE_LB": 0, "DRIVE_LF": 0, "DRIVE_RB": 0, "DRIVE_RF": 0}

        # Subscriptions
        rospy.init_node('odrive_interface')
        self.feedback_publisher = rospy.Publisher("/wheel_velocity_feedback", WheelSpeed, queue_size=1)
        self.error_publisher = rospy.Publisher("/odrive_error", MotorError, queue_size=1)
        self.state_publisher = rospy.Publisher("/odrive_state", MotorState, queue_size=1)
        self.command_subscriber = rospy.Subscriber("/wheel_velocity_cmd", WheelSpeed, self.handle_drive_command)
        # TODO: Determine what metacommands software want to use
        # self.metacommand_subscriber = rospy.Subscriber("/odrive_meta_cmd")
        

        # Frequency of the ODrive I/O
        self.rate = rospy.Rate(100)
        self.run()

    def handle_drive_command(self, command):
        self.lb_speed_cmd = command.left[0] * 0.159155
        self.lf_speed_cmd = command.left[1] * 0.159155
        self.rb_speed_cmd = command.right[0] * 0.159155
        self.rf_speed_cmd = command.right[1] * 0.159155


    def run(self):
        # Discover motors on startup and assign variables for each
        motors_dict = odrive_utils.enumerate_motors()
        if not motors_dict:
            raise IOError("Motor enumeration failed")
        
        drive_lb = motors_dict["DRIVE_LB"]
        drive_lf = motors_dict["DRIVE_LF"]
        drive_rb = motors_dict["DRIVE_RB"]
        drive_rf = motors_dict["DRIVE_RF"]
        drive_motors = [drive_lb, drive_lf, drive_rb, drive_rf]

        # Calibrate all motors
        if not odrive_utils.calibrate_motors(drive_motors):
            raise IOError("Motor initialization failed")
        
        while not rospy.is_shutdown():
            # Put in speed command
            drive_lb.axis0.controller.input_vel = -self.lb_speed_cmd
            drive_lf.axis0.controller.input_vel = -self.lf_speed_cmd
            drive_rb.axis0.controller.input_vel = self.rb_speed_cmd
            drive_rf.axis0.controller.input_vel = self.rf_speed_cmd

            # Get feedback and publish it to "/wheel_velocity_feedback"
            feedback = WheelSpeed()
            measured_speed_lb = -drive_lb.encoder_estimator0.vel_estimate
            measured_speed_lf = -drive_lf.encoder_estimator0.vel_estimate
            measured_speed_rb = drive_rb.encoder_estimator0.vel_estimate
            measured_speed_rf = drive_rf.encoder_estimator0.vel_estimate

            feedback.left[0], feedback.left[1] = measured_speed_lb, measured_speed_lf
            feedback.right[0], feedback.right[1] = measured_speed_rb, measured_speed_rf
            self.feedback_publisher.publish(feedback)

            print(f"\rDRIVE_LB: {round(measured_speed_lb, 2)}, DRIVE_LF: {round(measured_speed_lf, 2)}, \
                  DRIVE_RB: {round(measured_speed_rb, 2)}, DRIVE_RF: {round(measured_speed_rf, 2)}", end='')

            odrive_utils.check_odriveErrors(drive_motors)


            self.rate.sleep()
        
        # On shutdown, bring motors to idle state
        print("Shutdown prompt received. Setting all motors to idle state.")
        for motor in drive_motors:
            motor.axis0.requested_state = AxisState.IDLE


if __name__ == "__main__":
    driver = Node_OD_Drive()
    rospy.spin()

