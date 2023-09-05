import os, sys
currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(currentdir)
import init_functions
import rospy
from drive_control.msg import WheelSpeed
from odrive.enums import AxisState

# TODO: Once drive is working well, expand this node to include the three arm motors
class Node_ODriveInterface():
    def __init__(self):
        # Initialize node and subscribe to whatever needs to be subscribed to
        self.lb_speed_cmd = 0.0
        rospy.init_node('odrive_interface')
        self.feedback_publisher = rospy.Publisher("/wheel_velocity_feedback", WheelSpeed, queue_size=1)
        self.command_subscriber = rospy.Subscriber("/wheel_velocity_cmd", WheelSpeed, self.handle_drive_command)
        # TODO: Discuss with software what new topic(s) to create for communicating motor errors as well as motor
        # meta-commands (e.g. recalibrate, clear errors etc.)

        # Frequency of the ODrive I/O
        self.rate = rospy.Rate(100)
        self.run()

    def handle_drive_command(self, command):
        self.lb_speed_cmd = (command.left[0])*0.159155
        pass


    def run(self):
        # Discover motors on startup and assign variables for each
        motors_dict = init_functions.enumerate_motors()
        if not motors_dict:
            raise IOError("Motor enumeration failed")
        drive_lb = motors_dict["DRIVE_LB"]

        # Calibrate all motors
        if not init_functions.calibrate_motors([drive_lb]):
            raise IOError("Motor initialization failed")
        
        while not rospy.is_shutdown():
            # Put in speed command
            drive_lb.axis0.controller.input_vel = self.lb_speed_cmd

            # Get feedback and publish it to "/wheel_velocity_feedback"
            feedback = WheelSpeed()
            measured_speed_lb = drive_lb.encoder_estimator0.vel_estimate
            measured_speed_lf = 0
            measured_speed_rb = 0
            measured_speed_rf = 0

            feedback.left[0], feedback.left[1] = measured_speed_lb, measured_speed_lf
            feedback.right[0], feedback.right[1] = measured_speed_rb, measured_speed_rf
            print(f"\rMeasured Speed: {measured_speed_lb}", end='')
            self.feedback_publisher.publish(feedback)

            # See if there are any errors
            if drive_lb.axis0.active_errors != 0:
                print(f"\nError(s) occurred: ", init_functions.decode_errors(drive_lb.axis0.active_errors))
                # TODO: Discuss with software what to do with errors. Do we clear them and try to move on?
                # Or do we quit the process and try again from scratch?
                # drive_lb.clear_errors()

            self.rate.sleep()
        
        # On shutdown, bring motors to idle state
        print("Shutdown prompt received. Setting all motors to idle state.")
        drive_lb.axis0.requested_state = AxisState.IDLE


if __name__ == "__main__":
    driver = Node_ODriveInterface()
    rospy.spin()
