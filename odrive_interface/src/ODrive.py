import os, sys
currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(currentdir)

# TODO: Figure out why catkin on the Jetson isn't playing nice with this import. It worked on my PC -Eren
# import init_functions
import rospy
from drive_control.msg import WheelSpeed
from odrive_interface.msg import MotorError, MotorState
from ODrive_utils import *
from odrive.enums import AxisState, ProcedureResult 

# TODO: Once drive is working well, expand this node to include the three arm motors
class Node_ODriveInterface():
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
        motors_dict = enumerate_motors()
        if not motors_dict:
            raise IOError("Motor enumeration failed")
        
        drive_lb = motors_dict["DRIVE_LB"]
        drive_lf = motors_dict["DRIVE_LF"]
        drive_rb = motors_dict["DRIVE_RB"]
        drive_rf = motors_dict["DRIVE_RF"]
        drive_motors = [drive_lb, drive_lf, drive_rb, drive_rf]

        # Calibrate all motors
        if not calibrate_motors(drive_motors):
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

            # Poll the ODrives for their states and check for errors
            for motor in drive_motors:
                state_fb = MotorState()
                state_fb.id = drive_ids[format(motor.serial_number, "x").upper()]
                
                # TODO: Decode and add state information
                # state_fb.state =
                self.state_publisher.publish(state_fb)

                if motor.axis0.active_errors != 0:
                    # Tell the rover to stop
                    for motor in drive_motors:
                        motor.axis0.controller.input_vel = 0
                    
                    # Wait until it actually stops
                    motor_stopped = False
                    while not motor_stopped:
                        motor_stopped = True
                        for motor in drive_motors:
                            if abs(motor.encoder_estimator0.vel_estimate) >= 0.01:
                                motor_stopped = False
                        if rospy.is_shutdown():
                            print("Shutdown prompt received. Setting all motors to idle state.")
                            for motor in drive_motors:
                                motor.axis0.requested_state = AxisState.IDLE
                            break
                    
                    # Wait for two seconds while all the transient currents and voltages calm down
                    rospy.sleep(5)
                    
                    # Now try to recover from the error. This will always succeed the first time, but if
                    # the error persists, the ODrive will not allow the transition to closed loop control, and
                    # re-throw the error.
                    motor.clear_errors()
                    rospy.sleep(0.5)
                    motor.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
                    rospy.sleep(0.5)

                    # If the motor cannot recover successfully publish a message about the error, then print to console
                    if motor.axis0.active_errors != 0:
                        error_fb = MotorError()
                        error_fb.id = drive_ids[format(motor.serial_number, "x").upper()]
                        error_fb.error = decode_errors(motor.axis0.active_errors)
                        self.error_publisher.publish(error_fb)
                        print(f"\nError(s) occurred. Motor ID: {error_fb.id}, Error(s): {error_fb.error}")
                    
                        # Finally, hang the node and keep trying to recover until the error is gone or the shutdown signal is received
                        print(f"\nMotor {error_fb.id} could not recover from error(s) {error_fb.error}. R to retry, keyboard interrupt to shut down node.")
                        while not rospy.is_shutdown():
                            prompt = input(">").upper()
                            if prompt == "R":
                                motor.clear_errors()
                                rospy.sleep(0.5)
                                motor.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
                                rospy.sleep(0.5)
                                if motor.axis0.active_errors == 0:
                                    break
                                else:
                                    print("Recovery failed. Try again?")


            self.rate.sleep()
        
        # On shutdown, bring motors to idle state
        print("Shutdown prompt received. Setting all motors to idle state.")
        for motor in drive_motors:
            motor.axis0.requested_state = AxisState.IDLE


if __name__ == "__main__":
    driver = Node_ODriveInterface()
    rospy.spin()
