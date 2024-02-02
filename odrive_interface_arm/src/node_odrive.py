import os, sys

currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(currentdir)

# TODO: Figure out why catkin on the Jetson isn't playing nice with this import. It worked on my PC -Eren
# import init_functions
import rospy
from enum import Enum
from odrive.enums import AxisState, ProcedureResult
from odrive.utils import dump_errors
from std_msgs.msg import Float32MultiArray
from ODrive_Joint import *
from odrive_interface_arm.msg import MotorState, MotorError


class FeedbackMode(Enum):
    FROM_ODRIVE = "fb_from_odrive"
    FROM_OUTSHAFT = "fb_from_outshaft"


# CONFIGURATION
# Serial number of the ODrive controlling the joint
arm_serial_numbers = {
    "elbow_joint": "383834583539",  # 0x383834583539 = 61814047520057 in decimal
    "shoulder_joint": "386434413539",  # 0x386434413539 = 62003024573753 in decimal
    "waist_joint": "0",
}
arm_gear_ratios = {
    "elbow_joint": 1,
    "shoulder_joint": 1,
    "waist_joint": 1,
}
current_mode = FeedbackMode.FROM_ODRIVE

# VARIABLES
# Dictionary of ODrive_Joint objects, key is the joint name in string format, value is the ODrive_Joint object
arm_joint_dict = {
    "elbow_joint": None,
    "shoulder_joint": None,
    "waist_joint": None,
}


class Node_odrive_interface_arm:
    def __init__(self):
        # FEEDBACK VARIABLES
        self.joint_pos_outshaft_dict = {
            "elbow_joint": None,
            "shoulder_joint": None,
            "waist_joint": None,
        }

        # SETPOINT VARIABLES
        self.joint_setpoint_dict = {
            "elbow_joint": None,
            "shoulder_joint": None,
            "waist_joint": None,
        }

        # Subscriptions
        rospy.init_node("odrive_interface_arm")
        self.outshaft_subscriber = rospy.Subscriber(
            "/arm_outshaft_fb",
            Float32MultiArray,
            self.handle_arm_outshaft_fb,
        )
        self.command_subscriber = rospy.Subscriber(
            "/armBrushlessCmd", Float32MultiArray, self.handle_arm_command
        )

        # Publishers
        self.state_publisher = rospy.Publisher(
            "/odrive_arm_state", MotorState, queue_size=1
        )
        # Could be from the ODrive or from the outshaft, depending on configuration
        self.feedback_publisher = rospy.Publisher(
            "/armBrushlessFB",
            Float32MultiArray,
            queue_size=10,
        )

        # Frequency of the ODrive I/O
        self.rate = rospy.Rate(100)
        self.run()

    # Update encoder angle from external encoders
    def handle_arm_outshaft_fb(self, msg):
        self.joint_pos_outshaft_dict["elbow_joint"] = msg.data[0]
        self.joint_pos_outshaft_dict["shoulder_joint"] = msg.data[1]
        self.joint_pos_outshaft_dict["waist_joint"] = msg.data[2]
        if current_mode == FeedbackMode.FROM_OUTSHAFT:
            self.feedback_publisher.publish(msg)

    # Receive setpoint from external control node
    def handle_arm_command(self, msg):
        self.joint_setpoint_dict["elbow_joint"] = msg.data[0]
        self.joint_setpoint_dict["shoulder_joint"] = msg.data[1]
        self.joint_setpoint_dict["waist_joint"] = msg.data[2]

    def run(self):
        # CONNECT TO ODRIVE
        for key, value in arm_serial_numbers.items():
            try:
                arm_joint_dict[key] = ODrive_Joint(
                    gear_ratio=arm_gear_ratios[key],
                )
                if value != "0":
                    try:
                        odrv = odrive.find_any(serial_number=value, timeout=5)
                        arm_joint_dict[key].attach_odrive(odrv)
                        print(f"Connected joint: {key}, serial_number: {value}")
                    except:
                        odrv = None
            except TimeoutError:
                print(f"Cannot connect joint: {key}, serial_number: {value}")
                arm_joint_dict[key] = None

        # Predefine the order of joints for publishing feedback
        joint_order = ["elbow_joint", "shoulder_joint", "waist_joint"]

        # MAIN LOOP
        while not rospy.is_shutdown():
            if current_mode == FeedbackMode.FROM_ODRIVE:
                feedback = Float32MultiArray()
                for joint_name in joint_order:
                    joint_obj = arm_joint_dict.get(joint_name)
                    try:
                        # Assuming .odrv.axis0.pos_vel_mapper.pos_abs and .gear_ratio are correct
                        feedback.data.append(
                            360
                            * (
                                joint_obj.odrv.axis0.pos_vel_mapper.pos_abs
                                / joint_obj.gear_ratio
                            )
                        )
                    # Default value in case lose connection to ODrive
                    except AttributeError:
                        feedback.data.append(0.0)
                # Publish feedback
                self.feedback_publisher.publish(feedback)

            # APPLY SETPOINT
            for joint_name, joint_obj in arm_joint_dict.items():
                if not joint_obj.odrv:
                    continue
                if current_mode == FeedbackMode.FROM_ODRIVE:
                    try:
                        joint_obj.odrv.axis0.controller.input_pos = (
                            self.joint_setpoint_dict[joint_name]
                        )
                    except:
                        print(f"Cannot apply setpoint to joint: {joint_name}")

            # PRINT POSITIONS TO CONSOLE
            for joint_name, joint_obj in arm_joint_dict.items():
                # Check if the odrive is connected
                status = "connected" if joint_obj.odrv else "disconnected"
                print(f"{joint_name} {joint_obj.serial_number} ({status})")
                if joint_obj.odrv:
                    print(f"-pos_rel={joint_obj.odrv.axis0.pos_vel_mapper.pos_rel}")
                    print(f"-pos_abs={joint_obj.odrv.axis0.pos_vel_mapper.pos_abs}")
                # Newline
                print()

            # SEND ODRIVE INFO AND HANDLE ERRORS
            for joint_name, joint_obj in arm_joint_dict.items():
                if not joint_obj.odrv:
                    continue
                try:
                    state_fb = MotorState()
                    state_fb.id = joint_obj.serial_number
                    state_fb.state = joint_obj.odrv.axis0.current_state
                    state_fb.pos_rel = joint_obj.odrv.axis0.pos_vel_mapper.pos_rel
                    state_fb.pos_abs = joint_obj.odrv.axis0.pos_vel_mapper.pos_abs
                    state_fb.input_pos = joint_obj.odrv.axis0.controller.input_pos
                    self.state_publisher.publish(state_fb)

                    # ERROR HANDLING
                    if joint_obj.odrv.axis0.active_errors != 0:
                        # Tell the rover to stop
                        for joint_name, joint_obj in arm_joint_dict.items():
                            if not joint_obj.odrv:
                                continue
                            joint_obj.odrv.axis0.controller.input_vel = 0

                        # Wait until it actually stops
                        motor_stopped = False
                        while not motor_stopped:
                            motor_stopped = True
                            for joint_name, joint_obj in arm_joint_dict.items():
                                if not joint_obj.odrv:
                                    continue
                                if (
                                    abs(joint_obj.odrv.encoder_estimator0.vel_estimate)
                                    >= 0.01
                                ):
                                    motor_stopped = False
                            if rospy.is_shutdown():
                                print(
                                    "Shutdown prompt received. Setting all motors to idle state."
                                )
                                for joint_name, joint_obj in arm_joint_dict.items():
                                    if not joint_obj.odrv:
                                        continue
                                    joint_obj.odrv.axis0.requested_state = (
                                        AxisState.IDLE
                                    )
                                break

                        # Wait for two seconds while all the transient currents and voltages calm down
                        rospy.sleep(5)

                        # Now try to recover from the error. This will always succeed the first time, but if
                        # the error persists, the ODrive will not allow the transition to closed loop control, and
                        # re-throw the error.
                        joint_obj.odrv.clear_errors()
                        rospy.sleep(0.5)
                        joint_obj.odrv.axis0.requested_state = (
                            AxisState.CLOSED_LOOP_CONTROL
                        )
                        rospy.sleep(0.5)

                        # If the motor cannot recover successfully publish a message about the error, then print to console
                        if joint_obj.odrv.axis0.active_errors != 0:
                            error_fb = MotorError()
                            error_fb.id = joint_obj.serial_number
                            # error_fb.error = decode_errors(joint.axis0.active_errors)
                            error_fb.error = ODriveError(
                                joint_obj.odrv.axis0.active_errors
                            ).name
                            self.error_publisher.publish(error_fb)
                            print(
                                f"\nError(s) occurred. Motor ID: {error_fb.id}, Error(s): {error_fb.error}"
                            )

                            # Finally, hang the node and keep trying to recover until the error is gone or the shutdown signal is received
                            print(
                                f"\nMotor {error_fb.id} Cannot recover from error(s) {error_fb.error}. R to retry, keyboard interrupt to shut down node."
                            )
                            while not rospy.is_shutdown():
                                prompt = input(">").upper()
                                if prompt == "R":
                                    joint_obj.odrv.clear_errors()
                                    rospy.sleep(0.5)
                                    joint_obj.odrv.axis0.requested_state = (
                                        AxisState.CLOSED_LOOP_CONTROL
                                    )
                                    rospy.sleep(0.5)
                                    if joint_obj.odrv.axis0.active_errors == 0:
                                        break
                                    else:
                                        print("Recovery failed. Try again?")
                except AttributeError:
                    print(f"{joint_name} is not connected")

            self.rate.sleep()

        # On shutdown, bring motors to idle state
        print("Shutdown prompt received. Setting all motors to idle state.")
        for joint_name, joint_obj in arm_joint_dict.items():
            if not joint_obj.odrv:
                continue
            joint_obj.odrv.axis0.requested_state = AxisState.IDLE


if __name__ == "__main__":
    driver = Node_odrive_interface_arm()
    rospy.spin()
