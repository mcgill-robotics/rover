import threading
from threading import Lock
from odrive_interface.msg import MotorState, MotorError
from ODriveJoint import *
from std_msgs.msg import Float32MultiArray
from odrive.utils import dump_errors
from odrive.enums import AxisState, ODriveError, ProcedureResult
from enum import Enum
import rospy
import os
import sys

currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(currentdir)


# unused at the moment
class FeedbackMode(Enum):
    FROM_ODRIVE = "fb_from_odrive"
    FROM_OUTSHAFT = "fb_from_outshaft"


current_mode = FeedbackMode.FROM_OUTSHAFT

# VARIABLES -------------------------------------------------------------------
# Dictionary of ODriveJoint objects, key is the joint name in string format, value is the ODriveJoint object

# Predefine the order of joints for publishing feedback
joint_order = ["rover_arm_elbow", "rover_arm_shoulder", "rover_arm_waist"]


class NodeODriveInterfaceArm:
    def __init__(self):
        self.is_homed = False
        self.is_calibrated = False
        self.threads = []
        self.shutdown_flag = False

        # CONFIGURATION ---------------------------------------------------------------
        # Serial number of the ODrive controlling the joint
        self.joint_serial_numbers = {
            # 0x383834583539 = 61814047520057 in decimal
            "rover_arm_elbow": "383834583539",
            # 0x386434413539 = 62003024573753 in decimal
            "rover_arm_shoulder": "386434413539",
            # Not installed yet
            # "rover_arm_waist": "395935333231",
            "rover_arm_waist": "0",
        }
        self.joint_dict = {
            "rover_arm_elbow": None,
            "rover_arm_shoulder": None,
            "rover_arm_waist": None,
        }

        # FEEDBACK VARIABLES
        self.joint_pos_outshaft_dict = {
            "rover_arm_elbow": 0,
            "rover_arm_shoulder": 0,
            "rover_arm_waist": 0,
        }

        # Joint limits in degrees
        self.joint_pos_lim_dict = {
            "rover_arm_elbow": [-30, 30],
            "rover_arm_shoulder": [-30, 30],
            "rover_arm_waist": [-30, 30],
        }

        self.locks = {joint_name: Lock() for joint_name in self.joint_dict.keys()}

        rospy.init_node("odrive_interface_arm", disable_signals=True)

        # Subscriptions
        self.outshaft_pos_fb_subscriber = rospy.Subscriber(
            "/armBrushlessFb",
            Float32MultiArray,
            self.handle_outshaft_fb,
        )
        # Cmd comes from the external control node, we convert it to setpoint and apply it to the ODrive
        self.arm_joint_cmd_subscriber = rospy.Subscriber(
            "/armBrushlessCmd", Float32MultiArray, self.handle_arm_cmd
        )

        # Publishers
        self.odrive_state_publisher = rospy.Publisher(
            "/odrive_status", MotorState, queue_size=1
        )
        self.odrive_pos_fb_publisher = rospy.Publisher(
            "/odrive_armBrushlessFb", Float32MultiArray, queue_size=1
        )

        rospy.on_shutdown(self.shutdown_hook)

        # Frequency of the ODrive I/O
        self.rate = rospy.Rate(100)

        self.run()

    # Update encoder angle from external encoders
    def handle_outshaft_fb(self, msg):
        if not self.is_calibrated:
            return
        if not self.is_homed:
            self.joint_pos_outshaft_dict["rover_arm_elbow"] = msg.data[0]
            self.joint_pos_outshaft_dict["rover_arm_shoulder"] = msg.data[1]
            self.joint_pos_outshaft_dict["rover_arm_waist"] = msg.data[2]

            # Home the joints to 0 position
            for joint_name, joint_obj in self.joint_dict.items():
                try:
                    if not joint_obj.odrv:
                        continue
                    temp = (
                        self.joint_pos_outshaft_dict[joint_name]
                        * joint_obj.gear_ratio
                        / 360
                    )
                    joint_obj.odrv.axis0.set_abs_pos(temp)
                    print("Homing joint: ", joint_name)
                except Exception as e:
                    print(
                        f"Cannot home joint: {joint_name} to position: {self.joint_pos_outshaft_dict[joint_name]} - {str(e)}"
                    )

            # Set all pos_cmd to 0
            for joint_name, joint_obj in self.joint_dict.items():
                joint_obj.pos_cmd = 0
            self.apply_arm_cmd()
            self.is_homed = True

    # Receive setpoint from external control node
    def handle_arm_cmd(self, msg):
        if not self.is_calibrated:
            print("ODrive not calibrated. Ignoring command.")
            return
        self.joint_dict["rover_arm_elbow"].pos_cmd = msg.data[0]
        self.joint_dict["rover_arm_shoulder"].pos_cmd = msg.data[1]
        self.joint_dict["rover_arm_waist"].pos_cmd = msg.data[2]
        self.apply_arm_cmd()

    def apply_arm_cmd(self):
        # APPLY Position Cmd
        for joint_name, joint_obj in self.joint_dict.items():
            if not joint_obj.odrv:
                continue
            try:
                # setpoint in radians
                setpoint = joint_obj.pos_cmd * joint_obj.gear_ratio / 360
                joint_obj.odrv.axis0.controller.input_pos = setpoint
                print(f"Setpoint applied to {joint_name}: {setpoint}")
            except fibre.libfibre.ObjectLostError:
                joint_obj.odrv = None
            except Exception as e:
                print(
                    f"Cannot apply setpoint {setpoint} to joint: {joint_name} - {str(e)}"
                )

    # SEND ODRIVE INFO AND HANDLE ERRORS
    def handle_joints_error(self):
        for joint_name, joint_obj in self.joint_dict.items():
            with self.locks[joint_name]:  # Use a lock specific to each joint
                if joint_obj.odrv is not None:
                    if (
                        joint_obj.odrv.axis0.current_state
                        != AxisState.CLOSED_LOOP_CONTROL
                    ):
                        if not joint_obj.is_reconnecting:
                            print(
                                f"{joint_name} is not in closed loop control, recalibrating..."
                            )
                            joint_obj.is_reconnecting = True
                            t = threading.Thread(
                                target=self.enter_closed_loop_control,
                                # target=self.calibrate_and_enter_closed_loop,
                                args=(joint_obj,),
                                # target=joint_obj.enter_closed_loop_control
                            )
                            self.threads.append(t)
                            t.start()
                    else:
                        # Handle case where 'axis0' is not available
                        # print(
                        #     f"Cannot check current state for {joint_name}, 'axis0' attribute missing."
                        # )
                        pass
                else:
                    # ODrive interface is None, indicating a disconnection or uninitialized state
                    if not joint_obj.is_reconnecting:
                        print(f"RECONNECTING {joint_name}...")
                        joint_obj.is_reconnecting = True
                        t = threading.Thread(
                            target=self.reconnect_joint,
                            args=(joint_name, joint_obj),
                        )
                        t.start()

    # Send joints angle feedback to ROS
    def publish_joints_feedback(self):
        feedback = Float32MultiArray()
        for joint_name, joint_obj in self.joint_dict.items():
            if not joint_obj.odrv:
                continue
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
            except fibre.libfibre.ObjectLostError:
                joint_obj.odrv = None
                feedback.data.append(0.0)
            except Exception as e:
                print(f"Cannot get feedback from joint: {joint_name} - {str(e)}")
                feedback.data.append(0.0)

        # Publish
        self.odrive_pos_fb_publisher.publish(feedback)

    def reconnect_joint(self, joint_name, joint_obj):
        # Attempt to reconnect...
        # Update joint_obj.odrv as necessary...
        joint_obj.reconnect()

        # Once done, reset the flag
        with self.locks[joint_name]:
            joint_obj.is_reconnecting = False

    def calibrate_and_enter_closed_loop_control(self, joint_obj):
        if joint_obj.odrv is not None:
            print(f"CALIBRATING joint {joint_obj.name}...")
            joint_obj.calibrate()

            print(f"ENTERING CLOSED LOOP CONTROL for joint {joint_obj.name}...")
            joint_obj.enter_closed_loop_control()

    def enter_closed_loop_control(self, joint_obj):
        if joint_obj.odrv is not None:
            print(f"ENTERING CLOSED LOOP CONTROL for joint {joint_obj.name}...")
            joint_obj.enter_closed_loop_control()

    def setup_odrive(self):
        # CONNECT -----------------------------------------------------
        for key, value in self.joint_serial_numbers.items():
            # Instantiate ODriveJoint class whether or not the connection attempt was made/successful
            self.joint_dict[key] = ODriveJoint(name=key, serial_number=value)

            # Set limits
            self.joint_dict[key].pos_min_deg = self.joint_pos_lim_dict[key][0]
            self.joint_dict[key].pos_max_deg = self.joint_pos_lim_dict[key][1]

            if value == "0":
                print(
                    f"Skipping connection for joint: {key} due to serial_number being 0"
                )
                continue

            self.joint_dict[key].reconnect()

        print("Connection step completed.")

        # CALIBRATE AND ENTER CLOSED LOOP CONTROL -----------------------------------------------------
        for joint_name, joint_obj in self.joint_dict.items():
            if joint_obj.odrv is None:
                continue
            print(f"Creating thread for joint {joint_obj.name}")
            t = threading.Thread(
                target=joint_obj.calibrate,
            )
            self.threads.append(t)
            t.start()

        # Wait for all threads to complete
        for t in self.threads:
            t.join()
        self.threads = []

        print("Calibration step completed.")

        for joint_name, joint_obj in self.joint_dict.items():
            if joint_obj.odrv is None:
                continue
            print(f"Creating thread for joint {joint_obj.name}")
            t = threading.Thread(
                target=joint_obj.enter_closed_loop_control,
            )
            self.threads.append(t)
            t.start()

        # Wait for all threads to complete
        for t in self.threads:
            t.join()
        self.threads = []

        self.is_calibrated = True

        # Set the direction of the motors
        self.joint_dict["rover_arm_elbow"].gear_ratio = 100
        self.joint_dict["rover_arm_shoulder"].gear_ratio = 100
        self.joint_dict["rover_arm_waist"].gear_ratio = 1

        print("Entering closed loop control step completed.")

    def print_loop(self):
        while True:
            print_joint_state_from_dict(self.joint_dict, sync_print=True)
            time.sleep(0.2)

    def loop_odrive(self):
        # MAIN LOOP ---------------------------------------------------------------
        while not rospy.is_shutdown():
            # PRINT TIMESTAMP
            # print(f"Time: {rospy.get_time()}")

            # ODRIVE POSITION FEEDBACK, different from the outshaft feedback
            self.publish_joints_feedback()

            # APPLY Position Cmd
            # Done by the handle_arm_cmd function

            # HANDLE ERRORS
            self.handle_joints_error()

            # Delay
            self.rate.sleep()

    def run(self):
        self.setup_odrive()
        thread = threading.Thread(target=self.loop_odrive)
        thread.start()
        print_thread = threading.Thread(target=self.print_loop)
        print_thread.start()
        rospy.spin()
        self.shutdown_hook()

    def shutdown_hook(self):
        self.shutdown_flag = True
        print("Shutdown initiated. Setting all motors to idle state.")
        for joint_name, joint_obj in self.joint_dict.items():
            if not joint_obj.odrv:
                continue
            joint_obj.odrv.axis0.requested_state = AxisState.IDLE
        for t in self.threads:
            t.join()
        print("All threads joined. Shutdown complete.")


if __name__ == "__main__":
    try:
        driver = NodeODriveInterfaceArm()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("Shutting down due to KeyboardInterrupt")
        rospy.signal_shutdown("KeyboardInterrupt")
        driver.shutdown_hook()
