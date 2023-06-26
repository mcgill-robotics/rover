#!/usr/bin/env python3

import rospy
import os, sys
currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(currentdir)
from std_msgs.msg import Int32
from arm_control.msg import ArmMotorCommand, ArmStatusFeedback
from drive_control.msg import WheelSpeed
from embedded_bridge.msg import PowerFeedback
from science_module.msg import ScienceCmd, ScienceFeedback, CcdData
import time
import serial_states
import serial
import matplotlib.pyplot as plt
import threading

class Node_EmbeddedBridge():
    """Serial ROS Node for exchanging data with the embedded systems on the 
    McGill Mars Rover. This Bridge is designed to follow the Serial Communication
    Documentation for message interpretation and message encoding.

    Attributes
    --------
    port_name: String
        filepath for the serial port. default is "/dev/ttyACM0"
    baud : int
        data rate of the communication with the embedded systems (must match them!)
    timeout: int
        the amount of time to block and wait after serial.read() calls. currently set
        to 0 for non-blocking serial reads
    write_timeout: int
        same as above, except serial.write() calls
    serial_polling_interval: int
        amount of time between each cycle, where each cycle consists of one transmission
        to the backplane, one reply and ROS state publishing
    verbose: boolean
        when true, prints tx/rx states on terminal as well as some connection statistics
    graph:
        generates a tx/rx plot on exit
    """
    def __init__(self):
        # Initialize ROS
        rospy.init_node("SerialNode", anonymous=False)

        # Bridge Parameters
        self.port_name = "/dev/ttyACM0"
        self.baud = 115200
        self.timeout = 0
        self.write_timeout = 0
        self.serial_polling_interval = 0.01
        self.serial_buffer = bytearray(87)
        self.verbose = True
        self.graph = False

        # Instantiate serial port
        self.serial_port = serial.Serial(self.port_name, self.baud, timeout=self.timeout,
                                         write_timeout=self.write_timeout)

        # Initialize serial state objects and lock for the command state
        self.commandState = serial_states.SerialTX()
        self.feedbackState = serial_states.SerialRX()
        self.commandStateLock = threading.Lock()

        # Drive System
        self.drive_state_publisher      = rospy.Publisher("/feedback_velocity", WheelSpeed, queue_size=1)
        self.drive_control_subscriber   = rospy.Subscriber("/wheel_velocity_cmd", WheelSpeed, self.writeDriveCommand)
        self.drive_state = WheelSpeed()

        # Arm System
        self.arm_state_publisher        = rospy.Publisher("arm_state_data", ArmStatusFeedback, queue_size=1)
        self.arm_control_subscriber     = rospy.Subscriber("arm_control_data", ArmMotorCommand, self.writeArmCommand)
        self.arm_state = ArmStatusFeedback()

        # Science System
        self.science_state_publisher    = rospy.Publisher("science_state_data", ScienceFeedback, queue_size=1)
        self.science_ccd_data_publisher = rospy.Publisher("science_ccd_data", CcdData)
        self.science_control_subscriber = rospy.Subscriber("science_control_data", ScienceCmd, self.writeScienceCommand)

        # Power System
        self.power_state_publisher      = rospy.Publisher("power_state_data", PowerFeedback, queue_size=1)
        self.power_control_subscriber   = rospy.Subscriber("power_control_data", Int32, None)
        self.power_state = PowerFeedback()

        # Start Node
        self.run()

    def run(self):
        """Runs the node loop for getting and updating the Gamepad information for user control
        """

        # Initialize timekeeping/statistic variables
        t = time.time()
        if self.verbose:
            tx_data = []
            rx_data = []
            timestamps = []
            tx_time = time.time()
            rx_time = time.time()
            avg_latency = 0
            lost_packets = 0
            packets_received = 0
            latencies = []
        
        while not rospy.is_shutdown():
            if rospy.is_shutdown():
                self.serial_port.close()
                exit()
            try:
                # Transmit data using the SerialTX states
                if time.time() - t >= self.serial_polling_interval:
                    self.commandStateLock.acquire()
                    
                    # Enclose this in a try-finally block to make sure the lock
                    # is released even if there is an exception
                    try:
                        byte_array = self.commandState.encode()
                    finally:
                        self.commandStateLock.release()

                    tx = self.serial_port.write(byte_array)
                    t = time.time()

                    if self.verbose:
                        tx_time = time.time()
                        print(f"Wrote {tx} bytes to serial at {time.time()}")
                        self.commandState.debugPrint()

                # Check serial line for new feedback, put it into buffer and pass it to the
                # feedbackState object to update the feedback values
                waiting_chars = self.serial_port.in_waiting          
                if waiting_chars >= len(self.serial_buffer):
                    self.serial_port.readinto(self.serial_buffer)
                    rx_status = self.feedbackState.decode(self.serial_buffer)
                    if self.verbose:
                        packets_received += 1
                        if not rx_status:
                            lost_packets += 1
                        rx_time = time.time()
                        latency = rx_time - tx_time
                        latencies.append(latency)
                        print(f"Read {waiting_chars} bytes from serial at {time.time()}")
                        print(f"Latency: {round(1000 * latency, 4)} ms")
                        self.feedbackState.debugPrint()
                
                # Record TX/RX data for each cycle for debug graph
                if self.graph:
                    timestamps.append(time.time())
                    tx_data.append(self.commandState.driveMotorLBSpeed)
                    rx_data.append(self.feedbackState.driveMotorLBSpeed)

                # Transfer over the data from unified feedbackState to various system states
                self.drive_state.left[0] = self.feedbackState.driveMotorLBSpeed
                self.drive_state.left[1] = self.feedbackState.driveMotorLFSpeed
                self.drive_state.right[0] = self.feedbackState.driveMotorRBSpeed
                self.drive_state.right[1] = self.feedbackState.driveMotorRFSpeed

                # TODO: Clarify what the embedded node should publish
                self.arm_state_publisher.publish(self.arm_state)
                self.drive_state_publisher.publish(self.drive_state)
                self.power_state_publisher.publish(self.power_state)

                time.sleep(self.serial_polling_interval)

            except Exception as error:
                self.serial_port.close()
                rospy.logerr(str(error))
            
        # On exit, draw and show the debug graph as well as some connection statistics
        if self.verbose:
            avg_latency = sum(latencies) / len(latencies)
            packet_loss_percentage = 100 * (lost_packets / packets_received)
            print(f"\nAverage latency: {round(1000 * avg_latency, 4)} ms")
            print(f"\nPacket loss: {round(packet_loss_percentage, 2)}%")
            if self.graph:
                plt.plot(timestamps, tx_data, label='Transmitted Data')
                plt.plot(timestamps, rx_data, label='Received Data')
                plt.grid()
                plt.xlabel('Timestamp')
                plt.ylabel('Data')
                plt.legend()
                plt.show()
        
        # Also close serial port
        self.serial_port.close()

        exit()

    # TODO: Clarify message content with software teams for science and power
    def writeArmCommand(self, control):
        # Busy spin while waiting for lock, check back every ms
        while not self.commandStateLock.acquire(blocking=False):
            time.sleep(0.001)
        
        # Enclose with try-finally block to make sure the lock is released even if an
        # exception happens
        try:
            self.commandState.waistPosition = control.MotorPos[0]
            self.commandState.tumorPosition = control.MotorPos[1]
            self.commandState.elbowPosition = control.MotorPos[2]
            self.commandState.wristPitchPosition = control.MotorPos[3]
            self.commandState.wristRollPosition = control.MotorPos[4]
            self.commandState.clawState = control.ClawState
        finally:
            self.commandStateLock.release()

    def writeDriveCommand(self, control):
        # Busy spin while waiting for lock, check back every ms
        while not self.commandStateLock.acquire(blocking=False):
            time.sleep(0.001)

        # Enclose with try-finally block to make sure the lock is released even if an
        # exception happens
        try:
            self.commandState.driveMotorLBSpeed = int(round(control.left[0]*5, 2))
            self.commandState.driveMotorLFSpeed = int(round(control.left[1]*5, 2))
            self.commandState.driveMotorRBSpeed = int(round(control.right[0]*5, 2))
            self.commandState.driveMotorRFSpeed = int(round(control.right[1]*5, 2))
        finally:
            self.commandStateLock.release()

    def writeScienceCommand(self, control):
        pass


    def writePowerCommand(self, control):
        pass

if __name__ == "__main__":
    bridge = Node_EmbeddedBridge()
    rospy.spin()