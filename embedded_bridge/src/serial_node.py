#!/usr/bin/env python3

import rospy
import os, sys
currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(currentdir)
import serial_interface as serialInt 
from std_msgs.msg import Int32
from arm_control.msg import ArmMotorCommand, ArmStatusFeedback
from drive_control.msg import WheelSpeed
from embedded_bridge.msg import PowerFeedback
from science_module.msg import ScienceCmd, ScienceFeedback, CcdData
import time
import struct

class Node_EmbeddedBridge():
    """Serial ROS Node for exchanging data with the embedded systems on the 
    McGill Mars Rover. This Bridge is designed to follow the Serial Communication
    Documentation for message interpretation and message encoding.

    Attributes
    --------
    baud : int
        data rate of the communication with the embedded systems (must match them!)
    port_list : list(str)
        list of port names found connected to the CPU
    mapping : dict(str, serial_interface)
        dictionnary containing the port connection of every embedded board connected
        mapped to the system identification
    """
    def __init__(self):
        # Initialize ROS
        rospy.init_node("SerialNode", anonymous=False)

        # Bridge Parameters
        self.baud = 9600
        self.timeout = 1
        self.port_list = []
        self.mapping = {
            "drive"         : None,
            "arm_shoulder"  : None,
            "arm_forearm"   : None,
            "power"         : None,
            "science"       : None
        }

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

        # Map found devices
        self.mapPorts()
        print("mapped")
        rospy.on_shutdown(self.releasePorts)

        # Start Node
        self.run()

    def run(self):
        """Runs the node loop for getting and updating the Gamepad information for user control
        """
        
        while not rospy.is_shutdown():
            if rospy.is_shutdown():
                exit()
            try:
                # Acquire latest messages from embedded systems
                for sys in self.mapping:
                    if self.mapping[sys] is not None:
                        time.sleep(0.1)
                        if self.mapping['drive'].serial.inWaiting() > 0:
                            valid, packet, rest_of_msg = self.mapping[sys].read_bytes()
                        else:
                            valid = False

                        # Filter out the rest of the message
                        while len(rest_of_msg) > 0 and valid is True:
                            valid, packet, rest_of_msg = serialInt.decode_bytes(rest_of_msg)
                            str_packet = [str(i) for i in packet]
                            formatted_msg = ' '.join(str_packet)
                            print(f"Message from arduino: {formatted_msg}")

                        if valid:
                            # Interpret embedded system data
                            frame_type, payload_len, sys_id, payload, crc = packet
                            data = []
                            #print(sys)
                            if(
                                frame_type == '0' or
                                frame_type == '1' or
                                frame_type == '6'
                            ):
                                # Floats seperated by commas
                                #payload_values_str = payload.split(',')
                                for byte_data in payload:
                                    data.append(byte_data)

                            elif frame_type == '2':
                                # Bits representing booleans
                                binary = bytes(payload, 'ascii')
                                bits = int.from_bytes(binary, 'big')
                                max_bits = 8
                                for i in range(0, max_bits):
                                    data.insert(0, bool(bits & 0x01))
                                    bits = bits >> 1
                                
                            elif frame_type == '3':
                                # Int and Float seperated by comma
                                payload_values_str = payload.split(',')
                                data.append(int(payload_values_str[0]))
                                data.append(float(payload_values_str[1]))

                            elif frame_type == '4':
                                # Int
                                data.append(int(payload))

                            elif frame_type == '5':
                                # CCD Sensor data conversion
                                pass
                            else:
                                # Unknown Data format
                                pass

                            if len(data) == 0:
                                continue

                            #print(f"sys: {sys}\t| data : {data}")
                            # Get ROS message format
                            if sys == 'drive':
                                #print("setting drive left data")
                                self.drive_state.left[0] = data[0]
                                self.drive_state.left[1] = data[1]
                                self.drive_state.right[0] = data[2]
                                self.drive_state.right[1] = data[3]
                                
                                
                            elif sys == 'arm_shoulder':
                                # print("setting arm shoulder data")
                                self.arm_state.MotorPos[0] = data[0]
                                self.arm_state.MotorPos[1] = data[1]
                                self.arm_state.MotorPos[2] = data[2]

                            elif sys == 'arm_forearm':
                                # print("setting arm forearm data")
                                self.arm_state.MotorPos[3] = data[0]
                                self.arm_state.MotorPos[4] = data[1]
                                self.arm_state.ClawMode = bool(data[2])

                            elif sys == 'power':
                                # print("setting power data")
                                self.power_state.VoltageBattery1 = data[0]
                                self.power_state.CurrentBattery1 = data[1]
                                self.power_state.VoltageBattery2 = data[2]
                                self.power_state.CurrentBattery2 = data[3]                             

                            elif sys == 'science':
                                pass
                            else:
                                # Unrecognized system
                                pass

                # Publish latest system states
                self.arm_state_publisher.publish(self.arm_state)
                self.drive_state_publisher.publish(self.drive_state)
                self.power_state_publisher.publish(self.power_state)

            except Exception as error:
                rospy.logerr(str(error))
        
        exit()

    def mapPorts(self):
        """Maps all embedded systems found 
        """
        # Find the Arduino Serial
        # found = False
        # while not found:
        #     found, ports = serialInt.find_arduino_ports()
        
        s = serialInt.SerialInterface("/dev/ttyACM0", baud_rate=self.baud, timeout=self.timeout)
        self.mapping['drive'] = s

        # self.port_list = serialInt.find_ports()

        # for port in self.port_list:
        #     sys = serialInt.SerialInterface(port, self.baud, self.timeout)
            # # Map system
            # if sys.peer_sys == '0':
                # self.mapping['drive'] = sys
            # #elif sys.peer_sys == '1':
            # #    self.mapping['drive_right'] = sys
            # elif sys.peer_sys == '2':
            #     self.mapping['arm_shoulder'] = sys
            # elif sys.peer_sys == '3':
            #     self.mapping['arm_forearm'] = sys
            # elif sys.peer_sys == '4':
            #     self.mapping['power'] = sys
            # elif sys.peer_sys == '5':
            #     self.mapping['science'] = sys

        print(self.port_list)
        print(self.mapping)

    def releasePorts(self):
        """Releases all the ports used by the embedded bridge
        """
        for sys in self.mapping.values():
            if sys is None:
                continue
            else:
                sys.stop_link()

    def writeArmCommand(self, control):
        data_shoulder = f"{control.MotorVel[0]},{control.MotorVel[1]},{control.MotorVel[2]}"
        data_forearm  = f"{control.MotorVel[3]},{control.MotorVel[4]},{int(control.ClawState)}"
        self.mapping["arm_shoulder"].send_bytes('1', data_shoulder)
        self.mapping["arm_forearm"].send_bytes('1', data_forearm)

    def writeDriveCommand(self, control):
        # print(control.left+control.right)
        self.mapping['drive'].send_bytes('0', control.left + control.right, '1')
        print(f"speeds: {control.left + control.right}")
        #self.mapping['drive'].send_bytes('0', [100, 100, 100, 100], '1')
        
        time.sleep(15)
        

    def writeScienceCommand(self, control):
        # Send State Request
        state = 0
        state += int(control.LedState)
        state += int(control.LaserState) << 1
        state += int(control.SolenoidState) << 2
        state += int(control.PeltierState) << 3
        state += int(control.CcdSensorSnap) << 4
        state += int(control.Shutdown) << 5
        state_msg = f"{state}"

        self.mapping["science"].send_bytes('2', state_msg)

        # Send Motor Control
        motor_msg = f"{control.MotorSpeed:.3f}"
        self.mapping["science"].send_bytes('1', motor_msg)

        # Send Stepper Commands
        step1_msg = f"0,{control.Stepper1IncAng:.4f}"
        self.mapping["science"].send_bytes('3', step1_msg)
        step2_msg = f"1,{control.Stepper2IncAng:.4f}"
        self.mapping["science"].send_bytes('3', step2_msg)


    def writePowerCommand(self, control):
        pass

if __name__ == "__main__":
    bridge = Node_EmbeddedBridge()
    rospy.spin()