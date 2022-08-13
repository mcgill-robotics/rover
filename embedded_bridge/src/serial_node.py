#!/usr/bin/env python3

from ctypes import sizeof
import numpy
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
from std_msgs.msg import Int16
import struct


tx_counter = 0

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
            "arm"             : None,
            
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
        self.science_state = ScienceFeedback()

        # Power System
        self.power_state_publisher      = rospy.Publisher("power_state_data", PowerFeedback, queue_size=1)
        self.power_control_subscriber   = rospy.Subscriber("power_control_data", Int32, None)
        self.mode_sub = rospy.Subscriber("system_selection", Int16, self.current_system_power)
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
                        time.sleep(0.5)
                        if self.mapping[sys].serial.inWaiting() > 0:
                            valid, packet, rest_of_msg = self.mapping[sys].read_bytes()
                        else:
                            valid = False


                        # Filter out the rest of the message
                        while valid is True and len(rest_of_msg) > 0:
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
                                print("setting drive data")
                                self.drive_state.left[0] = data[0]
                                self.drive_state.left[1] = data[1]
                                self.drive_state.right[0] = data[2]
                                self.drive_state.right[1] = data[3]
                                
                                
                            elif sys == 'arm':
                                print("setting arm data")
                                self.arm_state.MotorPos[0] = data[0]
                                self.arm_state.MotorPos[1] = data[1]
                                self.arm_state.MotorPos[2] = data[2]
                                self.arm_state.MotorPos[3] = data[0]
                                self.arm_state.MotorPos[4] = data[1]
                                self.arm_state.ClawMode = bool(data[2])
                                

                            elif sys == 'power':
                                print("setting power data")
                                self.power_state.VoltageBattery1 = data[0]
                                self.power_state.CurrentBattery1 = data[1]
                                self.power_state.VoltageBattery2 = data[2]
                                self.power_state.CurrentBattery2 = data[3]                             

                            elif sys == 'science':
                                self.science_state.LedState = data[0]
                                self.science_state.LaserState = data[1]
                                self.science_state.GripperState = data[2]
                                self.science_state.PeltierState = data[3]
                                self.science_state.Stepper1Fault = data[4]
                                self.science_state.Stepper2Fault = data[5]
                            else:
                                # Unrecognized system
                                pass

                # Publish latest system states
                self.arm_state_publisher.publish(self.arm_state)
                self.drive_state_publisher.publish(self.drive_state)
                self.power_state_publisher.publish(self.power_state)
                self.science_state_publisher.publish(self.science_state)

            except Exception as error:
                rospy.logerr(str(error))
        
        exit()

    def mapPorts(self):
        """Maps all embedded systems found 
        """
        
        # s = serialInt.SerialInterface("/dev/ttyACM0", baud_rate=self.baud, timeout=self.timeout, prints=True)
        # self.mapping['science'] = s

        self.port_list = serialInt.find_ports()

        for port in self.port_list:
            print("=========>>>>> port")
            
            bridge = serialInt.SerialInterface(port, self.baud, self.timeout)
            bridge.send_bytes('0', [0, 0, 0, 0, 0], '1')
            time.sleep(1)
            if bridge.serial.inWaiting() > 0:
                print("Reading")
                valid, packet, rest_of_msg = bridge.read_bytes()
            if valid is True:
                valid, packet, rest_of_msg = serialInt.decode_bytes(rest_of_msg)
                str_packet = [str(i) for i in packet]
                formatted_msg = ' '.join(str_packet)
                print(f"Message from arduino: {formatted_msg}")
            
            print(f"packet: {packet}")
            sys_id = packet[2]

            # Map system
            if sys_id == '1':
                self.mapping['drive'] = bridge
            elif sys_id == '0':
                self.mapping['arm'] = bridge
            elif sys_id == '4':
                self.mapping['power'] = bridge
            elif sys_id == '5':
                self.mapping['science'] = bridge

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
        payload = [control.MotorVel[0], control.MotorVel[1], control.MotorVel[2], control.MotorVel[3], control.MotorVel[4], int(control.ClawState)]
        self.mapping["arm"].send_bytes('0', payload, '0')


    def writeDriveCommand(self, control):
        global tx_counter
        if tx_counter == 10:
            tx_counter=0
            #FILTER FOR ELECTRICAL, from -100 to a 100
            payload = []
            payload.append(round(control.left[0]/1,2))
            payload.append(round(control.left[0]/1,2))
            payload.append(round(control.right[0]/1,2))
            payload.append(round(control.right[0]/1,2))
            self.mapping['drive'].send_bytes('0', payload, '1')
        tx_counter = tx_counter+1
        time.sleep(0.1)
        

    def writeScienceCommand(self, control):
        # Send State Request
        # state = 0
        # state += int(control.LedState)
        # state += int(control.LaserState) << 1
        # state += int(control.GripperState) << 2
        # state += int(control.PeltierState) << 3
        # state += int(control.CcdSensorSnap) << 4
        # state += int(control.Shutdown) << 5
        global tx_counter
        if tx_counter == 10:
            tx_counter=0
            msg = [int(control.Shutdown),
                int(control.GripperState),
                control.Stepper1IncAng,
                control.Stepper2IncAng,
                round(control.MotorSpeed,2)]
            self.mapping["science"].send_bytes('0', msg, '5')
        tx_counter = tx_counter+1
        time.sleep(0.1)


    def writePowerCommand(self, control):
        pass

    def current_system_power(self, sys_id):
        if sys_id.data == 1:   #drive
            self.mapping["power"].send_bytes('0', [1,0,0], '4')
        elif sys_id.data == 0:  #arm
            self.mapping["power"].send_bytes('0', [1,1,0], '4')
        elif sys_id.data == 5:  #science
            self.mapping["power"].send_bytes('0', [1,0,1], '4')
        else:
            print("Unknown System")

        
        





if __name__ == "__main__":
    bridge = Node_EmbeddedBridge()
    rospy.spin()