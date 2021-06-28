#!/usr/bin/env python3

import rospy
import serial_interface as serialInt 
from std_msgs.msg import Int32
from arm_control.msg import ArmMotorCommand, ArmStatusFeedback
from DriveControl.msg import WheelSpeed
from embedded_bridge.msg import PowerFeedback
import time

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
        self.baud = 115200
        self.timeout = 0.01
        self.port_list = []
        self.mapping = {
            "drive_left"    : None,
            "drive_right"   : None,
            "arm_shoulder"  : None,
            "arm_forearm"   : None,
            "power"         : None,
            "science"       : None
        }

        # Drive System
        self.drive_state_publisher      = rospy.Publisher("drive_state_data", WheelSpeed, queue_size=1)
        self.drive_control_subscriber   = rospy.Subscriber("drive_control_data", WheelSpeed, None)
        self.drive_state = WheelSpeed()

        # Arm System
        self.arm_state_publisher        = rospy.Publisher("arm_state_data", ArmStatusFeedback, queue_size=1)
        self.arm_control_subscriber     = rospy.Subscriber("arm_control_data", ArmMotorCommand, None)
        self.arm_state = ArmStatusFeedback()

        # Science System
        self.science_state_publisher    = rospy.Publisher("science_state_data", Int32, queue_size=1)
        self.science_control_subscriber = rospy.Subscriber("science_control_data", Int32, None)

        # Power System
        self.power_state_publisher      = rospy.Publisher("power_state_data", PowerFeedback, queue_size=1)
        self.power_control_subscriber   = rospy.Subscriber("power_control_data", Int32, None)
        self.power_state = PowerFeedback()

        # Map found devices
        self.mapPorts()
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
                        msg = self.mapping[sys].get_packet()
                        if msg is not None:
                            # Interpret embedded system data
                            sys_id, frame_id, frame_type, payload = msg
                            data = []
                            # print(f"{sys_id}|{frame_id}|{frame_type}|{payload}")
                            if(
                                frame_type == '0' or
                                frame_type == '1' or
                                frame_type == '6'
                            ):
                                # Floats seperated by commas
                                payload_values_str = payload.split(',')
                                for val_str in payload_values_str:
                                    data.append(float(val_str))

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

                            # print(f"sys: {sys}\t| data : {data}")
                            # Get ROS message format
                            if sys == 'drive_left':
                                # print("setting drive left data")
                                self.drive_state.wheel_speed[0] = sum(data)/len(data)

                            elif sys == 'drive_right':
                                # print("setting drive right data")
                                self.drive_state.wheel_speed[1] = sum(data)/len(data)

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
        self.port_list = serialInt.find_ports()

        for port in self.port_list:
            sys = serialInt.SerialInterface(port, self.baud, self.timeout)
            connected = sys.start_link()
            if connected:
                # Map system
                if sys.peer_sys == '0':
                    self.mapping['drive_left'] = sys
                elif sys.peer_sys == '1':
                    self.mapping['drive_right'] = sys
                elif sys.peer_sys == '2':
                    self.mapping['arm_shoulder'] = sys
                elif sys.peer_sys == '3':
                    self.mapping['arm_forearm'] = sys
                elif sys.peer_sys == '4':
                    self.mapping['power'] = sys
                elif sys.peer_sys == '5':
                    self.mapping['science'] = sys

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
        self.mapping["arm_shoulder"].put_packet('1', data_shoulder)
        self.mapping["arm_forearm"].put_packet('1', data_forearm)

    def writeDriveCommand(self, control):
        data_left = f"{control.wheel_speed[0]},{control.wheel_speed[0]}"
        data_right = f"{control.wheel_speed[1]},{control.wheel_speed[1]}"
        self.mapping["drive_left"].put_packet('1', data_left)
        self.mapping["drive_right"].put_packet('1', data_right)

    def writeScienceCommand(self, control):
        pass

    def writePowerCommand(self, control):
        pass


if __name__ == "__main__":
    bridge = Node_EmbeddedBridge()
    rospy.spin()