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

def config_shoulder_odrive(odrive_joint):
    print("APPLYING CONFIG for rover_arm_shoulder...")
    odrv_joint.odrv.config.dc_bus_overvoltage_trip_level = 30
    odrv_joint.odrv.config.dc_bus_undervoltage_trip_level = 10.5
    odrv_joint.odrv.config.dc_max_positive_current = 10
    odrv_joint.odrv.config.brake_resistor0.enable = True
    odrv_joint.odrv.config.brake_resistor0.resistance = 2
    odrv_joint.odrv.axis0.config.motor.motor_type = MotorType.HIGH_CURRENT
    odrv_joint.odrv.axis0.config.motor.torque_constant = 0.06080882352941176
    odrv_joint.odrv.axis0.config.motor.pole_pairs = 11
    odrv_joint.odrv.axis0.config.motor.current_soft_max = 12
    odrv_joint.odrv.axis0.config.motor.current_hard_max = 25.6
    odrv_joint.odrv.axis0.config.motor.calibration_current = 2.5
    odrv_joint.odrv.axis0.config.motor.resistance_calib_max_voltage = 2
    odrv_joint.odrv.axis0.config.calibration_lockin.current = 2.5
    odrv_joint.odrv.axis0.controller.config.input_mode = InputMode.PASSTHROUGH
    odrv_joint.odrv.axis0.controller.config.control_mode = (
            ControlMode.POSITION_CONTROL
        )
    # vel_limit determines how fast, vel_limit_tolerance determines how much it can go over, so we avoid vel_limit violations
    odrv_joint.odrv.axis0.controller.config.vel_limit = 3
    odrv_joint.odrv.axis0.controller.config.vel_limit_tolerance = 10
    odrv_joint.odrv.axis0.config.torque_soft_min = -5
    odrv_joint.odrv.axis0.config.torque_soft_max = 5
    odrv_joint.odrv.can.config.protocol = Protocol.NONE
    odrv_joint.odrv.config.enable_uart_a = False
    odrv_joint.odrv.rs485_encoder_group0.config.mode = (
        Rs485EncoderMode.AMT21_POLLING
    )
    odrv_joint.odrv.axis0.config.load_encoder = EncoderId.RS485_ENCODER0
    odrv_joint.odrv.axis0.config.commutation_encoder = EncoderId.RS485_ENCODER0

def config_shoulder_odrive(ordive_joint):
    print("APPLYING CONFIG for rover_arm_elbow...")
    odrv_joint.odrv.config.dc_bus_overvoltage_trip_level = 30
    odrv_joint.odrv.config.dc_max_positive_current = 5
    odrv_joint.odrv.config.brake_resistor0.enable = True
    odrv_joint.odrv.config.brake_resistor0.resistance = 2
    odrv_joint.odrv.axis0.config.motor.motor_type = MotorType.HIGH_CURRENT
    odrv_joint.odrv.axis0.config.motor.torque_constant = 0.04543956043956044
    odrv_joint.odrv.axis0.config.motor.pole_pairs = 7
    odrv_joint.odrv.axis0.config.motor.current_soft_max = 6
    odrv_joint.odrv.axis0.config.motor.current_hard_max = 17.8
    # test_odrv_joint.odrv.axis0.config.motor.current_hard_max = 12
    odrv_joint.odrv.axis0.config.motor.calibration_current = 2.5
    odrv_joint.odrv.axis0.config.motor.resistance_calib_max_voltage = 2
    odrv_joint.odrv.axis0.config.calibration_lockin.current = 2.5
    odrv_joint.odrv.axis0.controller.config.input_mode = InputMode.PASSTHROUGH
    odrv_joint.odrv.axis0.controller.config.control_mode = (
        ControlMode.POSITION_CONTROL
    )
    odrv_joint.odrv.axis0.controller.config.vel_limit = 3
    odrv_joint.odrv.axis0.controller.config.vel_limit_tolerance = 10
    odrv_joint.odrv.axis0.config.torque_soft_min = -2
    odrv_joint.odrv.axis0.config.torque_soft_max = 2
    odrv_joint.odrv.can.config.protocol = Protocol.NONE
    odrv_joint.odrv.config.enable_uart_a = False
    odrv_joint.odrv.rs485_encoder_group0.config.mode = (
        Rs485EncoderMode.AMT21_POLLING
    )
    odrv_joint.odrv.axis0.config.load_encoder = EncoderId.RS485_ENCODER0
    odrv_joint.odrv.axis0.config.commutation_encoder = EncoderId.RS485_ENCODER0


