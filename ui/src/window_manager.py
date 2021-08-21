import sys

import PyQt5
import rospy as rp 
from std_msgs import *
from PyQt5 import QtCore, QtGui, QtWidgets, uic
import asyncio
from functools import partial

import gui.main_window
import gui.battery
import gui.elec.current_consumption
import gui.elec.wheel_speed
import gui.status
import gui.video_feed
import std_msgs

from main import queueMethodForMain
from config import *
import gui.science.science_main


_windowOpenStatus = {}


def _clearWindowOpenStatus(T):
    _windowOpenStatus[T] = None


def _setText(component: QtWidgets.QPlainTextEdit, text: str):
    component.setPlainText(text)


# BATTERY INFO

def _batteryInfoOnClose(self, T, wnd: gui.battery.Ui_BatteryInfo, event):
    wnd.batSub.unregister()
    _clearWindowOpenStatus(T)


def _batteryInfoOnMsgReceived(self: gui.battery.Ui_BatteryInfo, arr: msg.Float32MultiArray):
    voltage = str(round(arr.data[0], 2))
    current = str(round(arr.data[1], 2))
    queueMethodForMain(_setText, self.txtVoltage, voltage)
    queueMethodForMain(_setText, self.txtCurrent, current)


def _batteryInfoOnOpen(self: gui.battery.Ui_BatteryInfo):
    self.batSub = rp.Subscriber("battery", msg.Float32MultiArray, partial(_batteryInfoOnMsgReceived, self))


# WHEELS

def _wheelsOnMsgReceived(self: gui.elec.wheel_speed.Ui_WheelSpeed, arr: msg.Float32MultiArray):
    frontLeft = str(round(arr.data[0], 1))
    rearLeft = str(round(arr.data[1], 1))
    frontRight = str(round(arr.data[2], 1))
    rearRight = str(round(arr.data[3], 1))
    queueMethodForMain(_setText, self.txtFL, frontLeft)
    queueMethodForMain(_setText, self.txtRL, rearLeft)
    queueMethodForMain(_setText, self.txtFR, frontRight)
    queueMethodForMain(_setText, self.txtRR, rearRight)

def _wheelSpeedOnClose(self, T, wnd, event):
    wnd.wheelSub.unregister()
    _clearWindowOpenStatus(T)


def _wheelSpeedOnOpen(self):
    self.wheelSub = rp.Subscriber("wheels", msg.Float32MultiArray, partial(_wheelsOnMsgReceived, self))


# CURRENT


def _currentConsumptionOnClose(self, T, wnd, event):
    wnd.currentSub.unregister()
    _clearWindowOpenStatus(T)

def _currentOnMsgReceived(self: gui.elec.current_consumption.Ui_CurrentConsumption, arr: msg.Float32MultiArray):
    cpu = str(round(arr.data[0], 2))
    motor = str(round(arr.data[1], 2))
    queueMethodForMain(_setText, self.txtCpu, cpu)
    queueMethodForMain(_setText, self.txtMotor, motor)


def _currentConsumptionOnOpen(self):
    self.currentSub = rp.Subscriber("current", msg.Float32MultiArray, partial(_currentOnMsgReceived, self))


# status

def _statusOnOpen(self):
    pass

def _statusOnClose(self, T, wnd, event):
    _clearWindowOpenStatus(T)

def _videoFeedsOnOpen(self):
    pass

def _videoFeedsOnClose(self, T, wnd, event):
    _clearWindowOpenStatus(T)


def closeAllWindows():
    for _, wid in _windowOpenStatus.items():
        wid: QtWidgets.QWidget = wid
        if wid is not None:
            wid.close()


def _openWindow(T, parent, onOpen, onClose):
    if T in _windowOpenStatus.keys():
        if _windowOpenStatus[T] is not None:
            theWidget: QtWidgets.QWidget = _windowOpenStatus[T]
            theWidget.activateWindow()
            return
            
    wid = QtWidgets.QWidget()
    parent.windows.append(wid)
    wnd = T()
    wnd.setupUi(wid)
    onOpen(wnd)
    wid.closeEvent = partial(onClose, wid, T, wnd)
    wid.show()
    _windowOpenStatus[T] = wid

def _openCustomWindow(T, parent, onOpen, onClose):
    if T in _windowOpenStatus.keys():
        if _windowOpenStatus[T] is not None:
            theWidget: QtWidgets.QWidget = _windowOpenStatus[T]
            theWidget.activateWindow()
            return
    wid = T()
    parent.windows.append(wid)
    onOpen(wid)
    wid.closeEvent = partial(onClose, wid, T, None)
    wid.show()
    _windowOpenStatus[T] = wid

def _openScience(parent):
    _openCustomWindow(gui.science.science_main.Ui_ScienceMain, parent, _videoFeedsOnOpen, _videoFeedsOnClose)

def openBatteryInfo(parent):
    _openWindow(gui.battery.Ui_BatteryInfo, parent, _batteryInfoOnOpen, _batteryInfoOnClose)


def openWheelSpeed(parent):
    _openWindow(gui.elec.wheel_speed.Ui_WheelSpeed, parent, _wheelSpeedOnOpen, _wheelSpeedOnClose)


def openCurrentConsumption(parent):
    _openWindow(gui.elec.current_consumption.Ui_CurrentConsumption, parent, _currentConsumptionOnOpen, _currentConsumptionOnClose)

def openVideoFeeds(parent):
    _openCustomWindow(gui.video_feed.SingleVideoScreen, parent, _videoFeedsOnOpen, _videoFeedsOnClose)

def openStatus(parent):
    _openWindow(gui.status.Ui_Status, parent, _statusOnOpen, _statusOnClose)

def openScience(parent):
    _openScience(parent)
