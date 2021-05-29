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
import std_msgs

from main import queueMethodForMain
from config import *


_windowOpenStatus = {}


def _clearWindowOpenStatus(T):
    _windowOpenStatus[T] = None


# BATTERY INFO

def _batteryInfoOnClose(self, T, wnd: gui.battery.Ui_BatteryInfo, event):
    wnd.batSub.unregister()
    _clearWindowOpenStatus(T)


def _setText(component: QtWidgets.QPlainTextEdit, text: str):
    component.setPlainText(text)


def _batteryInfoOnMsgReceived(self: gui.battery.Ui_BatteryInfo, arr: msg.Float32MultiArray):
    voltage = str(arr.data[0])
    current = str(arr.data[1])
    queueMethodForMain(_setText, self.txtVoltage, voltage)
    queueMethodForMain(_setText, self.txtCurrent, current)


def _batteryInfoOnOpen(self: gui.battery.Ui_BatteryInfo):
    self.batSub = rp.Subscriber("battery", msg.Float32MultiArray, partial(_batteryInfoOnMsgReceived, self))


# WHEELS


def _wheelSpeedOnClose(self, T, event):
    _clearWindowOpenStatus(T)


def _wheelSpeedOnOpen(self):
    pass


# CURRENT


def _currentConsumptionOnClose(self, T, event):
    _clearWindowOpenStatus(T)


def _currentConsumptionOnOpen(self):
    pass


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


def openBatteryInfo(parent):
    _openWindow(gui.battery.Ui_BatteryInfo, parent, _batteryInfoOnOpen, _batteryInfoOnClose)


def openWheelSpeed(parent):
    _openWindow(gui.elec.wheel_speed.Ui_WheelSpeed, parent, _wheelSpeedOnOpen, _wheelSpeedOnClose)


def openCurrentConsumption(parent):
    _openWindow(gui.elec.current_consumption.Ui_CurrentConsumption, parent, _currentConsumptionOnOpen, _currentConsumptionOnClose)