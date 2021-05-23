import sys
import rospy as rp 
from std_msgs import *
from PyQt5 import QtCore, QtGui, QtWidgets, uic
import asyncio
from functools import partial

import gui.main_window
import gui.battery
import gui.elec.current_consumption
import gui.elec.wheel_speed


windowOpenStatus = {}


def _clearWindowOpenStatus(T):
    windowOpenStatus[T] = None


def _batteryInfoOnClose(self, T, event):
    _clearWindowOpenStatus(T)


def _wheelSpeedOnClose(self, T, event):
    _clearWindowOpenStatus(T)


def _openWindow(T, parent, onClose):
    if T in windowOpenStatus.keys():
        if windowOpenStatus[T] is not None:
            theWidget: QtWidgets.QWidget = windowOpenStatus[T]
            theWidget.activateWindow()
            return
            
    wid = QtWidgets.QWidget()
    parent.windows.append(wid)
    wnd = T()
    wnd.setupUi(wid)
    wid.closeEvent = partial(onClose, wid, T)
    wid.show()
    windowOpenStatus[T] = wid


def openBatteryInfo(parent):
    _openWindow(gui.battery.Ui_BatteryInfo, parent, _batteryInfoOnClose)


def openWheelSpeed(parent):
    _openWindow(gui.elec.wheel_speed.Ui_WheelSpeed, parent, _wheelSpeedOnClose)
