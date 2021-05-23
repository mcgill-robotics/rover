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


def clearWindowOpenStatus(T):
    windowOpenStatus[T] = None


def batteryInfoOnClose(self, T, event):
    clearWindowOpenStatus(T)


def wheelSpeedOnClose(self, T, event):
    clearWindowOpenStatus(T)


def openWindow(T, parent, onClose):
    if T in windowOpenStatus.keys():
        if windowOpenStatus[T] is not None:
            return
    wid = QtWidgets.QWidget()
    parent.windows.append(wid)
    wnd = T()
    wnd.setupUi(wid)
    wid.closeEvent = partial(onClose, wid, T)
    wid.show()
    windowOpenStatus[T] = wid


def openBatteryInfo(parent):
    openWindow(gui.battery.Ui_BatteryInfo, parent, batteryInfoOnClose)


def openWheelSpeed(parent):
    openWindow(gui.elec.wheel_speed.Ui_WheelSpeed, parent, wheelSpeedOnClose)
