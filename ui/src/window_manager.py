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


def openBatteryInfo(parent):
    T = type(gui.battery.Ui_BatteryInfo)
    if T in windowOpenStatus.keys():
        if windowOpenStatus[T] is not None:
            return
    parent.batWnd = QtWidgets.QWidget()
    parent.bat = gui.battery.Ui_BatteryInfo()
    parent.bat.setupUi(parent.batWnd)
    parent.batWnd.closeEvent = partial(batteryInfoOnClose, parent.batWnd, T)
    parent.batWnd.show()
    windowOpenStatus[T] = parent.batWnd

