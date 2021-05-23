import sys
import rospy as rp 
from std_msgs import *
from PyQt5 import QtCore, QtGui, QtWidgets, uic
import asyncio
from functools import partial
from window_manager import *
import gui.main_window
import gui.battery
import gui.elec.current_consumption
import gui.elec.wheel_speed


ui: gui.main_window.Ui_MainWindow = None


def pbClicked(self):
    openBatteryInfo(self)


def showMain():
    wnd = QtWidgets.QMainWindow()
    ui = gui.main_window.Ui_MainWindow()
    ui.setupUi(wnd)
    wnd.show()


def onTestReceived(data: msg.String):
    data: str = str(data)
    print(data)


def exitProgram():
    exit(0)


def setUpMainWindowHandlers(mainWnd):
    mainWnd.actionExit.triggered.connect(exitProgram)
    mainWnd.actionPowerInfo.triggered.connect(partial(openBatteryInfo, mainWnd))


async def main():
    rp.init_node("ui")
    app = QtWidgets.QApplication(sys.argv)
    wnd = QtWidgets.QMainWindow()
    ui = gui.main_window.Ui_MainWindow()
    sub = rp.Subscriber("test", msg.String, onTestReceived)
    global pub
    pub = rp.Publisher("test2", msg.String, queue_size=1)
    ui.setupUi(wnd)
    ui.windows = []
    setUpMainWindowHandlers(ui)
    wnd.show()
    app.exec()


if __name__ == "__main__":
    asyncio.run(main())
