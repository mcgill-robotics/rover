import sys
from threading import Thread
import rospy as rp
from rospy.topics import Subscriber 
from std_msgs import *
from PyQt5 import QtCore, QtGui, QtWidgets, uic
import asyncio
from functools import partial
import time
from window_manager import *
import gui.main_window
import gui.battery
import gui.elec.current_consumption
import gui.elec.wheel_speed
import signal
import threading
import queue
from config import *
from gui.video_feed import SingleVideoScreen

ui: gui.main_window.Ui_MainWindow = None


def pbClicked(self):
    openBatteryInfo(self)

def onTestReceived(data: msg.String):
    data: str = str(data)
    print(data)

def mainWindowOnClose(event):
    exitProgram()


def exitProgram(code=0):
    closeAllWindows()
    exit(code)

def setUpMainWindowHandlers(mainWnd : gui.main_window.Ui_MainWindow):
    mainWnd.actionExit.triggered.connect(exitProgram)
    mainWnd.actionReset.triggered.connect(closeAllWindows)
    mainWnd.actionPowerInfo.triggered.connect(partial(openBatteryInfo, mainWnd))
    mainWnd.actionWheelSpeeds.triggered.connect(partial(openWheelSpeed, mainWnd))
    mainWnd.actionCurrents.triggered.connect(partial(openCurrentConsumption, mainWnd))
    mainWnd.actionStatus.triggered.connect(partial(openStatus, mainWnd))
    mainWnd.actionVideoFeeds.triggered.connect(partial(openVideoFeeds, mainWnd))
    mainWnd.actionScience.triggered.connect(partial(openScience, mainWnd))


def main():
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
    wnd.closeEvent = mainWindowOnClose
    wnd.show()
    # interrupt handling
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    t = Thread()
    global _checkQueueSig
    _checkQueueSig = BindedFunction()
    _checkQueueSig.triggered.connect(_checkQueue)
    t.start()

    app.exec()

def queueMethodForMain(method, *args):
    methodQueue.put((method, args))


class BindedFunction(QtCore.QObject):
    triggered = QtCore.pyqtSignal()

    def __init__(self):
        super().__init__()

class Thread(QtCore.QThread):
    def run(self):
        while True:
            QtCore.QCoreApplication.processEvents()
            _checkQueueSig.triggered.emit()
            

def _checkQueue():
    try:
        func, args = methodQueue.get(block=False, timeout=0.00001)
        func(*args)
    except Exception as ex:
        pass


if __name__ == "__main__":
    main()
