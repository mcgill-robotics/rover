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


ui: gui.main_window.Ui_MainWindow = None


def pbClicked(self):
    self.batWid = QtWidgets.QWidget()
    self.bat = gui.elec.wheel_speed.Ui_WheelSpeed()
    self.bat.setupUi(self.batWid)
    self.batWid.show()


def showMain():
    wnd = QtWidgets.QMainWindow()
    ui = gui.main_window.Ui_MainWindow()
    ui.setupUi(wnd)
    wnd.show()


def onTestReceived(data: msg.String):
    data: str = str(data)
    print(data)


async def main():
    rp.init_node("ui")
    app = QtWidgets.QApplication(sys.argv)
    wnd = QtWidgets.QMainWindow()
    ui = gui.main_window.Ui_MainWindow()
    sub = rp.Subscriber("test", msg.String, onTestReceived)
    global pub
    pub = rp.Publisher("test2", msg.String, queue_size=1)
    ui.setupUi(wnd)
    pb: QtWidgets.QPushButton = ui.pushButton
    pb.clicked.connect(partial(pbClicked, pb))
    wnd.show()
    app.exec()


if __name__ == "__main__":
    asyncio.run(main())
