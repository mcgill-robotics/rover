import sys
import rospy as rp 
from std_msgs import *
from PyQt5 import QtCore, QtGui, QtWidgets, uic
import asyncio
import gui.main_window


ui: gui.main_window.Ui_MainWindow = None
pub: rp.Publisher = None


def pbClicked():
    pub.publish("test")


def onTestReceived(data: msg.String):
    data: str = str(data)


async def main():
    rp.init_node("ui")
    app = QtWidgets.QApplication(sys.argv)
    wnd = QtWidgets.QMainWindow()
    ui = gui.main_window.Ui_MainWindow()
    sub = rp.Subscriber("test", msg.String, onTestReceived)
    pub = rp.Publisher("test2", msg.String)
    ui.setupUi(wnd)
    pb: QtWidgets.QPushButton = ui.pushButton
    pb.clicked.connect(pbClicked)
    wnd.show()
    app.exec()


if __name__ == "__main__":
    asyncio.run(main())
