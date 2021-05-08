#!/usr/bin/python3


import sys
import rospy as rp 
from std_msgs import *
from PyQt5 import QtCore, QtGui, QtWidgets, uic
import asyncio
import gui.main_window


def pbClicked():
    rp.loginfo("Hello")


def main():
    rp.init_node("ui")
    app = QtWidgets.QApplication(sys.argv)
    wnd = QtWidgets.QMainWindow()
    ui = gui.main_window.Ui_MainWindow()
    ui.setupUi(wnd)
    pb: QtWidgets.QPushButton = ui.pushButton
    pb.clicked.connect(pbClicked)
    wnd.show()
    app.exec()


if __name__ == "__main__":
    main()
