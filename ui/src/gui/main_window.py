# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'main.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(800, 600)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 800, 22))
        self.menubar.setObjectName("menubar")
        self.menuFile = QtWidgets.QMenu(self.menubar)
        self.menuFile.setObjectName("menuFile")
        self.menuWindow = QtWidgets.QMenu(self.menubar)
        self.menuWindow.setObjectName("menuWindow")
        self.menuWindows = QtWidgets.QMenu(self.menuWindow)
        self.menuWindows.setObjectName("menuWindows")
        self.menuElectrical = QtWidgets.QMenu(self.menuWindows)
        self.menuElectrical.setObjectName("menuElectrical")
        self.menuMechanical = QtWidgets.QMenu(self.menuWindows)
        self.menuMechanical.setObjectName("menuMechanical")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)
        self.actionOpen = QtWidgets.QAction(MainWindow)
        self.actionOpen.setObjectName("actionOpen")
        self.actionExit = QtWidgets.QAction(MainWindow)
        self.actionExit.setShortcutContext(QtCore.Qt.ApplicationShortcut)
        self.actionExit.setObjectName("actionExit")
        self.actionOpen_2 = QtWidgets.QAction(MainWindow)
        self.actionOpen_2.setObjectName("actionOpen_2")
        self.actionPower_Info = QtWidgets.QAction(MainWindow)
        self.actionPower_Info.setObjectName("actionPower_Info")
        self.actionScience = QtWidgets.QAction(MainWindow)
        self.actionScience.setObjectName("actionScience")
        self.actionVideoFeeds = QtWidgets.QAction(MainWindow)
        self.actionVideoFeeds.setObjectName("actionVideoFeeds")
        self.actionPowerInfo = QtWidgets.QAction(MainWindow)
        self.actionPowerInfo.setObjectName("actionPowerInfo")
        self.actionCurrents = QtWidgets.QAction(MainWindow)
        self.actionCurrents.setObjectName("actionCurrents")
        self.actionArm = QtWidgets.QAction(MainWindow)
        self.actionArm.setObjectName("actionArm")
        self.actionWheels = QtWidgets.QAction(MainWindow)
        self.actionWheels.setObjectName("actionWheels")
        self.actionWheelSpeeds = QtWidgets.QAction(MainWindow)
        self.actionWheelSpeeds.setObjectName("actionWheelSpeeds")
        self.actionReset = QtWidgets.QAction(MainWindow)
        self.actionReset.setShortcutContext(QtCore.Qt.ApplicationShortcut)
        self.actionReset.setObjectName("actionReset")
        self.actionStatus = QtWidgets.QAction(MainWindow)
        self.actionStatus.setObjectName("actionStatus")
        self.menuFile.addAction(self.actionExit)
        self.menuElectrical.addAction(self.actionPowerInfo)
        self.menuElectrical.addAction(self.actionCurrents)
        self.menuElectrical.addAction(self.actionWheelSpeeds)
        self.menuMechanical.addAction(self.actionArm)
        self.menuWindows.addAction(self.actionStatus)
        self.menuWindows.addAction(self.menuElectrical.menuAction())
        self.menuWindows.addAction(self.menuMechanical.menuAction())
        self.menuWindows.addAction(self.actionScience)
        self.menuWindows.addAction(self.actionVideoFeeds)
        self.menuWindow.addSeparator()
        self.menuWindow.addAction(self.menuWindows.menuAction())
        self.menuWindow.addAction(self.actionReset)
        self.menubar.addAction(self.menuFile.menuAction())
        self.menubar.addAction(self.menuWindow.menuAction())

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.menuFile.setTitle(_translate("MainWindow", "File"))
        self.menuWindow.setTitle(_translate("MainWindow", "Window"))
        self.menuWindows.setTitle(_translate("MainWindow", "Windows"))
        self.menuElectrical.setTitle(_translate("MainWindow", "Electrical"))
        self.menuMechanical.setTitle(_translate("MainWindow", "Mechanical"))
        self.actionOpen.setText(_translate("MainWindow", "Open"))
        self.actionExit.setText(_translate("MainWindow", "Exit"))
        self.actionExit.setShortcut(_translate("MainWindow", "Ctrl+Q"))
        self.actionOpen_2.setText(_translate("MainWindow", "Windows"))
        self.actionPower_Info.setText(_translate("MainWindow", "Power Info"))
        self.actionScience.setText(_translate("MainWindow", "Science"))
        self.actionVideoFeeds.setText(_translate("MainWindow", "Video Feeds"))
        self.actionPowerInfo.setText(_translate("MainWindow", "Power Info"))
        self.actionCurrents.setText(_translate("MainWindow", "Current Info"))
        self.actionArm.setText(_translate("MainWindow", "Arm"))
        self.actionWheels.setText(_translate("MainWindow", "Wheels"))
        self.actionWheelSpeeds.setText(_translate("MainWindow", "Wheels"))
        self.actionReset.setText(_translate("MainWindow", "Reset"))
        self.actionReset.setShortcut(_translate("MainWindow", "Ctrl+R"))
        self.actionStatus.setText(_translate("MainWindow", "Status"))