# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'ui_layout.ui'
##
## Created by: Qt User Interface Compiler version 5.15.2
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide2.QtCore import *
from PySide2.QtGui import *
from PySide2.QtWidgets import *


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(886, 628)
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.Systemsview = QTabWidget(self.centralwidget)
        self.Systemsview.setObjectName(u"Systemsview")
        self.Systemsview.setGeometry(QRect(470, 10, 351, 531))
        self.Arm = QWidget()
        self.Arm.setObjectName(u"Arm")
        self.Systemsview.addTab(self.Arm, "")
        self.Drive = QWidget()
        self.Drive.setObjectName(u"Drive")
        self.Systemsview.addTab(self.Drive, "")
        self.Science = QWidget()
        self.Science.setObjectName(u"Science")
        self.Systemsview.addTab(self.Science, "")
        self.Power = QWidget()
        self.Power.setObjectName(u"Power")
        self.Systemsview.addTab(self.Power, "")
        self.GPS = QWidget()
        self.GPS.setObjectName(u"GPS")
        self.Systemsview.addTab(self.GPS, "")
        self.camera_selector = QComboBox(self.centralwidget)
        self.camera_selector.addItem("")
        self.camera_selector.addItem("")
        self.camera_selector.addItem("")
        self.camera_selector.addItem("")
        self.camera_selector.addItem("")
        self.camera_selector.addItem("")
        self.camera_selector.setObjectName(u"camera_selector")
        self.camera_selector.setGeometry(QRect(10, 290, 131, 31))
        self.Camera = QLabel(self.centralwidget)
        self.Camera.setObjectName(u"Camera")
        self.Camera.setGeometry(QRect(10, 10, 411, 271))
        font = QFont()
        font.setPointSize(14)
        self.Camera.setFont(font)
        self.Camera.setFrameShape(QFrame.Box)
        self.Camera.setLineWidth(2)
        self.OverallFeedback = QWidget(self.centralwidget)
        self.OverallFeedback.setObjectName(u"OverallFeedback")
        self.OverallFeedback.setGeometry(QRect(10, 360, 311, 191))
        self.label = QLabel(self.centralwidget)
        self.label.setObjectName(u"label")
        self.label.setGeometry(QRect(10, 340, 151, 17))
        self.label.setFont(font)
        self.control_selector = QComboBox(self.centralwidget)
        self.control_selector.addItem("")
        self.control_selector.addItem("")
        self.control_selector.addItem("")
        self.control_selector.addItem("")
        self.control_selector.setObjectName(u"control_selector")
        self.control_selector.setGeometry(QRect(160, 290, 181, 31))
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QMenuBar(MainWindow)
        self.menubar.setObjectName(u"menubar")
        self.menubar.setGeometry(QRect(0, 0, 886, 22))
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QStatusBar(MainWindow)
        self.statusbar.setObjectName(u"statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)

        self.Systemsview.setCurrentIndex(4)


        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"Rover UI", None))
        self.Systemsview.setTabText(self.Systemsview.indexOf(self.Arm), QCoreApplication.translate("MainWindow", u"Arm", None))
        self.Systemsview.setTabText(self.Systemsview.indexOf(self.Drive), QCoreApplication.translate("MainWindow", u"Drive ", None))
        self.Systemsview.setTabText(self.Systemsview.indexOf(self.Science), QCoreApplication.translate("MainWindow", u"Science", None))
        self.Systemsview.setTabText(self.Systemsview.indexOf(self.Power), QCoreApplication.translate("MainWindow", u"Power", None))
        self.Systemsview.setTabText(self.Systemsview.indexOf(self.GPS), QCoreApplication.translate("MainWindow", u"GPS", None))
        self.camera_selector.setItemText(0, QCoreApplication.translate("MainWindow", u"Cam 1", None))
        self.camera_selector.setItemText(1, QCoreApplication.translate("MainWindow", u"Cam 2", None))
        self.camera_selector.setItemText(2, QCoreApplication.translate("MainWindow", u"Cam 3", None))
        self.camera_selector.setItemText(3, QCoreApplication.translate("MainWindow", u"Cam 4", None))
        self.camera_selector.setItemText(4, QCoreApplication.translate("MainWindow", u"Cam 5", None))
        self.camera_selector.setItemText(5, QCoreApplication.translate("MainWindow", u"Cam 6", None))

        self.camera_selector.setCurrentText(QCoreApplication.translate("MainWindow", u"Cam 1", None))
        self.Camera.setText(QCoreApplication.translate("MainWindow", u"Camera (add pixmap)", None))
        self.label.setText(QCoreApplication.translate("MainWindow", u"Overall Feedback", None))
        self.control_selector.setItemText(0, QCoreApplication.translate("MainWindow", u"Arm", None))
        self.control_selector.setItemText(1, QCoreApplication.translate("MainWindow", u"Drive", None))
        self.control_selector.setItemText(2, QCoreApplication.translate("MainWindow", u"Science", None))
        self.control_selector.setItemText(3, QCoreApplication.translate("MainWindow", u"Power", None))

    # retranslateUi

