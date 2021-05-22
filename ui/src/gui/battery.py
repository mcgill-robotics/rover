# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file '../qt_ui/battery.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_BatteryInfo(object):
    def setupUi(self, BatteryInfo):
        BatteryInfo.setObjectName("BatteryInfo")
        BatteryInfo.resize(200, 110)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(BatteryInfo.sizePolicy().hasHeightForWidth())
        BatteryInfo.setSizePolicy(sizePolicy)
        BatteryInfo.setMinimumSize(QtCore.QSize(200, 110))
        BatteryInfo.setMaximumSize(QtCore.QSize(200, 110))
        self.lblBattery = QtWidgets.QLabel(BatteryInfo)
        self.lblBattery.setGeometry(QtCore.QRect(10, 10, 171, 20))
        self.lblBattery.setObjectName("lblBattery")
        self.lblVoltage = QtWidgets.QLabel(BatteryInfo)
        self.lblVoltage.setGeometry(QtCore.QRect(10, 40, 91, 20))
        self.lblVoltage.setObjectName("lblVoltage")
        self.lblCurrent = QtWidgets.QLabel(BatteryInfo)
        self.lblCurrent.setGeometry(QtCore.QRect(10, 70, 91, 20))
        self.lblCurrent.setObjectName("lblCurrent")
        self.txtVoltage = QtWidgets.QPlainTextEdit(BatteryInfo)
        self.txtVoltage.setGeometry(QtCore.QRect(110, 30, 71, 31))
        self.txtVoltage.viewport().setProperty("cursor", QtGui.QCursor(QtCore.Qt.ForbiddenCursor))
        self.txtVoltage.setReadOnly(True)
        self.txtVoltage.setObjectName("txtVoltage")
        self.txtCurrent = QtWidgets.QPlainTextEdit(BatteryInfo)
        self.txtCurrent.setGeometry(QtCore.QRect(110, 70, 71, 31))
        self.txtCurrent.viewport().setProperty("cursor", QtGui.QCursor(QtCore.Qt.ForbiddenCursor))
        self.txtCurrent.setReadOnly(True)
        self.txtCurrent.setObjectName("txtCurrent")

        self.retranslateUi(BatteryInfo)
        QtCore.QMetaObject.connectSlotsByName(BatteryInfo)

    def retranslateUi(self, BatteryInfo):
        _translate = QtCore.QCoreApplication.translate
        BatteryInfo.setWindowTitle(_translate("BatteryInfo", "Power"))
        self.lblBattery.setText(_translate("BatteryInfo", "Battery Status"))
        self.lblVoltage.setText(_translate("BatteryInfo", "Voltage (V):"))
        self.lblCurrent.setText(_translate("BatteryInfo", "Current (A): "))
