# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file '../qt_ui/elec/wheel_speed.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_WheelSpeed(object):
    def setupUi(self, WheelSpeed):
        WheelSpeed.setObjectName("WheelSpeed")
        WheelSpeed.resize(230, 140)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(WheelSpeed.sizePolicy().hasHeightForWidth())
        WheelSpeed.setSizePolicy(sizePolicy)
        WheelSpeed.setMinimumSize(QtCore.QSize(230, 140))
        WheelSpeed.setMaximumSize(QtCore.QSize(300, 140))
        WheelSpeed.setBaseSize(QtCore.QSize(0, 0))
        self.txtFL = QtWidgets.QPlainTextEdit(WheelSpeed)
        self.txtFL.setGeometry(QtCore.QRect(40, 40, 71, 41))
        self.txtFL.viewport().setProperty("cursor", QtGui.QCursor(QtCore.Qt.ForbiddenCursor))
        self.txtFL.setReadOnly(True)
        self.txtFL.setObjectName("txtFL")
        self.lblWheelSpeed = QtWidgets.QLabel(WheelSpeed)
        self.lblWheelSpeed.setGeometry(QtCore.QRect(10, 10, 171, 20))
        self.lblWheelSpeed.setObjectName("lblWheelSpeed")
        self.lblFR = QtWidgets.QLabel(WheelSpeed)
        self.lblFR.setGeometry(QtCore.QRect(200, 50, 21, 20))
        self.lblFR.setObjectName("lblFR")
        self.txtFR = QtWidgets.QPlainTextEdit(WheelSpeed)
        self.txtFR.setGeometry(QtCore.QRect(120, 40, 71, 41))
        self.txtFR.viewport().setProperty("cursor", QtGui.QCursor(QtCore.Qt.ForbiddenCursor))
        self.txtFR.setReadOnly(True)
        self.txtFR.setObjectName("txtFR")
        self.lblFL = QtWidgets.QLabel(WheelSpeed)
        self.lblFL.setGeometry(QtCore.QRect(10, 50, 16, 20))
        self.lblFL.setObjectName("lblFL")
        self.lblRR = QtWidgets.QLabel(WheelSpeed)
        self.lblRR.setGeometry(QtCore.QRect(200, 100, 21, 20))
        self.lblRR.setObjectName("lblRR")
        self.lblRL = QtWidgets.QLabel(WheelSpeed)
        self.lblRL.setGeometry(QtCore.QRect(10, 100, 16, 20))
        self.lblRL.setObjectName("lblRL")
        self.txtRL = QtWidgets.QPlainTextEdit(WheelSpeed)
        self.txtRL.setGeometry(QtCore.QRect(40, 90, 71, 41))
        self.txtRL.viewport().setProperty("cursor", QtGui.QCursor(QtCore.Qt.ForbiddenCursor))
        self.txtRL.setReadOnly(True)
        self.txtRL.setObjectName("txtRL")
        self.txtRR = QtWidgets.QPlainTextEdit(WheelSpeed)
        self.txtRR.setGeometry(QtCore.QRect(120, 90, 71, 41))
        self.txtRR.viewport().setProperty("cursor", QtGui.QCursor(QtCore.Qt.ForbiddenCursor))
        self.txtRR.setReadOnly(True)
        self.txtRR.setObjectName("txtRR")

        self.retranslateUi(WheelSpeed)
        QtCore.QMetaObject.connectSlotsByName(WheelSpeed)

    def retranslateUi(self, WheelSpeed):
        _translate = QtCore.QCoreApplication.translate
        WheelSpeed.setWindowTitle(_translate("WheelSpeed", "Wheel Speed"))
        self.lblWheelSpeed.setText(_translate("WheelSpeed", "Wheel Speed (rpm)"))
        self.lblFR.setText(_translate("WheelSpeed", "FR"))
        self.lblFL.setText(_translate("WheelSpeed", "FL"))
        self.lblRR.setText(_translate("WheelSpeed", "RR"))
        self.lblRL.setText(_translate("WheelSpeed", "RL"))
