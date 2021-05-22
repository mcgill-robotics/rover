# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file '../qt_ui/elec/current_consumption.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_CurrentConsumption(object):
    def setupUi(self, CurrentConsumption):
        CurrentConsumption.setObjectName("CurrentConsumption")
        CurrentConsumption.resize(200, 110)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(CurrentConsumption.sizePolicy().hasHeightForWidth())
        CurrentConsumption.setSizePolicy(sizePolicy)
        CurrentConsumption.setMinimumSize(QtCore.QSize(200, 110))
        CurrentConsumption.setMaximumSize(QtCore.QSize(200, 110))
        self.lblMotor = QtWidgets.QLabel(CurrentConsumption)
        self.lblMotor.setGeometry(QtCore.QRect(10, 70, 91, 20))
        self.lblMotor.setObjectName("lblMotor")
        self.lblCurrentConsumption = QtWidgets.QLabel(CurrentConsumption)
        self.lblCurrentConsumption.setGeometry(QtCore.QRect(10, 10, 171, 20))
        self.lblCurrentConsumption.setObjectName("lblCurrentConsumption")
        self.txtCpu = QtWidgets.QPlainTextEdit(CurrentConsumption)
        self.txtCpu.setGeometry(QtCore.QRect(110, 30, 71, 31))
        self.txtCpu.viewport().setProperty("cursor", QtGui.QCursor(QtCore.Qt.ForbiddenCursor))
        self.txtCpu.setReadOnly(True)
        self.txtCpu.setObjectName("txtCpu")
        self.lblCpu = QtWidgets.QLabel(CurrentConsumption)
        self.lblCpu.setGeometry(QtCore.QRect(10, 40, 91, 20))
        self.lblCpu.setObjectName("lblCpu")
        self.txtMotor = QtWidgets.QPlainTextEdit(CurrentConsumption)
        self.txtMotor.setGeometry(QtCore.QRect(110, 70, 71, 31))
        self.txtMotor.viewport().setProperty("cursor", QtGui.QCursor(QtCore.Qt.ForbiddenCursor))
        self.txtMotor.setReadOnly(True)
        self.txtMotor.setObjectName("txtMotor")

        self.retranslateUi(CurrentConsumption)
        QtCore.QMetaObject.connectSlotsByName(CurrentConsumption)

    def retranslateUi(self, CurrentConsumption):
        _translate = QtCore.QCoreApplication.translate
        CurrentConsumption.setWindowTitle(_translate("CurrentConsumption", "Current Consumption"))
        self.lblMotor.setText(_translate("CurrentConsumption", "Motor (mA):"))
        self.lblCurrentConsumption.setText(_translate("CurrentConsumption", "Current Consumption"))
        self.lblCpu.setText(_translate("CurrentConsumption", "CPU (mA):"))
