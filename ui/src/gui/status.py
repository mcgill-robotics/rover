# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'status.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_Status(object):
    def setupUi(self, Status):
        Status.setObjectName("Status")
        Status.resize(400, 300)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(Status.sizePolicy().hasHeightForWidth())
        Status.setSizePolicy(sizePolicy)
        Status.setMinimumSize(QtCore.QSize(400, 300))
        Status.setMaximumSize(QtCore.QSize(400, 300))
        self.txtStatus = QtWidgets.QTextEdit(Status)
        self.txtStatus.setGeometry(QtCore.QRect(10, 10, 381, 281))
        self.txtStatus.setObjectName("txtStatus")

        self.retranslateUi(Status)
        QtCore.QMetaObject.connectSlotsByName(Status)

    def retranslateUi(self, Status):
        _translate = QtCore.QCoreApplication.translate
        Status.setWindowTitle(_translate("Status", "Status"))
