# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'gps_layout.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_GPS(QtWidgets.QWidget):
    def setupUi(self, GPS):
        GPS.setObjectName("GPS")
        GPS.resize(506, 648)
        self.verticalLayoutWidget = QtWidgets.QWidget(GPS)
        self.verticalLayoutWidget.setGeometry(QtCore.QRect(10, 20, 481, 435))
        self.verticalLayoutWidget.setObjectName("verticalLayoutWidget")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.verticalLayoutWidget)
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout.setObjectName("verticalLayout")
        self.GPS_Plot = QtWidgets.QLabel(self.verticalLayoutWidget)
        font = QtGui.QFont()
        font.setPointSize(17)
        self.GPS_Plot.setFont(font)
        self.GPS_Plot.setObjectName("GPS_Plot")
        self.verticalLayout.addWidget(self.GPS_Plot)
        self.gps_plotter = QtWidgets.QWidget(self.verticalLayoutWidget)
        self.gps_plotter.setMinimumSize(QtCore.QSize(0, 400))
        self.gps_plotter.setStyleSheet("background-color: rgb(0, 0, 0);")
        self.gps_plotter.setObjectName("gps_plotter")
        self.verticalLayout.addWidget(self.gps_plotter)

        self.retranslateUi(GPS)
        QtCore.QMetaObject.connectSlotsByName(GPS)

    def retranslateUi(self, GPS):
        _translate = QtCore.QCoreApplication.translate
        GPS.setWindowTitle(_translate("GPS", "GPS"))
        self.GPS_Plot.setText(_translate("GPS", "GPS_Plot"))