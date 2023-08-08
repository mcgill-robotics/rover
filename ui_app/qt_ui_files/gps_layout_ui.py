# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'gps_layout.ui'
##
## Created by: Qt User Interface Compiler version 5.15.2
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide2.QtCore import *
from PySide2.QtGui import *
from PySide2.QtWidgets import *


class Ui_GPS(object):
    def setupUi(self, GPS):
        if not GPS.objectName():
            GPS.setObjectName(u"GPS")
        GPS.resize(576, 648)
        self.GPS_Plot = QLabel(GPS)
        self.GPS_Plot.setObjectName(u"GPS_Plot")
        self.GPS_Plot.setGeometry(QRect(20, 10, 121, 41))
        font = QFont()
        font.setPointSize(17)
        self.GPS_Plot.setFont(font)
        self.plot = QLabel(GPS)
        self.plot.setObjectName(u"plot")
        self.plot.setGeometry(QRect(20, 60, 491, 311))
        font1 = QFont()
        font1.setPointSize(14)
        self.plot.setFont(font1)
        self.plot.setFrameShape(QFrame.Box)
        self.plot.setLineWidth(2)

        self.retranslateUi(GPS)

        QMetaObject.connectSlotsByName(GPS)
    # setupUi

    def retranslateUi(self, GPS):
        GPS.setWindowTitle(QCoreApplication.translate("GPS", u"GPS", None))
        self.GPS_Plot.setText(QCoreApplication.translate("GPS", u"GPS_Plot", None))
        self.plot.setText(QCoreApplication.translate("GPS", u"plot", None))
    # retranslateUi

