# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'base_layout.ui'
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
        self.Systemsview = QtWidgets.QTabWidget(self.centralwidget)
        self.Systemsview.setGeometry(QtCore.QRect(470, 10, 301, 531))
        self.Systemsview.setObjectName("Systemsview")
        self.Arm = QtWidgets.QWidget()
        self.Arm.setObjectName("Arm")
        self.Systemsview.addTab(self.Arm, "")
        self.Drive = QtWidgets.QWidget()
        self.Drive.setObjectName("Drive")
        self.Systemsview.addTab(self.Drive, "")
        self.Science = QtWidgets.QWidget()
        self.Science.setObjectName("Science")
        self.Systemsview.addTab(self.Science, "")
        self.Autonomy = QtWidgets.QWidget()
        self.Autonomy.setObjectName("Autonomy")
        self.Systemsview.addTab(self.Autonomy, "")
        self.comboBox = QtWidgets.QComboBox(self.centralwidget)
        self.comboBox.setGeometry(QtCore.QRect(10, 290, 131, 31))
        self.comboBox.setObjectName("comboBox")
        self.comboBox.addItem("")
        self.comboBox.addItem("")
        self.comboBox.addItem("")
        self.comboBox.addItem("")
        self.comboBox.addItem("")
        self.comboBox.addItem("")
        self.Camera = QtWidgets.QLabel(self.centralwidget)
        self.Camera.setGeometry(QtCore.QRect(10, 10, 411, 271))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.Camera.setFont(font)
        self.Camera.setFrameShape(QtWidgets.QFrame.Box)
        self.Camera.setLineWidth(2)
        self.Camera.setObjectName("Camera")
        self.OverallFeedback = QtWidgets.QWidget(self.centralwidget)
        self.OverallFeedback.setGeometry(QtCore.QRect(10, 360, 311, 191))
        self.OverallFeedback.setObjectName("OverallFeedback")
        self.label = QtWidgets.QLabel(self.centralwidget)
        self.label.setGeometry(QtCore.QRect(10, 340, 151, 17))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label.setFont(font)
        self.label.setObjectName("label")
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 800, 22))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        self.Systemsview.setCurrentIndex(0)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "Rover UI"))
        self.Systemsview.setTabText(self.Systemsview.indexOf(self.Arm), _translate("MainWindow", "Arm"))
        self.Systemsview.setTabText(self.Systemsview.indexOf(self.Drive), _translate("MainWindow", "Drive "))
        self.Systemsview.setTabText(self.Systemsview.indexOf(self.Science), _translate("MainWindow", "Science"))
        self.Systemsview.setTabText(self.Systemsview.indexOf(self.Autonomy), _translate("MainWindow", "Autonomy"))
        self.comboBox.setCurrentText(_translate("MainWindow", "Cam 1"))
        self.comboBox.setItemText(0, _translate("MainWindow", "Cam 1"))
        self.comboBox.setItemText(1, _translate("MainWindow", "Cam 2"))
        self.comboBox.setItemText(2, _translate("MainWindow", "Cam 3"))
        self.comboBox.setItemText(3, _translate("MainWindow", "Cam 4"))
        self.comboBox.setItemText(4, _translate("MainWindow", "Cam 5"))
        self.comboBox.setItemText(5, _translate("MainWindow", "Cam 6"))
        self.Camera.setText(_translate("MainWindow", "Camera (add pixmap)"))
        self.label.setText(_translate("MainWindow", "Overall Feedback"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())
