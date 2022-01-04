# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file '../qt_ui_files/control_selection.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_Form(object):
    def setupUi(self, Form):
        Form.setObjectName("Form")
        Form.resize(222, 105)
        self.controlSelector = QtWidgets.QComboBox(Form)
        self.controlSelector.setGeometry(QtCore.QRect(10, 40, 121, 31))
        self.controlSelector.setObjectName("controlSelector")
        self.controlSelector.addItem("")
        self.controlSelector.addItem("")
        self.controlSelector.addItem("")
        self.controlSelector.addItem("")
        self.label = QtWidgets.QLabel(Form)
        self.label.setGeometry(QtCore.QRect(10, 20, 131, 17))
        self.label.setObjectName("label")

        self.retranslateUi(Form)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        _translate = QtCore.QCoreApplication.translate
        Form.setWindowTitle(_translate("Form", "Form"))
        self.controlSelector.setItemText(0, _translate("Form", "Arm "))
        self.controlSelector.setItemText(1, _translate("Form", "Drive"))
        self.controlSelector.setItemText(2, _translate("Form", "Science"))
        self.controlSelector.setItemText(3, _translate("Form", "Autonomy"))
        self.label.setText(_translate("Form", "Control Selector"))
