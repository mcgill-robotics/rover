# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'qt_ui_files/arm_layout.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_Arm(QtWidgets.QWidget):
    def setupUi(self, Arm):
        Arm.setObjectName("Arm")
        Arm.resize(368, 619)
        Arm.setAutoFillBackground(True)
        self.arm_joints = QtWidgets.QLabel(Arm)
        self.arm_joints.setGeometry(QtCore.QRect(10, 10, 121, 41))
        font = QtGui.QFont()
        font.setPointSize(17)
        self.arm_joints.setFont(font)
        self.arm_joints.setObjectName("arm_joints")
        self.joint_Shoulder = QtWidgets.QLabel(Arm)
        self.joint_Shoulder.setGeometry(QtCore.QRect(120, 60, 81, 17))
        self.joint_Shoulder.setObjectName("joint_Shoulder")
        self.joint_one_value = QtWidgets.QLCDNumber(Arm)
        self.joint_one_value.setGeometry(QtCore.QRect(10, 80, 71, 31))
        self.joint_one_value.setObjectName("joint_one_value")
        self.joint_two_value = QtWidgets.QLCDNumber(Arm)
        self.joint_two_value.setGeometry(QtCore.QRect(120, 80, 71, 31))
        self.joint_two_value.setObjectName("joint_two_value")
        self.joint_Elbow = QtWidgets.QLabel(Arm)
        self.joint_Elbow.setGeometry(QtCore.QRect(230, 60, 71, 16))
        self.joint_Elbow.setObjectName("joint_Elbow")
        self.joint_three_value = QtWidgets.QLCDNumber(Arm)
        self.joint_three_value.setGeometry(QtCore.QRect(230, 80, 71, 31))
        self.joint_three_value.setObjectName("joint_three_value")
        self.joint_Wrist = QtWidgets.QLabel(Arm)
        self.joint_Wrist.setGeometry(QtCore.QRect(10, 130, 71, 17))
        self.joint_Wrist.setObjectName("joint_Wrist")
        self.joint_four_value = QtWidgets.QLCDNumber(Arm)
        self.joint_four_value.setGeometry(QtCore.QRect(10, 150, 71, 31))
        self.joint_four_value.setObjectName("joint_four_value")
        self.joint_five_value = QtWidgets.QLCDNumber(Arm)
        self.joint_five_value.setGeometry(QtCore.QRect(120, 150, 71, 31))
        self.joint_five_value.setObjectName("joint_five_value")
        self.joint_End = QtWidgets.QLabel(Arm)
        self.joint_End.setGeometry(QtCore.QRect(120, 130, 51, 17))
        self.joint_End.setObjectName("joint_End")
        self.joint_Waist = QtWidgets.QLabel(Arm)
        self.joint_Waist.setGeometry(QtCore.QRect(10, 60, 61, 17))
        self.joint_Waist.setObjectName("joint_Waist")
        self.hand_position_label = QtWidgets.QLabel(Arm)
        self.hand_position_label.setGeometry(QtCore.QRect(10, 250, 151, 41))

        self.hand_position_label.setFont(font)
        self.hand_position_label.setObjectName("hand_position_label")
        self.orientation_label = QtWidgets.QLabel(Arm)
        self.orientation_label.setGeometry(QtCore.QRect(10, 360, 151, 41))

        self.orientation_label.setFont(font)
        self.orientation_label.setObjectName("orientation_label")
        self.hand_x = QtWidgets.QLabel(Arm)
        self.hand_x.setGeometry(QtCore.QRect(40, 290, 21, 17))
        font = QtGui.QFont()
        font.setPointSize(14)

        self.joint_Shoulder.setFont(font)
        self.joint_Elbow.setFont(font)
        self.joint_Wrist.setFont(font)
        self.joint_End.setFont(font)
        self.joint_Waist.setFont(font)

        self.hand_x.setFont(font)
        self.hand_x.setObjectName("hand_x")
        self.hand_y = QtWidgets.QLabel(Arm)
        self.hand_y.setGeometry(QtCore.QRect(150, 290, 31, 21))

        self.hand_y.setFont(font)
        self.hand_y.setObjectName("hand_y")
        self.hand_z = QtWidgets.QLabel(Arm)
        self.hand_z.setGeometry(QtCore.QRect(250, 290, 21, 17))

        self.hand_z.setFont(font)
        self.hand_z.setObjectName("hand_z")
        self.orientation_z_val = QtWidgets.QLCDNumber(Arm)
        self.orientation_z_val.setGeometry(QtCore.QRect(230, 420, 71, 31))
        self.orientation_z_val.setObjectName("orientation_z_val")
        self.orientation_y_val = QtWidgets.QLCDNumber(Arm)
        self.orientation_y_val.setGeometry(QtCore.QRect(120, 420, 71, 31))
        self.orientation_y_val.setObjectName("orientation_y_val")
        self.orientation_x_val = QtWidgets.QLCDNumber(Arm)
        self.orientation_x_val.setGeometry(QtCore.QRect(10, 420, 71, 31))
        self.orientation_x_val.setObjectName("orientation_x_val")
        self.hand_z_val = QtWidgets.QLCDNumber(Arm)
        self.hand_z_val.setGeometry(QtCore.QRect(230, 310, 71, 31))
        self.hand_z_val.setObjectName("hand_z_val")
        self.hand_y_val = QtWidgets.QLCDNumber(Arm)
        self.hand_y_val.setGeometry(QtCore.QRect(120, 310, 71, 31))
        self.hand_y_val.setObjectName("hand_y_val")
        self.hand_x_val = QtWidgets.QLCDNumber(Arm)
        self.hand_x_val.setGeometry(QtCore.QRect(10, 310, 71, 31))
        self.hand_x_val.setObjectName("hand_x_val")
        self.orientation_y = QtWidgets.QLabel(Arm)
        self.orientation_y.setGeometry(QtCore.QRect(150, 400, 31, 21))

        self.orientation_y.setFont(font)
        self.orientation_y.setObjectName("orientation_y")
        self.orientation_x = QtWidgets.QLabel(Arm)
        self.orientation_x.setGeometry(QtCore.QRect(40, 400, 21, 17))

        self.orientation_x.setFont(font)
        self.orientation_x.setObjectName("orientation_x")
        self.orientation_z = QtWidgets.QLabel(Arm)
        self.orientation_z.setGeometry(QtCore.QRect(250, 400, 21, 17))

        self.orientation_z.setFont(font)
        self.orientation_z.setObjectName("orientation_z")
        self.error_label = QtWidgets.QTextBrowser(Arm)
        self.error_label.setGeometry(QtCore.QRect(10, 510, 330, 200))
        self.error_label.setObjectName("error_label")
        self.error_message_label = QtWidgets.QLabel(Arm)
        self.error_message_label.setGeometry(QtCore.QRect(10, 460, 151, 41))
        font = QtGui.QFont()
        font.setPointSize(17)
        self.error_message_label.setFont(font)
        self.error_message_label.setObjectName("error_message_label")

        self.retranslateUi(Arm)
        QtCore.QMetaObject.connectSlotsByName(Arm)

    def retranslateUi(self, Arm):
        _translate = QtCore.QCoreApplication.translate
        Arm.setWindowTitle(_translate("Arm", "Form"))
        self.arm_joints.setText(_translate("Arm", "Arm Joints"))
        self.joint_Shoulder.setText(_translate("Arm", "#Shoulder"))
        self.joint_Elbow.setText(_translate("Arm", "#Elbow"))
        self.joint_Wrist.setText(_translate("Arm", "#Wrist"))
        self.joint_End.setText(_translate("Arm", "#End "))
        self.joint_Waist.setText(_translate("Arm", "#Waist"))
        self.hand_position_label.setText(_translate("Arm", "Hand Position"))
        self.orientation_label.setText(_translate("Arm", "Orientation"))
        self.hand_x.setText(_translate("Arm", "X:"))
        self.hand_y.setText(_translate("Arm", "Y:"))
        self.hand_z.setText(_translate("Arm", "Z:"))
        self.orientation_y.setText(_translate("Arm", "Y:"))
        self.orientation_x.setText(_translate("Arm", "X:"))
        self.orientation_z.setText(_translate("Arm", "Z:"))
        self.error_message_label.setText(_translate("Arm", "Error Message"))
