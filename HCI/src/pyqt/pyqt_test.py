import rospy
from std_msgs.msg import String
import sys
from PyQt5 import QtWidgets
from PyQt5 import QtCore


class MainLauncher(QtWidgets.QWidget):

    def __init__(self):
        QtWidgets.QWidget.__init__(self)
        mainLayout = QtWidgets.QGridLayout()
        self.label = QtWidgets.QLabel()
        mainLayout.addWidget(self.label, 1, 1)
        self.setLayout(mainLayout)

    def display_data(self, data):
        self.label.setText(data)
        



def chatter_callback(message, launcher):
    #rospy.loginfo("I heard ", message.data)
    MainLauncher.display_data(launcher, message.data)


def program_launcher():
    app = QtWidgets.QApplication(sys.argv)
    mainLauncher = MainLauncher()
    rospy.init_node('displayer', anonymous=True)
    rospy.Subscriber("chatter", String, chatter_callback, mainLauncher)
    mainLauncher.show()
    sys.exit(app.exec_())



if __name__ == '__main__':
    try:
        program_launcher()
    except rospy.ROSInterruptException:
        pass