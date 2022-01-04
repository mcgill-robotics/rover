from ui_layout import Ui_MainWindow


from PyQt5 import QtWidgets as qtw
from PyQt5 import QtCore as qtc


class UI(qtw.QMainWindow, Ui_MainWindow):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.setupUi(self)
        self.control_selector.currentTextChanged.connect(self.on_control_changed)


    def on_control_changed(self, value):
        if value == "Arm-Cartesian Control":
            pass
            # return arm file
        elif value == "Arm-Joint Control":
            pass
            # Return arm file
        elif value == "Science":
            pass
            # Return science file
        elif value == "Drive":
            pass
            # Return drive file
        else:
            pass
            # Return self for autonomy
        

def main():
    app = qtw.QApplication([])

    window = UI()
    window.show()

    app.exec()


if __name__ == '__main__':
    main()
    
