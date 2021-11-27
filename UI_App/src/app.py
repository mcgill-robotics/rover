import PyQt5.QtWidgets as qtw


class MainWindow(qtw.QWidget):
    def __init__(self):
        super().__init__()

        # Title
        self.setWindowTitle("Rover UI")

        # Layout
        
        self.show()


app = qtw.QApplication([])
win = MainWindow()

app.exec_()