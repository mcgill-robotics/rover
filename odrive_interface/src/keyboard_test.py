import sys
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtCore import QThread, pyqtSignal
from pynput.keyboard import Listener, Key

class KeyboardListenerThread(QThread):
    keyPressed = pyqtSignal(str)  # Signal to communicate key presses

    def run(self):
        # Define the on_press callback for pynput
        def on_press(key):
            try:
                self.keyPressed.emit(key.char)  # Emit signal with the character of the key pressed
            except AttributeError:
                if key == Key.space:  # Example to handle special keys
                    self.keyPressed.emit(' ')

        # Start the pynput listener
        with Listener(on_press=on_press) as listener:
            listener.join()

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Qt App with Pynput")
        self.resize(300, 200)

        # Set up the keyboard listener thread
        self.keyboardListenerThread = KeyboardListenerThread()
        self.keyboardListenerThread.keyPressed.connect(self.on_key_press)
        self.keyboardListenerThread.start()

    def on_key_press(self, key):
        print(f"Key pressed: {key}")
        # Here you can update the UI or handle the key press event

if __name__ == "__main__":
    app = QApplication(sys.argv)
    mainWindow = MainWindow()
    mainWindow.show()
    sys.exit(app.exec_())
