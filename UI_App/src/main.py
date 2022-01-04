# Imports 
from ui_layout import Ui_MainWindow


from PyQt5 import QtWidgets as qtw
from PyQt5 import QtCore as qtc


class UI(qtw.QMainWindow, Ui_MainWindow):
    '''
    Main application interface. It inherits from Ui_MainWindow which is the base layout for the
    app present in ui_layout.py. Most of the app is controlled from this class.
    '''

    def __init__(self, *args, **kwargs):
        # Setup the UI from Ui_MainWindow
        super().__init__(*args, **kwargs)
        self.setupUi(self)     

        # Listeners
        self.control_selector.currentTextChanged.connect(self.on_control_changed)


    def on_control_changed(self, value):
        '''
        Method takes in the UI and the value of the control_selector combo box. It gets 
        called whenever the ComboBox value gets changed. 
        #TODO: Waiting for system controls to be implemented so that this selector can 
        select the control system.
        '''
        
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
    
