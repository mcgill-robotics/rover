# Rover UI
A PyQt5 application built to interact with the rover robot.

## Requirements
1. PyQt5
2. pyqt5-tools

#### Windows installation
[This](https://www.youtube.com/watch?v=kxSuHyQfStA) video walks through the Windows installation

Qt Designer which is a part of `pyqt5-tools` does not work on M1 macs. If anyone knows a way around this, feel free to contribute to this README.

#### Linux instalation
1. `pip3 install pyqt5`
> Sometimes pip3 won't install pyqt5 properly on Ubuntu so we run the next command to ensure that PyQt5 gets installed properly


## Running the app
1. Make sure to have PyQt5 installed before trying to run the app.
2. Clone the repository into a directory called `/rover`.
3. `cd ~/rover/UI_App/src`
4. On Mac and Linux: `python3 main.py` and on Windows: `python main.py`

## Understanding the app
The app is designed with PyQt5 along with Qt Designer. Qt designer allows you to easily generate GUIs by providing a drag and drop interface. Once you design your GUI, the GUI is saved as a `.ui` file which can then be converted into Python code by `pyuic5`. Instructions are given below.

## 