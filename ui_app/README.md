# Rover UI
A PyQt5 application built to interact with the rover robot.

## Requirements
1. PyQt5
2. pyqt5-tools

### Windows installation
[This](https://www.youtube.com/watch?v=kxSuHyQfStA) video walks through the Windows installation

Qt Designer which is a part of `pyqt5-tools` does not work on M1 macs. If anyone knows a way around this, feel free to contribute to this README.

### Linux installation
1. `$ pip3 install pyqt5`
> Sometimes pip3 won't install pyqt5 properly on Ubuntu so we run the next command to ensure that PyQt5 gets installed properly

2. `$ sudo apt-get install python3-pyqt5`  
3. `$ sudo apt-get install pyqt5-dev-tools`
4. `$ sudo apt-get install qttools5-dev-tools`
5. `$ qtchooser -run-tool=designer -qt=5`

## Running the app
1. Make sure to have PyQt5 installed before trying to run the app.
2. Clone the repository into a directory called `/rover`.
3. `$ cd ~/rover/ui_app/src`
4. On Mac and Linux: `$ python3 main.py` and on Windows: `$ python main.py`

## Understanding the app
The app is designed with PyQt5 along with Qt Designer. Qt designer allows you to easily generate GUIs by providing a drag and drop interface. Once you design your GUI, the GUI is saved as a `.ui` file which can then be converted into Python code by `pyuic5`. Instructions are given below.

## Using Qt Designer
In linux, Qt designer can be run directly from the terminal by `$ designer` if you followed the above instructions. In Windows, you need to find a file called `designer.exe` and run that.

Please put any GUIs you create using Qt Designer into `/ui_app/qt_ui_files`

### Converting .ui into Python code
Run the command `$ pyuic5 -o \<filename>.py \<desginerFile.ui>
