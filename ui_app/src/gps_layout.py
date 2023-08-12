# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'gps_layout.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.pyplot as plt
import math

class LocationComponent(QtWidgets.QWidget):
    def __init__(self, pin_number, longitude, latitude, parent=None):
        super().__init__(parent)
        
        self.pin_number = pin_number
        self.longitude = longitude
        self.latitude = latitude

        self.pin_label = QtWidgets.QLabel(f"Pin {pin_number}")
        self.longitude_label = QtWidgets.QLabel(f"Longitude: {self.longitude}")
        self.latitude_label = QtWidgets.QLabel(f"Latitude: {self.latitude}")
        self.distance_label = QtWidgets.QLabel(f"Distance: Unknown")
        self.delete_button = QtWidgets.QPushButton("Delete")
        self.delete_button.clicked.connect(self.delete_location_component)
        
        layout = QtWidgets.QVBoxLayout()
        layout.addWidget(self.pin_label)
        layout.addWidget(self.longitude_label)
        layout.addWidget(self.latitude_label)
        layout.addWidget(self.distance_label)
        layout.addWidget(self.delete_button)

        container_widget = QtWidgets.QWidget()
        container_widget.setLayout(layout)
        
        main_layout = QtWidgets.QVBoxLayout()
        main_layout.addWidget(container_widget)
        
        self.setStyleSheet("border: 1px solid black;")
        self.setLayout(main_layout)


    def delete_location_component(self):
        self.setParent(None)
        self.deleteLater()


class Ui_GPS(QtWidgets.QWidget):
    def setupUi(self, GPS):
        GPS.setObjectName("GPS")
        GPS.resize(506, 648)
        self.verticalLayoutWidget = QtWidgets.QWidget(GPS)
        self.verticalLayoutWidget.setGeometry(QtCore.QRect(10, 20, 481, 800))
        self.verticalLayoutWidget.setObjectName("verticalLayoutWidget")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.verticalLayoutWidget)
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout.setObjectName("verticalLayout")
        self.GPS_Plot = QtWidgets.QLabel(self.verticalLayoutWidget)
        font = QtGui.QFont()
        font.setPointSize(17)
        self.GPS_Plot.setFont(font)
        self.GPS_Plot.setObjectName("GPS_Plot")
        self.verticalLayout.addWidget(self.GPS_Plot)

        self.GPS_data_lable = QtWidgets.QLabel(GPS)
        self.GPS_data_lable.setGeometry(QtCore.QRect(10, 580, 479, 18))
        self.GPS_data_lable.setObjectName("GPS_data_lable")
        self.gps_data = QtWidgets.QLabel(GPS)
        self.gps_data.setGeometry(QtCore.QRect(10, 630, 479, 37))
        self.gps_data.setObjectName("gps_data")
        self.gps_data.setFont(font)
        self.verticalLayout.addWidget(self.gps_data)
        self.gps_fig = plt.figure()
        self.gps_canvas = FigureCanvas(self.gps_fig)
        self.gps_plotter = self.gps_canvas
        #self.gps_plotter = QtWidgets.QWidget(self.verticalLayoutWidget)
        self.verticalLayout.addWidget(self.gps_plotter)
        self.gps_plotter.setMinimumSize(QtCore.QSize(0, 400))
        self.gps_plotter.setStyleSheet("background-color: rgb(0, 0, 0);")
        self.gps_plotter.setObjectName("gps_plotter")
        self.verticalLayout.addWidget(self.gps_plotter)
        self.addButton = QtWidgets.QPushButton("Drop pin")
        self.addButton.clicked.connect(self.add_location_component)
        self.verticalLayout.addWidget(self.addButton)

        self.scroll_area = QtWidgets.QScrollArea()
        self.scroll_widget = QtWidgets.QWidget()  # Create a widget to hold the components
        self.scroll_layout = QtWidgets.QVBoxLayout(self.scroll_widget)
        self.scroll_layout.addStretch()  # Add a stretch to push components to the top

        self.scroll_area.setWidgetResizable(True)  # Allow scroll area to resize its widget
        self.scroll_area.setWidget(self.scroll_widget)
        
        self.verticalLayout.addWidget(self.scroll_area)

        self.last_gps_data = [0, 0]
        self.location_components = []

        self.retranslateUi(GPS)
        QtCore.QMetaObject.connectSlotsByName(GPS)


    def add_location_component(self):
        pin = len(self.location_components) + 1
        distance = 10.5
        component = LocationComponent(pin, self.last_gps_data[1], self.last_gps_data[0])
        self.location_components.append(component)
        self.scroll_layout.addWidget(component)


    def update_gps_data(self, gps_data):
        self.gps_data.setText("latitude: " + str(gps_data.data[0])+ "\nlongitude: " + str(gps_data.data[1]))
        self.last_gps_data = gps_data.data
        
        for component in self.location_components:
            # compute distance using haversine formula
            # https://stackoverflow.com/questions/4913349/haversine-formula-in-python-bearing-and-distance-between-two-gps-points
            lat1 = gps_data[0]
            lon1 = gps_data[1]
            lat2 = component.latitude
            lon2 = component.longitude

            R = 6371e3 # metres

            phi1 = lat1 * math.pi/180 # phi, lambda in radians
            phi2 = lat2 * math.pi/180

            delta_phi = (lat2-lat1) * math.pi/180
            delta_lambda = (lon2-lon1) * math.pi/180

            component.distance_label.setText(f"Distance: {round(2*R*math.asin(math.sqrt(math.sin(delta_phi/2)*math.sin(delta_phi/2) + math.cos(phi1)*math.cos(phi2)*math.sin(delta_lambda/2)*math.sin(delta_lambda/2))))}m")
        

    def retranslateUi(self, GPS):
        _translate = QtCore.QCoreApplication.translate
        GPS.setWindowTitle(_translate("GPS", "GPS"))
        self.GPS_Plot.setText(_translate("GPS", "GPS_Plot"))