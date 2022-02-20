## Sensors ## 
**([List of sensors with links and prices](https://docs.google.com/document/d/1mytz9acx1puyt1tQ0szOr_ofm-9RC6v6nYdhvXi_OuU/edit))**
* Intel Realsense2 T265 Tracking Camera
  * __Data provided__: x, y, z relative to start + roll, pitch and yaw
  * Used to obtain the x, y and yaw values relative to the start position of the camera; localization node subscribes to camera/odom/sample for info.
  * USB plug-in
  * ![Coordinate System](https://user-images.githubusercontent.com/6543766/64158631-20303a00-ce39-11e9-9c63-fe0baa8135ff.png)
  
* UM7 IMU
  * __Data provided__: Binary data that is parsed using this [library](https://github.com/mikehoyer/UM7-Arduino)
  * Used to obtain the current yaw of the rover.
  * Plug in and set up Arduino with the IMU;
  * Packet converted and sent to Arduino serial monitor, array containing roll, pitch and yaw then published to ROS
  * Connections: Black => Ground, Red => 5V, Orange => 3.3V, Yellow => TX, White => RX

* BU-353S4 GPS
  * __Data provided__: GPS data in NMEA formats (GGA, GSA, GSV, and RMC)
  * Used for x and y values of the rover.
  * Plug in then read from /dev/ttyUSB0 port (all done in GPS' serial node)


## Files
Needed files are all found in RealSensor with the exceptions of ar_detector.py which is in the package artag_detection and rs_t265.launch which is in the realsense2_camera package provided by [IntelRealSense](https://github.com/IntelRealSense/realsense-ros).
* Final_RealSensorEKF.py: 
  * Contains an Extended Kalman Filter (EKF), subscribes to various topics for measurement data
  * Also contains a simple marker simulation for tracking position of the "rover" and tag** in RVIZ.
* gpsSerialNode.py: 
  * GPS serial node provided [here](https://www.egr.msu.edu/classes/ece480/capstone/spring15/group14/uploads/4/2/0/3/42036453/wilsonappnote.pdf);
  * publishes topic _bu353_data_ --> NMEA sentence as a String which is then parsed in above file using pynmea2;
* imuSerialNode.py: 
  * Contains a serial ROS node made by Alex
  * publishes topic _um7_data_ --> IMU data as an array after reading String from Arduino's serial monitor
* **ar_detector.py***: 
  * May need to be changed depending on the competition we go to; currently tracks distance in x y and z between webcam and ARUCO tag.
  * **Note:** Calibrate the camera used beforehand for best readings. This can be done following this [tutorial](https://docs.opencv.org/4.5.2/da/d13/tutorial_aruco_calibration.html)
  * OpenCV Aruco Marker Detection [documentation](https://docs.opencv.org/4.5.2/d5/dae/tutorial_aruco_detection.html)

## How to run
After setting up sensors and cameras, simply run gpsSerialNode.py, imuSerialNode.py and launch rs_t265.launch using this command: roslaunch realsense2_camera rs_t265.launch. Finally, run Final_RealSensorEKF.py; can verify that it works properly by seeing output in the terminal or viewing the RVIZ sim.

## Design Decisions

### States 

The states we estimate are  state_pred = [x, y, yaw]
![/states.png]

### Choice of Covariance matrix values 

The diagonal values of the covariance matrices R represent the variance (std deviation^2) on the measurements. These values are available on the datasheets. 
-GPS : "accuracy" < 2.5m
-IMU (gyro yaw only): RMS noise = 0.06 deg/s-rms. Assuming the EKF runs at 1Hz. Then, RMS noise = 0.06deg-rms. 
-RealSense tracking camera: The datasheet provides no hard value. 

The full list of sensors is here : https://docs.google.com/document/d/1HPagB5k85hzWiUOU6I13NK1elHuTwbgmrrv7OT09EX0/edit. 
Note, however, that not every sensor is used. 
