## Sensors ## 
**(List of sensors with links and prices --> https://docs.google.com/document/d/1mytz9acx1puyt1tQ0szOr_ofm-9RC6v6nYdhvXi_OuU/edit)**
* Intel Realsense2 T265 Tracking Camera
  * __Data provided__: x, y, z relative to start + roll, pitch and yaw
  * Used to obtain the x, y and yaw values relative to the start position of the camera; localization node subscribes to camera/odom/sample for info.
  * USB plug-in
  
* UM7 IMU
  * __Data provided__: Binary data that is parsed using this library; https://github.com/mikehoyer/UM7-Arduino
  * Used to obtain the current yaw of the rover.
  * Plug in and set up Arduino with the IMU;
  * Packet converted and sent to Arduino serial monitor, array containing roll, pitch and yaw then published to ROS

* BU-353S4 GPS
  * __Data provided__: GPS data in NMEA formats (GGA, GSA, GSV, and RMC)
  * Used for x and y values of the rover.
  * Plug in then read from /dev/ttyUSB0 port (all done in GPS' serial node)


## Files
*	Final_RealSensorEKF.py: 
	* Contains an Extended Kalman Filter (EKF), subscribes to various topics for measurement data
	* Also contains a simple marker simulation for tracking position of the "rover" in RVIZ.
*	gpsSerialNode.py: 
	* GPS serial node provided by https://www.egr.msu.edu/classes/ece480/capstone/spring15/group14/uploads/4/2/0/3/42036453/wilsonappnote.pdf;
	* publishes topic _bu353_data_ --> NMEA sentence as a String which is then parsed in above file using pynmea2;
*	imuSerialNode.py: 
	* Contains a serial ROS node made by Alex
	* publishes topic _um7_data_ --> IMU data as an array after reading String from Arduino's serial monitor
*	**ar_detector.py**
 
