# Rover

Drive Control Package Info:

Messages manipulated:
/wheel_velocity_cmd - Message from user's end. Wheel velocities from drive_control_node steering calculations.
/rover_velocity_controller/cmd_vel - Twist velocities from gamepad inputs.
/feedback_velocity - Rover computer feedback velocities from firmware.

/wheel_velocity_cmd: The velocities that come out from the drive_control_node script and calculated by steering. Units: rad/s

/rover_velocity_controller/cmd_vel: Contains the Twist messages (SI Units). Units: rad/s

Scripts:

drive_control_node.py: Manages the velocities taken from the controller and then converts them into wheel velocities that the rover must be travelling with. Acceleration is created by convolving a gaussian kernel with arrays of wheel velocities.

steering.py: Implements a differential drive control for rover steering. This is the math behind converting twist velocities to the wheel
velocities. Read more in this link: http://wiki.ros.org/diff_drive_controller