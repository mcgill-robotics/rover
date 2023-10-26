# Rover

Human Control Interface:

Messages:

rover_velocity_controller/cmd_vel: Contains the Twist values that are output by the controller and regulated by the package scripts.

Scripts:

Gamepad.py:

Polls all the controller data and create a Gamepad object that contains all the input data. The Gamepad object maps well to
a dualshock controller.

GamepadProcess.py:

Script that manipulates the Gamepad data and outputs controller data for driving and data for camera control.