# Drive Control Package Info

## Messages manipulated
/wheel_velocity_cmd - Message from user's end. Wheel velocities from drive_control_node steering calculations. Units: rad/s
/rover_velocity_controller/cmd_vel - Twist velocities from gamepad inputs. Units: rad/s

/feedback_velocity - Rover computer feedback velocities from firmware.

## Scripts:
drive_control_node.py: Manages the velocities taken from the controller and then converts them into wheel velocities that the rover must be travelling with. Acceleration is created by convolving a gaussian kernel with arrays of wheel velocities.

steering.py: Implements a differential drive control for rover steering. This is the math behind converting twist velocities to the wheel
velocities.

## Steering
The controller takes as input the X component of linear twist velocity and the Z component of angular twist velocity.
All other velocities are ignored.
The hardware uses a velocity controller, hence why steering works with a differential drive controller that outputs velocities.
Read more in this link: http://wiki.ros.org/diff_drive_controller
