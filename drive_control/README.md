# Rover

Drive Control Package Info:

Messages manipulated:
/wheel_velocity_cmd
/rover_velocity_controller/cmd_vel
/feedback_velocity

/wheel_velocity_cmd: The velocities that come out from the drive_control_node script and calculated by steering.

/rover_velocity_controller/cmd_vel: Contains the Twist messages (SI Units)

/feedback_velocity: No idea what this is about.

drive_control_node.py: Manages the velocities taken from the controller and then converts them into velocities that the rover must be travelling with.

steering.py: Implements the math behind the driving of the rover.