# Rover System Launcher
This package contains the launch files for easily starting the rover systems desired. The table below contains the launch files available with the corresponding systems attached to them.

To run any of the following system launches, just run the following:
```
$ roslaunch rover_system/launch/{launch_file_name}.launch
```

## Available Launch files
| Launch File | Description |
|---|---|
| rover_robot.launch  |  Launches all the necessary nodes for operating the physical rover. |
| pilot_system.launch | Launches all the nodes and systems for the pilot to run the user interface and connect to the physical rover system. |
| rover_sim_manual.launch | Launches the gazebo simulation of the rover with the driving capabilities for a pilot to control the robot.|
| arm_sim_manual.launch | Launches the pybullet simulation of the rover's arm with the controller capabilities for a pilot to operate the system.|
