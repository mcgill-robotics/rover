# Mars Rover Project software development

This is a repository containing all the ROS packages needed for the physical rover as well as
the simulations. Each package contains a README file that explains the package architecture as well
as details pertaining to messages, functions, and so on. After you follow this README, make sure to check
the individual package README's for clarification.

# Getting Started
To get started, start by setting up your work environment by following the [Work Environment Setup page](https://github.com/mcgill-robotics/rover/wiki/1.-Work-Environment-Setup) in the Wiki section. Next, complete the [ROS](https://github.com/mcgill-robotics/rover/wiki/2.-ROS-Tutorial) and [Git](https://github.com/mcgill-robotics/rover/wiki/3.-Git-Tutorial) tutorials also found in the Wiki section. 

Finally, clone the repository by running `git clone https://github.com/mcgill-robotics/rover` in a folder called `src` (e.g., create a `catkin_ws` folder in `Documents` and clone the repo in `Documents/catkin_ws/src/`) and build the catkin workspace by running `catkin build` in your catkin workspace.

# Running the System on the Rover Computer (Updated May 19 2024)
Follow these steps to run the Drive, Cameras, and UI systems:
1. ssh to Jetson through McGill wifi or antenna link by running `ssh rover@JETSON_IP` and entering the Jetson User's password (ask a lead for that). The `JESTON_IP` through the antenna link is `192.168.1.69`. The Jetson IP address through McGill wifi is not static, you will need to obtain it by running `ifconfig` on the Jetson directly.
2. Ensure the code version is the one you want to test. To pull latest changes, run `git pull` in the rover folder repo `~/catkin_ws2/src/rover/`.
3. Build the project by running `catkin build` in `~/catkin_ws2/`. This step can be skipped if you already built the project code.
4. To start the Drive, Cameras, and UI systems, `source devel/setup.bash` then run `roslaunch rover_system rover_robot_odrive.launch`. Note that the rover should be picked up at this point so that the ODrives correctly calibrate.
5. Access the UI page at `http://JETSON_IP:8080`.
6. To drive, plug a controller to the computer and press on `On` in the Drive UI widget.

# Running Rover Simulation
- Raw sim: run `roslaunch rover_gazebo rover.launch`.
