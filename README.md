# rover
Mars Rover Project software development

This is a repository containing all the ROS packages needed for the physical rover as well as
the Gazebo simulation. Each package contains a README file that explains the package architecture as well
as details pertaining to messages, functions, and so on. After you follow this README, make sure to check
the individual package README's for clarification.

# Getting Started

### Working with Ubuntu
To use ROS, you will need an Ubuntu system. For Windows users, options include dualbooting, virtual machines
or WSL (Windows Subsystem for Linux). You can also try VPN'ing into McGill EMF (Engineering Microcomputing
Facilities) PC's, but some of the steps will need sudo which might not be allowed.

### Installing ROS
Follow [these steps](http://wiki.ros.org/noetic/Installation/Ubuntu) to install ROS. Next, follow [these steps](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment) to see how to configure your shell and how to make a workspace. If you
want, you can keep going with the tutorials to familiarize yourself with ROS and its commands. In addition, you
should install the packages listed in "requirements.txt".

### Installing Dependencies
The rover uses some packages provided by the larger community. To install them and be able to use them, the following commands need to be run
```
sudo apt-get install ros-noetic-rosserial

```

### Working with Catkin
Catkin is ROS's build system, which traverses every ROS package in a source directory and compiles them into 
ROS modules. Every ROS project has a Catkin workspace structure:
![Catkin workspace](https://miro.medium.com/v2/resize:fit:1400/format:webp/0*vfNM1mbkhUpvK-nW.png)

This repository represents the "src" folder in this file tree. So, create a folder that will be your catkin
workspace and clone this repository inside that folder. (This part might not be necessary, but rename the cloned
folder "src" in case the catkin builder gives errors.)

Navigate to the folder into which you cloned this repo, and run `catkin init` followed by `catkin build`. This should build the modules and
create two more folders: "build" and "devel". At this point, you should source the setup script in the devel folder:

`source devel/setup.bash`

### Starting Nodes
To start the nodes, you should first run `roscore` in a seperate terminal. This is the program that allows all
ROS modules to communicate. Then, you can go back to the first terminal to start whichever node(s) you need.
Navigate to the "rover" folder (or "src" if you renamed it), then use this syntax:

`rosrun [package_name] [node_name]`

Package names are the same as the folders that you see in the repo. You can find the node names in the "src" subfolder
of each package.

Also see the README in the folder "rover_system" to see how to launch all the necessary nodes for different tasks.
