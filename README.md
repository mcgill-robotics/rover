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

### Working on Mac
It is also possible to install ROS with MacOS. To do so, one needs to use [RoboStack](https://robostack.github.io).
RoboStack uses `conda`/`mamba` environments, and everything needed to get started is available directly on
[RoboStack's documentation](https://robostack.github.io/GettingStarted.html).

### Working with Docker
An experimental alternative to work with Windows, or an alternative to RoboStack on MacOS is to use
[Docker](https://www.docker.com/). A `Dockerfile` is available in the root of this directly, making it possible
to directly build a ready-to-go docker image.

Once docker is installed, run the following command to build the docker image:
```bash
docker build -t ros:mgr-rover .
```

This command builds the image and names it as `ros:mgr-rover`.

Then, run the docker container in interactive mode :
```bash
docker run -itp 9090:9090 -v ./:/root/src ros:mgr-rover
```

- `-it` runs the container in interactive mode.
- `-p 9090:9090` maps the port 9090 of the container to the host computer's 9090 port (for `rosbridge`).
- `-v ./:/root/src` creates a bind mount in order to access the source code from the container.
    __Important:__ if your `cwd` is not the root of this repository, you must change `./` (before `:`) to the
    path leading to this repository.

Now, running `catkin init` and `catkin build` inside the `root` will build the workspace.

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
