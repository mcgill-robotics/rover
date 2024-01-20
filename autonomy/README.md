# Rover Drive autonomy
This package provides a set of scripts and tools for implementing autonomous navigation using Probabilistic Roadmaps (PRM) in robotic systems. The package includes functionalities for generating maps, simulating obstacles, configuring autonomy parameters. Camera mapping. 

# Installation
1. `$pip install -U scikit-learn`
2. `$pip install scipy`

# Scripts
1. dynamic_prm.py
This script implements a dynamic version of the Probabilistic Roadmap (PRM) algorithm, or prm.py. It rerun prm algrithm continously with updating environment information

2. generate_maps.py
This script is responsible for generating maps used in the navigation process. It can create maps based on predefined parameters and configurations.

3. simulate_ob.py
The simulate_ob.py script facilitates the simulation of obstacles in the environment. For testing proposes

4. Adding_Polygons_to_rviz.py
This script is designed to work with RViz, allowing the addition of polygons to the visualization for a more comprehensive understanding of the environment.

5. bounding_boxes.py
This script provides functionality related to bounding boxes, used for obstacle detection and collision avoidance in the autonomy system.

6. mapping.py
Usding bounding_boxes.py, convert camera points clouds to 2d geometry shapes. 

# Visualizing Gazebo environment in rviz
1. `$ cd (workspace)`
2. `$ roslaunch rover_gazebo rover.launch`
Open new terminal
3. `$ python3 mapping.py`
Open new terminal
4. `$ rviz -d rand.rviz`

# Runing prm algorithm 
See how prm work using sample obstacles:
`$ python3 prm.py`

Need running mapping.py to update json file for update environment information for dynamic_prm.py
`python3 dynamic_prm.py`