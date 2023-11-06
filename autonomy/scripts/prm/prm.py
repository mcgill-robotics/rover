"""Probabilistic Road Map (PRM) Planner
author: Atsushi Sakai (@Atsushi_twi)
Edited by Joseph Saliba, Eric Deng, Helen Deng and Garett"""

import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import KDTree
import matplotlib
# Use the Qt5Agg backend for real-time plotting and to make it visible on my computer
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt
from generate_maps import *

# parameter
N_SAMPLE = 500  # number of sample_points
N_KNN = 10  # number of edge from one sampled point
MAX_EDGE_LEN = 30.0  # [m] Maximum edge length

# Simulated world
CHOSEN_MAP = map1()
# start and goal position
SX = 10.0  # [m]
SY = 10.0  # [m]
GX = 20.0  # [m]
GY = 70.0  # [m]
ROBOT_SIZE = 5.0  # [m]


show_animation = True


class Node:
    """
    Node class for dijkstra search
    """

    def __init__(self, x, y, distanceToGoal, parent_index):
        self.x = x
        self.y = y
        self.distanceToGoal = distanceToGoal
        self.parent_index = parent_index

    def __str__(self):
        return str(self.x) + "," + str(self.y) + "," +\
               str(self.distanceToGoal) + "," + str(self.parent_index)


def pythagorean_distance(s_x, s_y, g_x, g_y):
    '''
    s_x, s_y: the x and y coordinate of the starting point
    g_x, g_y: the x and y coodinate of the end point
    Returns: the hypthenus/ distance between the 2 points
    Computes the pythagorean distance between 2 points'''
    return np.hypot(abs(g_x-s_x), abs(g_y-s_y))


def prm_planning(start_x, start_y, goal_x, goal_y,
                 obstacle_x_list, obstacle_y_list, robot_radius, *, rng=None):
    """
    Run probabilistic road map planning

    :param start_x: start x position
    :param start_y: start y position
    :param goal_x: goal x position
    :param goal_y: goal y position
    :param obstacle_x_list: obstacle x positions
    :param obstacle_y_list: obstacle y positions
    :param robot_radius: robot radius
    :param rng: (Optional) Random generator
    :return:
    """
    # Initialises the obstacles
    obstacle_kd_tree = KDTree(np.vstack((obstacle_x_list, obstacle_y_list)).T)

    # Creates a list of sample points for the prm
    sample_x, sample_y = sample_points(start_x, start_y, goal_x, goal_y,
                                       robot_radius,
                                       obstacle_x_list, obstacle_y_list,
                                       obstacle_kd_tree, rng)
    
    # plots the sample points
    if show_animation:
        plt.plot(sample_x, sample_y, ".b")

    # Generates a road map from the sample points
    # The roadmap is a list, where every index represents a sample point and the content of every
    # index represents the indexes of its neighboring points
    road_map = generate_road_map(sample_x, sample_y,
                                 robot_radius, obstacle_kd_tree)

    rx, ry = greedy_planning(
        start_x, start_y, goal_x, goal_y, road_map, sample_x, sample_y)

    return rx, ry


def is_collision(sx, sy, gx, gy, rr, obstacle_kd_tree):
    x = sx
    y = sy
    dx = gx - sx
    dy = gy - sy
    yaw = math.atan2(gy - sy, gx - sx)
    d = math.hypot(dx, dy)

    if d >= MAX_EDGE_LEN:
        return True

    D = rr
    n_step = round(d / D)

    for i in range(n_step):
        dist, _ = obstacle_kd_tree.query([x, y])
        if dist <= rr:
            return True  # collision
        x += D * math.cos(yaw)
        y += D * math.sin(yaw)

    # goal point check
    dist, _ = obstacle_kd_tree.query([gx, gy])
    if dist <= rr:
        return True  # collision

    return False  # OK


def generate_road_map(sample_x, sample_y, rr, obstacle_kd_tree):
    """
    Road map generation

    sample_x: [m] x positions of sampled points
    sample_y: [m] y positions of sampled points
    robot_radius: Robot Radius[m]
    obstacle_kd_tree: KDTree object of obstacles
    """

    road_map = []
    n_sample = len(sample_x)
    sample_kd_tree = KDTree(np.vstack((sample_x, sample_y)).T)

    for (i, ix, iy) in zip(range(n_sample), sample_x, sample_y):

        dists, indexes = sample_kd_tree.query([ix, iy], k=n_sample)
        edge_id = []

        for ii in range(1, len(indexes)):
            nx = sample_x[indexes[ii]]
            ny = sample_y[indexes[ii]]

            if not is_collision(ix, iy, nx, ny, rr, obstacle_kd_tree):
                edge_id.append(indexes[ii])

            if len(edge_id) >= N_KNN:
                break

        road_map.append(edge_id)

    #  plot_road_map(road_map, sample_x, sample_y)

    return road_map


def greedy_planning(sx, sy, gx, gy, road_map, sample_x, sample_y):
    """
    s_x: start x position [m]
    s_y: start y position [m]
    goal_x: goal x position [m]
    goal_y: goal y position [m]
    obstacle_x_list: x position list of Obstacles [m]
    obstacle_y_list: y position list of Obstacles [m]
    robot_radius: robot radius [m]
    road_map: A list containing lists, where the indexes represent sample points, and the nested lists contains the indexes of neighboring sample points
    sample_x: List containing the x values of every sample point [m]
    sample_y: List containing the y values of every sample point [m]

    @return: Two lists of path coordinates ([x1, x2, ...], [y1, y2, ...]), empty list when no path was found
    """
    # Defining the start points and end points as nodes
    start_node = Node(sx, sy, pythagorean_distance(sx, sy, gx, gy), -1)
    goal_node = Node(gx, gy, 0.0, -1)

    # Creates a dictionary mapping indees to the nodes
    # Open set will contain nodes to be visited, while closed_set's nodes are already visited
    open_set, closed_set = dict(), dict()
    open_set[len(road_map) - 2] = start_node

    path_found = True
    
    # Start of the loop
    while True:
        
        # If no nodes are in the open set, and we haven't arrived at the goal, then it is impossible to 
        # find a path
        if not open_set:
            print("Cannot find path")
            path_found = False
            break
        
        c_id = min(open_set, key=lambda o: open_set[o].distanceToGoal)
        current = open_set[c_id] #current is the node being checked
        
        # shows the on the graph
        if show_animation and len(closed_set.keys()) % 2 == 0:
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            plt.plot(current.x, current.y, "xg")
            plt.pause(0.001)

        # if the index is the same as the goal, we found the goal
        if c_id == (len(road_map) - 1):
            print("goal is found!")
            goal_node.parent_index = current.parent_index
            goal_node.distanceToGoal = current.distanceToGoal
            break

        # Remove the item from the open set
        del open_set[c_id]
        # Add it to the closed set
        closed_set[c_id] = current

        # Checks every point neighboring the current point
        for i in range(len(road_map[c_id])):
            n_id = road_map[c_id][i] # n_id represents the index of the neighbor
            # Sets the distance as the distance to the goal
            distanceToGoal = pythagorean_distance(sample_x[n_id], sample_y[n_id], gx, gy)
            node = Node(sample_x[n_id], sample_y[n_id], distanceToGoal, c_id)
            
            # If already checked, we don't do anything
            if n_id in closed_set:
                continue
            # Otherwise if it is already in the open set, update it
            if n_id in open_set:
                if open_set[n_id].distanceToGoal > node.distanceToGoal:
                    open_set[n_id].distanceToGoal = node.distanceToGoal
                    open_set[n_id].parent_index = c_id
            else:
                # If it isn't in the open set, now we add it
                open_set[n_id] = node

    # returns an empty list if nothing was found
    if path_found is False:
        return [], []

    # generate final course
    rx, ry = [goal_node.x], [goal_node.y]
    parent_index = goal_node.parent_index
    while parent_index != -1:
        n = closed_set[parent_index]
        rx.append(n.x)
        ry.append(n.y)
        parent_index = n.parent_index

    return rx, ry


def plot_road_map(road_map, sample_x, sample_y):  # pragma: no cover

    for i, _ in enumerate(road_map):
        for ii in range(len(road_map[i])):
            ind = road_map[i][ii]

            plt.plot([sample_x[i], sample_x[ind]],
                     [sample_y[i], sample_y[ind]], "-k")


def sample_points(sx, sy, gx, gy, rr, ox, oy, obstacle_kd_tree, rng):
    max_x = max(ox)
    max_y = max(oy)
    min_x = min(ox)
    min_y = min(oy)

    sample_x, sample_y = [], []

    if rng is None:
        rng = np.random.default_rng()

    while len(sample_x) <= N_SAMPLE:
        tx = (rng.random() * (max_x - min_x)) + min_x
        ty = (rng.random() * (max_y - min_y)) + min_y

        dist, index = obstacle_kd_tree.query([tx, ty])

        if dist >= rr:
            sample_x.append(tx)
            sample_y.append(ty)

    sample_x.append(sx)
    sample_y.append(sy)
    sample_x.append(gx)
    sample_y.append(gy)

    return sample_x, sample_y


def main(rng=None):
    print(__file__ + " start!!")
    # ox, oy are the coordinates of the obstacle and map borders
    ox, oy = CHOSEN_MAP[0], CHOSEN_MAP[1]

    if show_animation:
        plt.plot(ox, oy, ".k")
        plt.plot(SX, SY, "^r")
        plt.plot(GX, GY, "^c")
        plt.grid(True)
        plt.axis("equal")

    rx, ry = prm_planning(SX, SY, GX, GY, ox, oy, ROBOT_SIZE, rng=rng)

    assert rx, 'Cannot find path'

    if show_animation:
        plt.plot(rx, ry, "-r")
        plt.pause(0.001)
        plt.show()

if __name__ == '__main__':
    main()

