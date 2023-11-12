import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN
from scipy.spatial import ConvexHull

# This file contains functions useful functions for simulated map generation and border defining

def generate_points(n, shape='circle', center=(0, 0)):
    if shape == 'circle':
        angles = np.linspace(0, 2*np.pi, n)
        x = center[0] + radius * np.cos(angles)
        y = center[1] + radius * np.sin(angles)
        return np.column_stack((x, y))
    elif shape == 'square':
        side = np.sqrt(n)
        if not side.is_integer():
            raise ValueError('Number of points is not compatible with a square shape.')
        side = int(side)
        x = np.linspace(center[0] - radius, center[0] + radius, side)
        y = np.linspace(center[1] - radius, center[1] + radius, side)
        xx, yy = np.meshgrid(x, y)
        return np.column_stack((xx.flatten(), yy.flatten()))
    else:
        raise ValueError('Unsupported shape.')

def generate_groups(num_groups, num_points, shapes, radii, centers):
    groups = []
    for _ in range(num_groups):
        group = []
        for shape, radius, center in zip(shapes, radii, centers):
            points = generate_points(num_points, shape, center=center, radius=radius)
            group.append(points)
        groups.append(group)
    return groups

def plot_groups(groups):
    colors = ['b', 'g', 'r', 'c', 'm', 'y', 'k']
    for group_idx, group in enumerate(groups):
        for shape_idx, points in enumerate(group):
            color = colors[shape_idx % len(colors)]
            plt.scatter(points[:, 0], points[:, 1], c=color, label=f'Shape {shape_idx+1}')
        plt.title(f'Group {group_idx+1}')
        plt.legend()
        plt.show()
    
def generate_line(start_point,end_point, num_point =80):
    if len(start_point) !=2 or len(end_point) !=2: 
        raise ValueError("need 2d coordinates")
    if num_point < 2:
        raise ValueError("need at least 2 points to create line")
    x = np.linspace(start_point[0], end_point[0],num_point)
    y = np.linspace(start_point[1],end_point[1],num_point)
    return np.column_stack((x, y))

def generate_rectangle(left_top_point,width,length,num_point=80):
    if len(left_top_point) !=2 : 
        raise ValueError("need 2d coordinates")
    rectangle = []
    left_bottom = [left_top_point[0],left_top_point[1]-width]
    right_top = [left_top_point[0]+length,left_top_point[1]]
    right_bottom = [left_top_point[0]+length,left_top_point[1]-width]

    rectangle.append(generate_line(left_top_point,right_top,num_point))
    rectangle.append(generate_line(left_top_point,left_bottom,num_point))
    rectangle.append(generate_line(left_bottom,right_bottom,num_point))
    rectangle.append(generate_line(right_top,right_bottom,num_point))
    
    return np.concatenate(rectangle,axis = 0)



def generate_rectangle_borders(ox: list, oy: list, bottom, top, left, right, precision):
    # the o in ox, oy stands for obstacle
    for i in range(round(abs(right-left))*precision):
        # Add bottom border
        ox.append(i/precision +left)
        oy.append(bottom)
        # Add Top border
        ox.append(i/precision + left)
        oy.append(top)
    for i in range(round(abs(top-bottom))*precision):
        # Add left border
        ox.append(left)
        oy.append(i/precision + bottom)
        # Add right border
        ox.append(right)
        oy.append(i/precision + bottom)