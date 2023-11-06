import os, sys
currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(currentdir)
from simulate_ob import *

# Map Scale
TOP = 100
BOTTOM = 0
LEFT = 0
RIGHT = 100
PRECISION = 100

def map1():
    map1_ox, map1_oy = [], []
    # Adds the borders as obstacles inside ox,oy
    generate_rectangle_borders(map1_ox, map1_oy, BOTTOM, TOP, LEFT, RIGHT, PRECISION)
    num_groups = 2
    num_points = 100
    shapes = ['circle', 'square']
    radii = [15, 10.0]
    centers = [(20, 40), (80, 60)]
    groups = generate_groups(num_groups, num_points, shapes, radii, centers)
    
    for group_idx, group in enumerate(groups):
        for shape_idx, points in enumerate(group):
            for point in points:
                map1_ox.append(point[0])
                map1_oy.append(point[1])
    return map1_ox, map1_oy


def map2():
    map2_ox, map2_oy = [], []
    group = []
    group.append(generate_rectangle([20,80],20,80))
    group.append(generate_rectangle([0,100],100,100))
    group.append(generate_line([20,60],[20,15]))
    group.append(generate_line([40,40],[40,15]))
    group.append(generate_line([60,60],[60,30]))
    group.append(generate_line([80,40],[80,0]))
    group.append(generate_line([40,15],[80,15]))
    in_one = np.concatenate(group,axis = 0)
    map2_ox = in_one[:,0]
    map2_oy = in_one[:,1]
    return map2_ox, map2_oy



    

