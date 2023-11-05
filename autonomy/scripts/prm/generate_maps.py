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

