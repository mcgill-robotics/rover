from simulate_ob import *
import json

# Map Scale
TOP = 40
BOTTOM = -40
LEFT = -40
RIGHT = 40
PRECISION = 1

def test_map():
    ox, oy = [], []
    data = json.load(open('autonomy/scripts/prm/obstacles.json'))
    obstacles = data["map"]
    for key in obstacles.keys():
        point = key.split()
        ox.append(float(point[0]))
        oy.append(float(point[1]))
    generate_rectangle_borders(ox, oy, BOTTOM, TOP, LEFT, RIGHT)
    
    return ox, oy

def map1():
    map1_ox, map1_oy = [], []
    # Adds the borders as obstacles inside ox,oy
    generate_rectangle_borders(map1_ox, map1_oy, BOTTOM, TOP, LEFT, RIGHT)
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

