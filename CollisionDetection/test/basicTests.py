#!/usr/bin/python

import sys
sys.path.insert(0, '../src')
import gjk_dist2 as gjk

#the shapes below are all corner points from cubes or rectangular prisms

shape1 = [(0,0,1), (0,1,1), (1,1,1), (1,0,1), (0,0,0), (1,0,0), (0,1,0), (1,1,0)]
shape2 = [(0,0,1), (0,1,1), (1,1,1), (1,0,1), (0,0,0), (1,0,0), (0,1,0), (1,1,0)]
shape3 = [(5,5,6), (5,6,6), (6,6,6), (6,5,6), (5,5,5), (6,5,5), (5,6,5), (6,6,5)]
shape4 = [(0,0,6), (0,1,6), (1,1,6), (1,0,6), (0,0,5), (1,0,5), (0,1,5), (1,1,5)]
shape5 = [(0,0,2), (0,1,2), (1,1,2), (1,0,2), (0,0,0), (1,0,0), (0,1,0), (1,1,0)]
shape6 = [(0,0,6), (0,1,6), (1,1,6), (1,0,6), (0,0,1), (1,0,1), (0,1,1), (1,1,1)]

shape7 = [(3, 1, 1), (2, 5, -3), (0, 0, 0)]
shape8 = [(3, 3, -3), (1, 2, -2), (1, 3, 1)]

# print(gjk.collide(shape1, shape2)) #should intersect
# print(gjk.collide(shape1, shape3)) #should not intersect
# print(gjk.collide(shape1, shape4)) #should not intersect
print(gjk.collide(shape5, shape6)) #should intersect

# print(gjk.collide(shape7, shape8)) # Should intersect