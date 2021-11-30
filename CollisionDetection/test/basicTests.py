#!/usr/bin/python

import sys

import pygame
from pygame.locals import *

sys.path.insert(0, '..\src')
import gjk_2

shape1 = [(0,0,1), (0,1,1), (1,1,1), (1,0,1), (0,0,0), (1,0,0), (0,1,0), (1,1,0)]
shape2 = [(0,0,1), (0,1,1), (1,1,1), (1,0,1), (0,0,0), (1,0,0), (0,1,0), (1,1,0)]
shape3 = [(5,5,6), (5,6,6), (6,6,6), (6,5,6), (5,5,5), (6,5,5), (5,6,5), (6,6,5)]
shape4 = [(0,0,6), (0,1,6), (1,1,6), (1,0,6), (0,0,5), (1,0,5), (0,1,5), (1,1,5)]


print(gjk_2.collide(shape1, shape2)) #should intersect
print(gjk_2.collide(shape1, shape3)) #should not intersect
print(gjk_2.collide(shape1, shape4)) #should not intersect