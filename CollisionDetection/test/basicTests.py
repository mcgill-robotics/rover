#!/usr/bin/python

import sys

import pygame
from pygame.locals import *

sys.path.insert(0, '..\src')
import gjk

shape1 = [(0,0,1), (0,1,1), (1,1,1), (1,0,1), (0,0,0), (1,0,0), (0,1,0), (1,1,0)]
shape2 = [(0,0,1), (0,1,1), (1,1,1), (1,0,1), (0,0,0), (1,0,0), (0,1,0), (1,1,0)]
shape3 = [(5,5,6), (5,6,6), (6,6,6), (6,5,6), (5,5,5), (6,5,5), (5,6,5), (6,6,5)]


print(gjk.collide(shape1, shape2))
print(gjk.collide(shape1, shape3))