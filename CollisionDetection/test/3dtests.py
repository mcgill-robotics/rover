#!/usr/bin/python

"""
Pygame script to test that the algorithm works.
"""

import gjk
import sys

import pygame
from pygame.locals import *
import pygame.draw
import pygame.time
from math import sin, cos

sys.path.insert(0, '..\src')

ORIGINX = 0
ORIGINY = 0


def draw_3dline(surface, color, a, b):
	"""Convert 3D coordinates to 2D and draw line."""
	ax, ay = a[0]+(a[2]*0.3)+ORIGINX, a[1]+(a[2]*0.3)+ORIGINY
	bx, by = b[0]+(b[2]*0.3)+ORIGINX, b[1]+(b[2]*0.3)+ORIGINY
	pygame.draw.line(surface, color, (ax, ay), (bx, by))


def draw_cube(surface, color, cube):
	"""Draw 3D cube."""
	a, b, c, d, e, f, g, h = cube
	draw_3dline(surface, color, a, b)
	draw_3dline(surface, color, b, c)
	draw_3dline(surface, color, c, d)
	draw_3dline(surface, color, d, a)

	draw_3dline(surface, color, e, f)
	draw_3dline(surface, color, f, g)
	draw_3dline(surface, color, g, h)
	draw_3dline(surface, color, h, e)

	draw_3dline(surface, color, a, e)
	draw_3dline(surface, color, b, f)
	draw_3dline(surface, color, c, g)
	draw_3dline(surface, color, d, h)


def rotate_3dpoint(p, angle, axis):
	"""Rotate a 3D point around given axis."""
	ret = [0, 0, 0]
	cosang = cos(angle)
	sinang = sin(angle)
	ret[0] += (cosang+(1-cosang)*axis[0]*axis[0])*p[0]
	ret[0] += ((1-cosang)*axis[0]*axis[1]-axis[2]*sinang)*p[1]
	ret[0] += ((1-cosang)*axis[0]*axis[2]+axis[1]*sinang)*p[2]
	ret[1] += ((1-cosang)*axis[0]*axis[1]+axis[2]*sinang)*p[0]
	ret[1] += (cosang+(1-cosang)*axis[1]*axis[1])*p[1]
	ret[1] += ((1-cosang)*axis[1]*axis[2]-axis[0]*sinang)*p[2]
	ret[2] += ((1-cosang)*axis[0]*axis[2]-axis[1]*sinang)*p[0]
	ret[2] += ((1-cosang)*axis[1]*axis[2]+axis[0]*sinang)*p[1]
	ret[2] += (cosang+(1-cosang)*axis[2]*axis[2])*p[2]
	return ret


def rotate_object(obj, angle, axis):
	"""Rotate an object around given axis."""
	for i in range(len(obj)):
		obj[i] = rotate_3dpoint(obj[i], angle, axis)


def main():
	global ORIGINX, ORIGINY
	pygame.init()
	screen = pygame.display.set_mode((320, 200))
	# Move origin to center of screen
	ORIGINX = screen.get_width()/2
	ORIGINY = screen.get_height()/2
	cube = [(-50, 50, 50),  (50, 50, 50),  (50, -50, 50),  (-50, -50, 50),
			(-50, 50, -50), (50, 50, -50), (50, -50, -50), (-50, -50, -50)]
	while 1:
		draw_cube(screen, 255, cube)
		event = pygame.event.poll()
		if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
			break
		pygame.display.flip()
		pygame.time.delay(25)
		draw_cube(screen, 0, cube)
		rotate_object(cube, 0.1, (0,1,0))
		rotate_object(cube, 0.01, (0,0,1))
		rotate_object(cube, 0.01, (1,0,0))

if __name__ == "__main__":
	main()
