#!/usr/bin/python

"""
Pygame script to test that the algorithm works.
"""

import sys

import pygame
from pygame.locals import *

import gjk

pygame.init()
SCREEN = pygame.display.set_mode((800, 600))
CLOCK = pygame.time.Clock()

BLACK = (  0,   0,   0)
WHITE = (255, 255, 255)
BLUE  = (  0,   0, 255)
GREEN = (  0, 255,   0)
RED   = (255,   0,   0)

def run():
    circle1 = ((400, 300), 100)

    poly1 = (
        ( 00 + 400,  50 + 300),
        (-50 + 400,  50 + 300),
        (-50 + 400,   0 + 300)
    )

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                sys.exit()
            elif event.type == KEYDOWN:
                if event.key == K_ESCAPE:
                    sys.exit()
                elif event.key == K_UP:
                    pass
                elif event.key == K_DOWN:
                    pass
                elif event.key == K_LEFT:
                    pass
                elif event.key == K_RIGHT:
                    pass



        SCREEN.fill(WHITE)
        poly2 = makePolyFromMouse()

        #collide = gjk.collidePolyPoly(poly2, poly1)
        #polygon(poly1)

        collide = gjk.collidePolyCircle(poly2, circle1)
        circle(circle1)

        polygon(poly2, GREEN if collide else RED)
        pygame.display.flip()
        CLOCK.tick(60)

def makePolyFromMouse():
    pos = pygame.mouse.get_pos()

    return (
        ( 50 + pos[0],  50 + pos[1]),
        ( 50 + pos[0], -15 + pos[1]),
        ( 40 + pos[0], -30 + pos[1]),
        ( 20 + pos[0], -50 + pos[1]),
        (  0 + pos[0], -50 + pos[1]),
        (-60 + pos[0],   0 + pos[1])
    )

def pairs(points):
    for i, j in enumerate(range(-1, len(points) - 1)):
        yield (points[i], points[j])

def circles(cs, color=BLACK, camera=(0, 0)):
    for c in cs:
        circle(c, color, camera)

def circle(c, color=BLACK, camera=(0, 0)):
    pygame.draw.circle(SCREEN, color, add(c[0], camera), c[1])

def polygon(points, color=BLACK, camera=(0, 0)):
    for a, b in pairs(points):
        line(a, b, color, camera)

def line(start, end, color=BLACK, camera=(0, 0)):
    pygame.draw.line(SCREEN, color, add(start, camera), add(end, camera))

def add(p1, p2):
    return p1[0] + p2[0], p1[1] + p2[1]

if __name__ == '__main__':
    run()
