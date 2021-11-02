#!/usr/bin/env python

class RoboPoint:
    def __init__(self, x, y, z, w):
        self.x = x
        self.y = y
        self.z = z
        self.w = w

class RoboMove:
    def __init__(self, spoint1, point1, spoint2, point2):
        self.spoint1 = spoint1
        self.point1 = point1
        self.spoint2 = spoint2
        self.point2 = point2
        
