import numpy as np
from math import *

def translatept(points,vec):
    translated = []
    for point in points:
        translated.append((point[0]+vec[0],point[1]+vec[1]))
    return translated

def rotatept(points,angle):
    rotated = []
    for point in points:
        pt = np.matrix([[point[0]], [point[1]]])
        rot = np.matrix([[cos(angle), -sin(angle)], [sin(angle), cos(angle)]])
        mat = rot * pt
        rotated.append((mat.tolist()[0][0],mat.tolist()[1][0]))
    return rotated

def getrectpoints(bounds):
    return ((bounds.left, bounds.top), (bounds.right, bounds.top), (bounds.left, bounds.bottom), (bounds.right, bounds.bottom), bounds.center)

def clamp(n, smallest, largest):
    return max(smallest, min(n, largest))

def distance(p1,p2):
    return sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

def isinside(rect,point):
    a = np.array([rect[0][0], rect[0][1]])
    b = np.array([rect[1][0], rect[1][1]])
    c = np.array([rect[3][0], rect[3][1]])
    m = np.array([point[0], point[1]])
    ab = b - a
    bc = c - b
    am = m - a
    bm = m - b
    return 0 <= np.inner(ab, am) <= np.inner(ab, ab) and \
           0 <= np.inner(bc, bm) <= np.inner(bc, bc)
