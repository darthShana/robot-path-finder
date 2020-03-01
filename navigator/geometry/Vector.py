from numpy import (dot, arccos, clip)
from numpy.linalg import norm


class Vector:

    def __init__(self, p1, p2):
        self.vector = p2.point - p1.point

    def angle_between(self, other):
        c = dot(self.vector,other.vector)/norm(self.vector)/norm(other.vector) # -> cosine of the angle
        return arccos(clip(c, -1, 1)) # if you really want the angle
