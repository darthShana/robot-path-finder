from numpy import (dot, arccos, clip, arctan2, pi, linalg)
from numpy.linalg import norm


class Vector:

    def __init__(self, p1, p2):
        self.vector = p2.point - p1.point

    def angle_between(self, other):
        c = dot(self.vector,other.vector)/norm(self.vector)/norm(other.vector) # -> cosine of the angle
        return arccos(clip(c, -1, 1)) # if you really want the angle

    def clockwise_angle_between(self, other):
        ang1 = arctan2(*self.vector[::-1])
        ang2 = arctan2(*other.vector[::-1])
        return (ang1 - ang2) % (2 * pi)

    def normalise(self):
        return self.vector / linalg.norm(self.vector)

    def __repr__(self):
        return str(self.vector)
