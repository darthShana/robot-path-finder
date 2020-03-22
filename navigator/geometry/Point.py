import numpy


class Point:

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.point = numpy.array([x, y])

    def distance(self, p):
        return numpy.linalg.norm(self.point-p.point)

    def __repr__(self):
        return str(self.x)+","+str(self.y)

