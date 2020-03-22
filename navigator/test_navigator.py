import unittest
import numpy as np
import math
from geometry.Vector import Vector
from geometry.Point import Point


class TestNavigator(unittest.TestCase):

    def test_vector_angle_between(self):
        p1 = Point(0, 0)
        p2 = Point(1, 0)
        p3 = Point(2, 1)

        v1 = Vector(p1, p2)
        v2 = Vector(p2, p3)

        self.assertAlmostEqual(v1.angle_between(v2), np.pi/4)

    def test_vector_clockwise_angle_between(self):
        p1 = Point(0, 0)
        p2 = Point(1, 0)
        p3 = Point(2, 1)
        p4 = Point(2, -1)

        v1 = Vector(p1, p2)
        v2 = Vector(p2, p3)
        v3 = Vector(p2, p4)

        self.assertAlmostEqual(v1.clockwise_angle_between(v2), 2 * np.pi - np.pi/4)
        self.assertAlmostEqual(v1.clockwise_angle_between(v3), np.pi/4)

    def test_vector_from_angle(self):
        p1 = Point(0, 0)
        p2 = Point(np.cos(np.pi/4), np.sin(np.pi/4))
        current_orientation = Vector(p1, p2)

        self.assertAlmostEqual(current_orientation.vector[0], math.sqrt(.5))
        self.assertAlmostEqual(current_orientation.vector[1], math.sqrt(.5))


if __name__ == '__main__':
    unittest.main()
