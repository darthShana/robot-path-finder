from matplotlib import pyplot as plt
import numpy as np
from geometry.Point import Point
from geometry.Vector import Vector

waypoints = np.array([
    [0.339158,0.706729],
    [0.573945,0.966652]
])

x, y = waypoints.T
fig = plt.figure(figsize=(8, 8))
# plt.gca().set_aspect('equal', adjustable='box')
# plt.scatter(x, y, marker='+')
# plt.pause(0.05)

real_set = np.array([
    [0.260604, 0.60451, 0.002772, 0.00128472, 0.078554, 0.102219],
    [0.260189, 0.604029, 0.000821, 0.00069812, 0.078969, 0.1027],
    [0.268817, 0.619011, 0.010145, 0.0165638, 0.070341, 0.087718],
    [0.277605, 0.666931, 0.017001, 0.0531772, 0.061553, 0.039798],
    [0.344682, 0.797106, 0.073879, 0.16858589, 0.229263, 0.169546],
    [0.380564, 0.88632, 0.033792, 0.05841867, 0.193381, 0.080332],
    [0.380813, 0.886626, 0.015255, 0.03503874, 0.193132, 0.080026],
    [0.375025, 0.883391, -0.000219, -0.00011816, 0.19892, 0.083261],
    [0.38172,0.891958, 0.000907, 0.0008627, 0.192225, 0.074694],
    [0.400701,0.896105, 0.025676, 0.01378414, 0.173244, 0.070547],
    [0.478786,0.956886, 0.090578, 0.06632362, 0.095159, 0.009766],
    [0.512526,0.997322, 0.111825, 0.10127851, 0.061419, -0.03067],
    [0.567643,1.052452, 0.027861, 0.03644039, 0.006302, -0.0858],
    [0.567222,1.052034, 0.00297, 0.01197254, 0.006723, -0.085382],
    [0.566457,1.053624, 0.000733, 0.0009549, 0.007488, -0.086972],
    [0.565917,1.051455, -0.000458, -0.00026751, 0.008028, -0.084803],
    [0.567117,1.055854, -0.000962, -0.00092273, 0.006828, -0.089202],
    [0.570079,1.052827, 0.002436, 0.00010947, 0.003866, -0.086175],
    [0.574579,1.062388, 0.008662, 0.0084443, -0.000634, -0.095736]
])

v1 = Vector(Point(0,0), Point(0.002772, 0.00128472)).normalise()
v2 = Vector(Point(0,0), Point(0.078554, 0.102219)).normalise()
print(v1)
print(v2)

x, y, hx, hy, rx, ry = real_set.T
plt.quiver(x, y, hx, hy, color='b')
# plt.quiver(x, y, rx, ry, color='r')
plt.pause(0.05)

# plt.plot(0, 0, 'bo')
# plt.plot(v1[0], v1[1], 'ro')
# plt.plot(v2[0], v2[1], 'ro')
# plt.plot(0.002772, 0.00128472, 'bo')
# plt.plot(0.078554, 0.102219, 'bo')
# plt.pause(0.05)

input("Press Enter to continue...")
