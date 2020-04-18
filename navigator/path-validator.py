from matplotlib import pyplot as plt
import numpy as np

waypoints = np.array([
    [-0.000747,-0.00075],
    [0.032325,0.095675],
    [0.109664,0.226586],
    [0.183259,0.441623]
])

x, y = waypoints.T
fig = plt.figure(figsize=(8, 8))
plt.gca().set_aspect('equal', adjustable='box')
plt.scatter(x, y, marker='+')
plt.pause(0.05)

real_set = np.array([
    [0.145072, 0.219581, -0.00018, -0.00025636, 0.038187, 0.222042],
    [0.147224, 0.22102, 0.002858, 0.00268438, 0.036035, 0.220603],
    [0.148674, 0.22601, 0.003997, 0.00618526, 0.034585, 0.215613],
    [0.194201, 0.321966, 0.015964, 0.04370902, -0.010942, 0.119657],
    [0.242273, 0.393112, 0.048723, 0.0754596, -0.059014, 0.048511],
    [0.238168, 0.456439, -0.004105, 0.00267884, -0.054909, -0.014816],
    [0.246529, 0.463364, 0.010023, 0.01743001, -0.06327, -0.021741]
])

x, y, hx, hy, rx, ry = real_set.T
plt.quiver(x, y, hx, hy, color='b')
plt.quiver(x, y, rx, ry, color='r')
plt.pause(0.05)

input("Press Enter to continue...")
