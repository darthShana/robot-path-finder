import numpy as np
from matplotlib import pyplot as plt
from mqtt.ORBMQTTSubscriber import ORBMQTTSubscriber

import threading
import numpy as np

from multiprocessing import Queue
from geometry.Point import Point
from geometry.Vector import Vector
from Robot import Robot


def follow(q, way_points):

    last_orientation = Vector(Point(0, 0), Point(1, 0))

    while True:
        if not q.empty():
            [x0, y0, y_rot] = q.get()
            print(str(x0)+","+str(y0)+","+str(y_rot))
            current_location = Point(x0, y0)

            convert_to_radians = y_rot * np.pi
            current_orientation = Vector(Point(0, 0), Point(np.sin(convert_to_radians), np.cos(convert_to_radians)))

            plt.quiver(x0, y0, current_orientation.vector[0], current_orientation.vector[1])
            plt.pause(0.05)

            if len(way_points) == 0:
                continue

            current_waypoint = Point(way_points[0][0], way_points[0][1])

            if current_location.distance(current_waypoint) < 0.05:
                way_points = way_points[1:]
                print('waypoint reached:'+str(current_waypoint))

            angle_change = current_orientation.angle_between(last_orientation)

            if angle_change > np.pi/32:

                smallest_angle = Vector(current_location, current_waypoint).clockwise_angle_between(current_orientation)
                angle_between = Vector(current_location, current_waypoint).clockwise_angle_between(current_orientation)
                print('current heading:'+str(current_orientation) + ' required heading:'+str(Vector(current_location, current_waypoint)))
                if smallest_angle > np.pi/16:
                    if angle_between > np.pi:
                        # robot.right()
                        print('turning right')
                    else:
                        # robot.left()
                        print('turning left')
                else:
                    # robot.forward()
                    print('going forward')

                last_orientation = current_orientation


def listen(q):
    mqttc = ORBMQTTSubscriber(q)
    rc = mqttc.run()
    print("rc: "+str(rc))


waypoints = np.array([
    [1.180498,1.106563],
    [1.32077,1.076823],
    [1.413599,1.026179],
    [1.516361,1.018673],
    [1.609157,0.962646],
    [1.751717,0.818076],
    [1.768946,0.714553],
    [1.758828,0.589682],
    [1.678605,0.476262],
    [1.725517,0.315646],
    [1.736003,0.154237],
    [1.673153,0.071388],
    [0.894091,-0.611463],
    [0.774792,-0.631557],
    [0.671396,-0.623666],
    [0.521758,-0.581395],
    [0.474998,-0.466817],
    [0.394313,-0.372097],
    [0.236235,-0.25272],
    [-0.004113,0.01808],
    [-0.047547,0.137185],
    [-0.102458,0.498536],
    [-0.072777,0.625958],
    [-0.021243,0.729773],
    [-0.047914,0.009354]
])
x, y = waypoints.T
fig = plt.figure(figsize=(8, 8))
plt.gca().set_aspect('equal', adjustable='box')
plt.scatter(x, y, marker='+')
plt.pause(0.05)

q = Queue()
x = threading.Thread(target=listen, args=(q,))
x.start()
# robot = Robot('http://localhost:5000')

follow(q, waypoints)
