import numpy as np
from matplotlib import pyplot as plt
from mqtt.ORBMQTTSubscriber import ORBMQTTSubscriber

import threading
import numpy as np

from multiprocessing import Queue
from geometry.Point import Point
from geometry.Vector import Vector
from sklearn.linear_model import LinearRegression
from Robot import Robot


def follow(q, way_points, robot):

    last_orientation = Vector(Point(0, 0), Point(1, 0))
    point_frame = []

    try:
        while True:
            if not q.empty():
                [x0, y0, z0, x_rot, y_rot, z_rot, w_rot] = q.get()

                current_location = Point(x0, z0)

                if len(point_frame) < 20:
                    point_frame.append(current_location)

                while len(point_frame) > 2 and current_location.distance(point_frame[0]) > 0.1:
                    point_frame.pop(0)

                xs = np.array(list(map(lambda p: p.x, point_frame))).reshape((-1, 1))
                ys = np.array(list(map(lambda p: p.y, point_frame)))
                model = LinearRegression().fit(xs, ys)
                y_pred = model.predict(xs)
                current_orientation = Vector(Point(point_frame[0].x, y_pred[0]), Point(point_frame[-1].x, y_pred[-1]))
                distance = point_frame[0].distance(point_frame[-1])

                # plt.quiver(x0, z0, current_orientation.vector[0], current_orientation.vector[1])
                plt.plot(x0, z0, 'bo', markersize=1)
                plt.pause(0.05)

                if len(way_points) == 0:
                    continue

                current_waypoint = Point(way_points[0][0], way_points[0][1])

                if current_location.distance(current_waypoint) < 0.05:
                    way_points = way_points[1:]
                    print('waypoint reached:'+str(current_waypoint))
                    if len(waypoints) == 1:
                        robot.stop()

                angle_change = current_orientation.angle_between(last_orientation)

                if angle_change > np.pi/32:
                    smallest_angle = Vector(current_location, current_waypoint).clockwise_angle_between(current_orientation)
                    angle_between = Vector(current_location, current_waypoint).clockwise_angle_between(current_orientation)
                    print('current heading:'+str(current_orientation) + ' required heading:'+str(Vector(current_location, current_waypoint)))

                    if smallest_angle > np.pi/16:
                        if angle_between > np.pi:
                            robot.right()
                            print('turning right')
                        else:
                            robot.left()
                            print('turning left')
                    else:
                        robot.straight()
                        print('going forward')

                    last_orientation = current_orientation

                if distance < 0.02:
                    print("accelerating")
                    robot.accelerate()
                elif distance > 0.2:
                    print("decelerating")
                    robot.decelerate()

    except KeyboardInterrupt:
        robot.stop()


def listen(q):
    mqttc = ORBMQTTSubscriber(q)
    rc = mqttc.run()
    print("rc: "+str(rc))


waypoints = np.array([
    [0.000531,0.00091],
    [0.073597,0.082323],
    [0.425684,0.695283],
    [0.509028,0.949478],
    [0.528572,1.132364],
    [0.74973,1.814171],
    [0.830563,1.899417],
    [1.273103,2.183242],
    [1.417957,2.220418],
    [1.564619,2.209674],
    [6.620558,0.788293],
    [6.667304,0.687995],
    [6.775818,0.200381],
    [6.750724,0.091334],
    [6.793179,-0.022835],
    [6.895534,-0.899645],
    [6.786471,-0.929014],
    [6.657936,-0.997429],
    [4.544754,-1.629764],
    [4.448957,-1.581997],
    [4.374058,-1.510019],
    [4.324708,-1.42105],
    [0.018419,-0.094408],
    [0.021674,0.027273],
    [0.091588,0.112717],
    [0.382317,0.684154],
    [0.345567,0.786271],
    [0.287494,0.87225],
    [0.252902,0.972536],
    [0.195079,1.059773],
    [0.1621,1.154621],
    [0.080945,1.272655],
    [-0.022766,1.349799],
    [-0.082378,1.439893],
    [-0.191737,1.495533],
    [-0.288089,1.585921]
])

x, y = waypoints.T
fig = plt.figure(figsize=(8, 8))
plt.gca().set_aspect('equal', adjustable='box')
plt.scatter(x, y, marker='+')
plt.pause(0.05)

q = Queue()
x = threading.Thread(target=listen, args=(q,))
x.start()
robot = Robot('http://localhost:5000')

follow(q, waypoints, robot)
