import numpy as np
from matplotlib import pyplot as plt
from mqtt.ORBMQTTSubscriber import ORBMQTTSubscriber

import threading
import numpy as np

import queue
import time
from geometry.Point import Point
from geometry.Vector import Vector
from sklearn.linear_model import LinearRegression
from Robot import Robot


def follow(q, way_points, robot):

    point_frame = []
    last_location = Point(0, -0.1)
    last_location_time = time.time()

    try:
        while True:
            if not q.empty():
                [x0, y0, z0, x_rot, y_rot, z_rot, w_rot] = q.get()

                with q.mutex:
                    q.queue.clear()

                current_location = Point(x0, z0)
                point_frame.append(current_location)

                if len(point_frame) > 6:
                    point_frame = point_frame[-6:]

                xs = np.array(list(map(lambda p: p.x, point_frame))).reshape((-1, 1))
                ys = np.array(list(map(lambda p: p.y, point_frame)))
                model = LinearRegression().fit(xs, ys)
                r_sq = model.score(xs, ys)
                print('r_sq:'+str(r_sq))

                if r_sq > 0.5:
                    y_pred = model.predict(xs)
                    p1 = Point(point_frame[0].x, y_pred[0])
                    p2 = Point(point_frame[-1].x, y_pred[-1])
                    current_orientation = Vector(p1, p2)

                else:
                    current_orientation = Vector(last_location, current_location)

                speed = last_location.distance(current_location)/(time.time()-last_location_time)
                print('current speed:'+str(speed))

                # plt.quiver(x0, z0, current_orientation.vector[0], current_orientation.vector[1])
                plt.plot(x0, z0, 'bo', markersize=1)
                plt.pause(0.05)

                if len(way_points) == 0:
                    continue

                current_waypoint = Point(way_points[0][0], way_points[0][1])

                print('distance to waypoint'+str(current_location.distance(current_waypoint)))
                if current_location.distance(current_waypoint) < 0.07:
                    way_points = way_points[1:]
                    print('waypoint reached:'+str(current_waypoint))
                    if len(waypoints) == 1:
                        robot.stop()

                print('['+str(current_location)+', '+str(current_orientation) + ' ,'+str(Vector(current_location, current_waypoint))+'],')
                angle_between = current_orientation.angle_between(Vector(current_location, current_waypoint))

                if angle_between > np.pi/32:
                    clockwise_angle = Vector(current_location, current_waypoint).clockwise_angle_between(current_orientation)
                    if clockwise_angle > np.pi:
                        robot.right((2*np.pi) - clockwise_angle)
                        print('turning right')
                    else:
                        robot.left(clockwise_angle)
                        print('turning left')
                else:
                    robot.straight()
                    print('going forward')

                if speed < 0.1:
                    print("accelerating")
                    robot.accelerate()
                elif speed > 0.3:
                    print("decelerating")
                    robot.decelerate()

                if current_location.distance(last_location) > 0.05:
                    last_location = current_location
                    last_location_time = time.time()

    except KeyboardInterrupt:
        robot.stop()


def listen(q):
    mqttc = ORBMQTTSubscriber(q)
    rc = mqttc.run()
    print("rc: "+str(rc))


waypoints = np.array([
    [0.000156,0.00035],
    [0.142775,0.266247],
    [0.221315,0.558516],
    [0.464738,1.03659],
    [0.744505,1.166092],
    [1.056679,1.139396],
    [1.327786,1.003449],
    [2.755482,-0.120892],
    [2.740217,-0.423676],
    [2.855406,-0.905568],
    [3.048772,-1.1617],
    [3.167079,-1.444698],
    [3.068166,-1.738694],
    [2.8364,-1.938357],
    [2.538329,-2.106215],
    [2.233978,-2.175582],
    [0.485771,-2.223538],
    [0.341137,-1.95841],
    [0.329532,-1.652658]
])

x, y = waypoints.T
fig = plt.figure(figsize=(8, 8))
plt.gca().set_aspect('equal', adjustable='box')
plt.scatter(x, y, marker='+')
plt.pause(0.05)

q = queue.LifoQueue()
x = threading.Thread(target=listen, args=(q,))
x.start()
robot = Robot('http://localhost:5000')

follow(q, waypoints, robot)
