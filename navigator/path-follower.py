import numpy as np
from matplotlib import pyplot as plt
from mqtt.ORBMQTTSubscriber import ORBMQTTSubscriber

import threading
import numpy as np

import queue
from geometry.Point import Point
from geometry.Vector import Vector
from sklearn.linear_model import LinearRegression
from Robot import Robot


def follow(q, way_points, robot):

    point_frame = []
    last_orientation = Vector(Point(0, 0), Point(0, 1))
    last_location = Point(0, 0)

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
                print('heading score:'+str(r_sq))

                y_pred = model.predict(xs)
                p1 = Point(point_frame[0].x, y_pred[0])
                p2 = Point(point_frame[-1].x, y_pred[-1])
                current_orientation = Vector(p1, p2)
                distance = p1.distance(p2)
                print('current speed:'+str(distance))

                # plt.quiver(x0, z0, current_orientation.vector[0], current_orientation.vector[1])
                plt.plot(x0, z0, 'bo', markersize=1)
                plt.pause(0.05)

                if len(way_points) == 0:
                    continue

                current_waypoint = Point(way_points[0][0], way_points[0][1])

                print('distance to waypoint'+str(current_location.distance(current_waypoint)))
                if current_location.distance(current_waypoint) < 0.05:
                    way_points = way_points[1:]
                    print('waypoint reached:'+str(current_waypoint))
                    if len(waypoints) == 1:
                        robot.stop()

                angle_between = current_orientation.angle_between(last_orientation)
                if angle_between > np.pi/32:
                    print(str(current_location)+', '+str(current_orientation) + ' ,'+str(Vector(current_location, current_waypoint)))

                    if angle_between > np.pi/16:
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
                    last_orientation = current_orientation

                if distance < 0.01:
                    print("accelerating")
                    robot.accelerate()
                elif distance > 0.05:
                    print("decelerating")
                    robot.decelerate()

                if current_location.distance(last_location) > 0.05:
                    last_location = current_location

    except KeyboardInterrupt:
        robot.stop()


def listen(q):
    mqttc = ORBMQTTSubscriber(q)
    rc = mqttc.run()
    print("rc: "+str(rc))


waypoints = np.array([
    [-0.000747,-0.00075],
    [0.032325,0.095675],
    [0.109664,0.226586],
    [0.183259,0.441623],
    [0.339158,0.706729],
    [0.573945,0.966652],
    [0.669111,1.015304],
    [0.821146,1.040087],
    [1.36834,0.99612],
    [1.467512,0.938207],
    [1.723498,0.703704],
    [1.99377,0.541792],
    [3.909013,-1.200831],
    [3.94094,-1.31204],
    [4.009097,-2.376009],
    [3.879463,-2.446921],
    [3.616779,-2.525115],
    [3.504275,-2.514579],
    [2.285571,-2.643726],
    [2.178937,-2.69175],
    [2.046591,-2.717553],
    [1.93807,-2.763528],
    [1.47592,-2.847043],
    [1.364852,-2.836777],
    [1.282793,-2.773348],
    [0.770846,-2.520971],
    [0.648515,-2.422256],
    [0.608668,-2.311986]
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
