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
    current_orientation = Vector(Point(0, 0), Point(0, 1))
    robot.straight()

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
                y_pred = model.predict(xs)
                p1 = Point(point_frame[0].x, y_pred[0])
                p2 = Point(point_frame[-1].x, y_pred[-1])
                distance = p1.distance(p2)

                if distance > 0.01:
                    current_orientation = Vector(p1, p2)
                    print('current speed:'+str(distance))
                else:
                    print('speed too low for heading:'+str(distance))

                # plt.quiver(x0, z0, current_orientation.vector[0], current_orientation.vector[1])
                plt.plot(x0, z0, 'bo', markersize=1)
                plt.pause(0.05)

                if len(way_points) == 0:
                    print('['+str(current_location)+', '+str(current_orientation) + ' ,'+str(Vector(current_location, current_location))+'],')
                    continue

                current_waypoint = Point(way_points[0][0], way_points[0][1])

                print('distance to waypoint'+str(current_location.distance(current_waypoint)))
                if current_location.distance(current_waypoint) < 0.05:
                    way_points = way_points[1:]
                    print('waypoint reached:'+str(current_waypoint))
                    if len(waypoints) == 1:
                        print("destination reached")
                        robot.stop()
                        continue

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

                if distance < 0.03:
                    print("accelerating")
                    robot.accelerate()
                elif distance > 0.05:
                    print("decelerating")
                    robot.decelerate()



    except KeyboardInterrupt:
        robot.stop()


def listen(q):
    mqttc = ORBMQTTSubscriber(q)
    rc = mqttc.run()
    print("rc: "+str(rc))


waypoints = np.array([
    [-0.000458,-0.000361],
    [0.142214,0.270564],
    [0.223221,0.573814],
    [0.676189,1.493873],
    [0.932008,1.659184],
    [1.263141,1.724789],
    [1.571818,1.686709],
    [1.848657,1.567276],
    [4.083863,0.020822],
    [3.803652,-0.120183],
    [3.502052,-0.121533],
    [2.502722,0.07397],
    [-0.011644,0.010546],
    [0.044619,0.306758],
    [0.173498,0.622892],
    [0.233017,0.939582],
    [0.134243,1.248281],
    [-0.05364,1.490692],
    [-0.301321,1.687501]
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
