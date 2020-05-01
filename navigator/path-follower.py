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
    last_location = Point(0, 0)
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
                speed = p1.distance(p2)

                if speed > 0.03:
                    current_orientation = Vector(p1, p2)
                    print('current speed:'+str(speed))
                else:
                    current_orientation = Vector(last_location, current_location)
                    print('speed too low for heading:'+str(speed))

                # plt.quiver(x0, z0, current_orientation.vector[0], current_orientation.vector[1])
                plt.plot(x0, z0, 'bo', markersize=1)
                plt.pause(0.05)

                if len(way_points) == 0:
                    print('['+str(current_location)+', '+str(current_orientation) + ' ,'+str(Vector(current_location, current_location))+'],')
                    continue

                current_waypoint = Point(way_points[0][0], way_points[0][1])

                print('distance to waypoint'+str(current_location.distance(current_waypoint)))
                if current_location.distance(current_waypoint) < 0.1:
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

                if speed < 0.03:
                    print("accelerating")
                    robot.accelerate()
                elif speed > 0.05:
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
    [-0.000822,-0.000577],
    [0.132452,0.276892],
    [1.178483,1.647386],
    [1.178483,1.647386]
    # [1.49917,1.676569],
    # [1.8522,1.585737],
    # [2.176578,1.415258],
    # [3.556403,1.039474],
    # [3.970087,1.03458],
    # [4.316127,0.935356],
    # [4.609102,0.673901],
    # [4.682612,0.311295],
    # [4.673847,-0.10979],
    # [4.796222,-0.73753],
    # [4.920383,-1.017392],
    # [4.886913,-1.331996],
    # [4.763519,-1.695554],
    # [4.532061,-1.964554],
    # [4.197813,-2.051195],
    # [3.066932,-2.102243],
    # [2.868526,-1.847282],
    # [0.105939,0.095283],
    # [0.147818,0.413407],
    # [0.078416,1.26492]
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
