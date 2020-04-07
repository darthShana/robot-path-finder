import matplotlib.pyplot as plt
import threading
import numpy as np

from multiprocessing import Queue
from geometry.Point import Point
from geometry.Vector import Vector
from mqtt.ORBMQTTSubscriber import ORBMQTTSubscriber



def draw(q):
    fig = plt.figure(figsize=(8, 8))
    plt.gca().set_aspect('equal', adjustable='box')

    waypoints = []

    while True:
        if not q.empty():
            [x0, y0, z0, x_rot, y_rot, z_rot, w_rot] = q.get()
            p = Point(x0, z0)

            if len(waypoints) == 0:
                print(str(p.x)+","+str(p.y))
                # plt.plot(p.x, p.y, 'bo', markersize=1)
                # plt.pause(0.05)
                waypoints.append(p)
            elif waypoints[-1].distance(p) > 0.1:
                if len(waypoints) > 1:
                    if Vector(waypoints[-2], waypoints[-1]).angle_between(Vector(waypoints[-1], p)) > np.pi/16:
                        waypoints.append(p)
                        print(str(p.x)+","+str(p.y))
                        # plt.plot(p.x, p.y, 'bo', markersize=1)
                        # plt.pause(0.05)
                else:
                    waypoints.append(p)
                    print(str(p.x)+","+str(p.y))
                    # plt.plot(p.x, p.y, 'bo', markersize=1)
                    # plt.pause(0.05)

            plt.plot(x0, z0, 'bo', markersize=1)
            plt.pause(0.05)

    plt.show()


def listen(q):
    mqttc = ORBMQTTSubscriber(q)
    rc = mqttc.run()
    print("rc: "+str(rc))


q = Queue()
x = threading.Thread(target=listen, args=(q,))
x.start()

draw(q)
