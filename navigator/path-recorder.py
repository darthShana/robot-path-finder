import paho.mqtt.client as mqtt
import matplotlib.pyplot as plt
import threading
import numpy as np

from multiprocessing import Queue
from geometry.Point import Point
from geometry.Vector import Vector


class ORBMQTTSubscriber(mqtt.Client):

    def __init__(self, q):
        self.x = 0
        self.z = 0
        self.q = q
        super().__init__()

    def on_connect(self, mqttc, obj, flags, rc):
        print("rc: "+str(rc))

    def on_message(self, mqttc, obj, msg):
        full_str = msg.payload.decode().split(",")
        x0 = float(full_str[0])
        z0 = float(full_str[2])

        if abs(self.x - x0) > 0.01 or abs(self.z - z0) > 0.01:
            self.q.put([x0, z0])
            self.x = x0
            self.z = z0

    def on_publish(self, mqttc, obj, mid):
        pass

    def on_subscribe(self, mqttc, obj, mid, granted_qos):
        print("Subscribed: "+str(mid)+" "+str(granted_qos))

    def on_log(self, mqttc, obj, level, string):
        pass

    def run(self):
        self.connect("localhost", 1883, 60)
        self.subscribe("robot/sensors/orbslam", 0)

        rc = 0
        while rc == 0:
            rc = self.loop()
        return rc


def draw(q):
    fig = plt.figure(figsize=(8, 8))
    plt.gca().set_aspect('equal', adjustable='box')

    waypoints = []

    while True:
        if not q.empty():
            [x0, y0] = q.get()
            p = Point(x0, y0)

            if len(waypoints) == 0:
                print(str(p.x)+","+str(p.y))
                plt.plot(p.x, p.y, 'ro', markersize=1)
                plt.pause(0.05)
                waypoints.append(p)
            elif waypoints[-1].distance(p) > 0.2:
                if len(waypoints) > 1:
                    if Vector(waypoints[-2], waypoints[-1]).angle_between(Vector(waypoints[-1], p)) > np.pi/8:
                        waypoints.append(p)
                        print(str(p.x)+","+str(p.y))
                        plt.plot(p.x, p.y, 'ro', markersize=1)
                        plt.pause(0.05)
            else:
                waypoints.append(p)
                print(str(p.x)+","+str(p.y))
                plt.plot(p.x, p.y, 'ro', markersize=1)
                plt.pause(0.05)

            plt.plot(x0, y0, 'bo', markersize=1)
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
