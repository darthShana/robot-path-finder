import paho.mqtt.client as mqtt
import matplotlib.pyplot as plt
import threading

from multiprocessing import Queue


class ORBMQTTSubscriber(mqtt.Client):

    def __init__(self, q):
        self.x = 0
        self.z = 0
        super().__init__()


    def on_connect(self, mqttc, obj, flags, rc):
        print("rc: "+str(rc))

    def on_message(self, mqttc, obj, msg):
        full_str = msg.payload.decode().split(",")
        x = float(full_str[0])
        z = float(full_str[2])

        if self.x != x or self.z != z:
            self.q.put([x, z])
            self.x = x
            self.z = z

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

    while True:
        if not q.empty():
            [x0, y0] = q.get()
            print(x0)
            print(y0)

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