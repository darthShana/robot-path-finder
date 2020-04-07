import paho.mqtt.client as mqtt


class ORBMQTTSubscriber(mqtt.Client):

    def __init__(self, q):
        self.x = -1
        self.z = -1
        self.q = q
        super().__init__()

    def on_connect(self, mqttc, obj, flags, rc):
        print("rc: "+str(rc))

    def on_message(self, mqttc, obj, msg):
        full_str = msg.payload.decode().split(",")
        x0 = float(full_str[0])
        y0 = float(full_str[0])
        z0 = float(full_str[2])
        x_rot = float(full_str[3])
        y_rot = float(full_str[4])
        z_rot = float(full_str[5])
        w_rot = float(full_str[6])

        self.q.put([x0, y0, z0, x_rot, y_rot, z_rot, w_rot])
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
