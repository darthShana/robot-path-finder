import requests


class Robot:

    def __init__(self, host):
        self.host = host
        self.heading = 1500
        self.stop_thrust = 1500
        self.thrust = 1500
        requests.post(self.host + '/robot/session', json={})

    def left(self):
        if self.heading < 1500:
            self.heading = 1500
        if self.heading < 2000:
            self.heading += 50

        requests.post(self.host + '/robot/commands', json={
            'thrust': self.thrust,
            'heading': self.heading
        })

    def right(self):
        if self.heading > 1500:
            self.heading = 1500
        if self.heading > 1000:
            self.heading -= 50

        requests.post(self.host + '/robot/commands', json={
            'thrust': self.thrust,
            'heading': self.heading
        })

    def straight(self):
        self.heading = 1500

        requests.post(self.host + '/robot/commands', json={
            'thrust': self.thrust,
            'heading': self.heading
        })

    def accelerate(self):
        if self.thrust > 1000:
            self.thrust -= 10
        requests.post(self.host + '/robot/commands', json={
            'thrust': self.thrust,
            'heading': self.heading
        })

    def decelerate(self):
        if self.thrust < 2000:
            self.thrust += 10
        requests.post(self.host + '/robot/commands', json={
            'thrust': self.thrust,
            'heading': self.heading
        })

    def stop(self):
        self.heading = 1500
        requests.post(self.host + '/robot/commands', json={
            'thrust': self.stop_thrust,
            'heading': self.heading
        })
