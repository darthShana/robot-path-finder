import requests


class Robot:

    def __init__(self, host):
        self.host = host
        self.heading = 1500
        self.turing_thrust = 1350
        self.straight_thrust = 1400
        requests.post(self.host + '/robot/session', json={})

    def left(self):
        if self.heading < 1500:
            self.heading = 1500
        if self.heading < 2000:
            self.heading += 100
        requests.post(self.host + '/robot/commands', json={
            'thrust': self.turing_thrust,
            'heading': self.heading
        })

    def right(self):
        if self.heading > 1500:
            self.heading = 1500
        if self.heading > 1000:
            self.heading -= 100
        requests.post(self.host + '/robot/commands', json={
            'thrust': self.turing_thrust,
            'heading': self.heading
        })

    def forward(self):
        if self.thrust < 2000:
            self.thrust -= 50
        requests.post(self.host + '/robot/commands', json={
            'thrust': self.straight_thrust,
            'heading': self.heading
        })

    def backward(self):
        if self.thrust > 1000:
            self.thrust += 50
        requests.post(self.host + '/robot/commands', json={
            'thrust': self.straight_thrust,
            'heading': self.heading
        })
