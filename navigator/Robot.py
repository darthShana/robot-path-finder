import requests
import numpy as np


class Robot:

    def __init__(self, host):
        self.host = host
        self.heading = 1550
        self.stop_thrust = 1500
        self.thrust = 1500
        requests.post(self.host + '/robot/session', json={})

    def left(self, angle):
        if angle > np.pi/4:
            angle = np.pi/4

        self.heading = round(1550 + (angle*4*600/np.pi))
        print('thrust:'+str(self.thrust)+' heading:'+str(self.heading))

        requests.post(self.host + '/robot/commands', json={
            'thrust': self.thrust,
            'heading': self.heading
        })

    def right(self, angle):
        if angle > np.pi/4:
            angle = np.pi/4

        self.heading = round(1550 - (angle*4*600/np.pi))
        print('thrust:'+str(self.thrust)+' heading:'+str(self.heading))

        requests.post(self.host + '/robot/commands', json={
            'thrust': self.thrust,
            'heading': self.heading
        })

    def straight(self):
        self.heading = 1550

        requests.post(self.host + '/robot/commands', json={
            'thrust': self.thrust,
            'heading': self.heading
        })

    def boost(self):
        self.thrust = 1400

        requests.post(self.host + '/robot/commands', json={
            'thrust': self.thrust,
            'heading': self.heading
        })

    def accelerate(self):
        if self.thrust > 1300:
            self.thrust -= 1

        print('thrust:'+str(self.thrust)+' heading:'+str(self.heading))

        requests.post(self.host + '/robot/commands', json={
            'thrust': self.thrust,
            'heading': self.heading
        })

    def decelerate(self):
        if self.thrust < 1500:
            self.thrust += 1

        print('thrust:'+str(self.thrust)+' heading:'+str(self.heading))

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
