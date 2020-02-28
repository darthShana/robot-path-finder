import sys,tty,termios
import requests


class _Getch:

    def __call__(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(3)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

class Robot:

    def __init__(self, host):
        self.host = host
        self.thrust = 1500
        self.heading = 1500
        requests.post(self.host + '/robot/session', json={})

    def left(self):
        if self.heading < 2000:
            self.heading += 100
        requests.post(self.host + '/robot/commands', json={
            'thrust': self.thrust,
            'heading': self.heading
        })

    def right(self):
        if self.heading > 1000:
            self.heading -= 100
        requests.post(self.host + '/robot/commands', json={
            'thrust': self.thrust,
            'heading': self.heading
        })

    def forward(self):
        if self.thrust < 2000:
            self.thrust -= 50
        requests.post(self.host + '/robot/commands', json={
            'thrust': self.thrust,
            'heading': self.heading
        })

    def backward(self):
        if self.thrust > 1000:
            self.thrust += 50
        requests.post(self.host + '/robot/commands', json={
            'thrust': self.thrust,
            'heading': self.heading
        })

def get(stub):
    inkey = _Getch()

    while(1):
        k=inkey()
        if k!='':break
    if k=='\x1b[A':
        print("up")
        stub.forward()
    elif k=='\x1b[B':
        print("down")
        stub.backward()
    elif k=='\x1b[C':
        print("right")
        stub.right()
    elif k=='\x1b[D':
        print("left")
        stub.left()
    else:
        print("not an arrow key!")
        sys.exit()


def main():
    robot = Robot('http://localhost:5000')
    while True:
        get(robot)


if __name__=='__main__':
    main()
