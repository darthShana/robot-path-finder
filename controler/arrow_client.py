import sys,tty,termios
from Robot import Robot

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
