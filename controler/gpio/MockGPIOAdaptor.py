

class MockGPIOAdaptor:

    def __init__(self):
        print("initialising....")

    def set_thrust(self, thrust):
        print(thrust)

    def set_heading(self, heading):
        print(heading)
