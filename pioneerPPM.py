import serial
import time

p = r = y = t = 0


class Drone(object):
    def __init__(self, com_port):
        self.ar = serial.Serial(f'COM{com_port}', 115200)  # initiating arduino at COM port
        time.sleep(2)
        for i in range(4):
            self.send(i, 1500)
        time.sleep(3)

    def send(self, ch, val):  # function to send specific value to specific channel (channel numeration goes from zero)
        self.ar.write(f'{ch}{val};'.encode('utf-8'))

    # set_yaw, set_pitch, set_roll and set_throttle functions take number from -1 to 1
    def set_yaw(self, a):  # negative value -- drone turns left
        global y  # positive value -- drone turns right
        if a != y:
            y = a
            print('yaw', a * 500 + 1500)
            a = a * 500 + 1500
            self.send(3, a)

    def set_throttle(self, a):  # negative value -- drone goes down
        global t  # positive value -- drone goes up
        if a != t:
            t = a
            print('throttle', a)
            a = a * 500 + 1500
            self.send(2, a)

    def set_roll(self, a):  # negative value -- drone goes left
        global r  # positive value -- drone goes right
        if a != r:
            r = a
            print('roll', a)
            a = a * 500 + 1500
            self.send(0, a)

    def set_pitch(self, a):  # negative value -- drone goes backward
        global p  # positive value -- drone goes forward
        if a != p:
            p = a
            print('pitch', a)
            a = a * 500 + 1500
            self.send(1, a)

    def arm(self):  # arming drone
        self.send(0, 1500)
        self.send(1, 1500)
        self.send(2, 1000)
        self.send(3, 1000)
        self.send(4, 2000)
        self.send(5, 1000)
        self.send(6, 1000)
        self.send(7, 1000)
        time.sleep(3)
        self.send(0, 1500)
        self.send(1, 1500)
        self.send(2, 1500)
        self.send(3, 1500)
        time.sleep(1)

    def disarm(self):  # disarming drone
        self.send(0, 1500)
        self.send(1, 1500)
        self.send(2, 1000)
        self.send(3, 2000)
        time.sleep(3)
        self.send(0, 1500)
        self.send(1, 1500)
        self.send(2, 1500)
        self.send(3, 1500)
        time.sleep(1)

    def hover(self, seconds):  # same as time.sleep, but cooler :)
        time.sleep(seconds)
