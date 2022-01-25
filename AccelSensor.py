import serial
import math


class AccelSensor:
    def __init__(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate

    def __enter__(self):
        self.ser = serial.Serial(self.port, self.baudrate, timeout=None)
        self.ser.setDTR(False)

        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.ser.close()

    def get_xy_angles(self):
        input_str = str(self.ser.readline())
        splitted = input_str.strip('b').strip("'").strip().split(',')

        try:
            splitted = list(map(float, splitted[:3]))
        except ValueError:
            return 0, 0

        angle_x = math.atan2(splitted[0], splitted[2])
        angle_y = math.atan2(splitted[1], splitted[2])
        return angle_x, angle_y

