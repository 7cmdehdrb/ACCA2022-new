#!/usr/bin/python3

import serial
import threading
from time import sleep


class Sensor(object):
    def __init__(self, port_num):

        self.port_num = port_num
        self.data = None

        self.ser = serial.Serial(
            port=port_num,
            baudrate=9600,
        )

        self.th = threading.Thread(target=self.receive_data)
        self.th.daemon = True
        self.th.start()

    def receive_data(self):
        line = []

        while True:
            try:
                data = self.ser.read()
                line.append(ord(data))

                # print(len(line))

                if len(line) >= 10:
                    if line[-1] == 73:
                        print(line)
                        del line[:]

            except Exception as ex:
                print(ex)


if __name__ == "__main__":
    sensor = Sensor("/dev/ttyACM0")

    while True:
        # print(sensor.data)
        sleep(0.01)
