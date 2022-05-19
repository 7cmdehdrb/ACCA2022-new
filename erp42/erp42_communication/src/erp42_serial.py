#!/usr/bin/env python


import rospy
import rospkg
import serial
import threading
import math as m
from erp42_msgs.msg import SerialFeedBack


exitThread = False


class Control():
    def __init__(self, port_num):

        # Packet Define

        S = 83
        T = 84
        X = 88
        AorM = 1
        ESTOP = 0
        ETX0 = 13
        ETX1 = 10
        ALIVE = 0

        self.DATA = bytearray(14)
        self.DATA[0] = S
        self.DATA[1] = T
        self.DATA[2] = X
        self.DATA[3] = AorM
        self.DATA[4] = ESTOP
        self.DATA[12] = ETX0
        self.DATA[13] = ETX1

        self.feedback_msg = SerialFeedBack()
        self.showData = True

        self.ALIVE = ALIVE
        self.ser = serial.Serial(
            port=port_num,
            baudrate=115200,
        )

        # Threading

        thread = threading.Thread(target=self.receive_data)
        thread.daemon = True
        thread.start()

    def receive_data(self):

        # Read Serial

        line = []

        while not exitThread:
            try:
                for i in self.ser.read():
                    line.append(i)
                    if ord(i) == 83:
                        del line[:]
                        line.append(i)
                    elif ord(i) == 10 and ord(line[-1]) == 10 and len(line) == 18:
                        self.handle_data(line)
                        del line[:]
                        break
                    if len(line) >= 18:
                        del line[:]
                        break

            except Exception as ex:
                print(ex)

    def handle_data(self, line):

        # Transfer packet data to AorM / Gear / Speed(m/s) / Steer(rad)

        # AorM - 0: M / 1: A
        feedback_AorM = ord(line[4])

        # Gear - 0: B / 1: N / 2: D
        feedback_gear = ord(line[5])

        # Speed (m/s)
        feedback_KPH = (ord(line[6]) + ord(line[7]) * 256) / 10
        feedback_speed = self.kph2mps(value=feedback_KPH)

        # Steer (RAD)
        feedback_DEG = ord(line[8]) + ord(line[9]) * 256

        if feedback_DEG >= 23768:
            feedback_DEG -= 65536

        if feedback_DEG != 0:
            feedback_DEG /= 71.0

        feedback_steer = m.radians(feedback_DEG)

        # Brake
        feedback_brake = ord(line[10])

        # Encoder
        self.feedback_encoder = (ord(line[11]) + ord(line[12]) * 256 + ord(
            line[13]) * 256 * 256 + ord(line[14]) * 256 * 256 * 256)

        if self.feedback_encoder >= 2147483648:
            self.feedback_encoder -= 4294967296

        data = SerialFeedBack()

        data.MorA = feedback_AorM
        data.Gear = feedback_gear
        data.speed = feedback_speed
        data.steer = feedback_steer
        data.brake = feedback_brake
        data.encoder = self.feedback_encoder
        data.alive = self.ALIVE

        self.feedback_msg = data

        if self.showData is True:
            print(self.feedback_msg)

    """ Util """

    def kph2mps(self, value):
        return value * 0.277778

    def mps2kph(self, value):
        return value * 3.6


if __name__ == "__main__":
    rospy.init_node("erp42_serial")

    # Params
    port = rospy.get_param("/erp42_port", "/dev/ttyUSB0")

    # Objects
    control = Control(port_num=port)

    # Publisher
    feedback_pub = rospy.Publisher(
        "/erp42_feedback", SerialFeedBack, queue_size=1)

    rate = rospy.Rate(50.0)
    while not rospy.is_shutdown():
        feedback_pub.publish(control.feedback_msg)

        rate.sleep()