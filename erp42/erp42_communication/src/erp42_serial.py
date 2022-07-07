#!/usr/bin/env python


import rospy
import rospkg
import serial
import threading
import math as m
from erp42_msgs.msg import SerialFeedBack
from erp42_control.msg import ControlMessage


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

        self.ALIVE = ALIVE

        # Publisher
        self.feedback_pub = rospy.Publisher(
            "/erp42_feedback", SerialFeedBack, queue_size=1)

        # Subscriber
        self.control_sub = rospy.Subscriber(
            "/cmd_msg", ControlMessage, callback=self.cmdCallback
        )

        self.feedback_msg = SerialFeedBack()
        self.cmd_msg = ControlMessage()

        self.showData = True
        self.ser = serial.Serial(
            port=port_num,
            baudrate=115200,
        )

        # Threading
        thread = threading.Thread(target=self.receive_data)
        thread.daemon = True
        thread.start()

    def cmdCallback(self, msg):
        self.cmd_msg = msg

    def send_data(self, data=ControlMessage()):
        # Speed
        SPEED = data.Speed * 10

        # Steer
        STEER = data.Steer * 71
        if STEER > 1999:
            STEER = 1999
        if STEER < -1999:
            STEER = -1999

        if STEER >= 0:
            self.DATA[8] = int(STEER // 256)
            self.DATA[9] = int(STEER % 256)
        else:
            STEER = -STEER
            self.DATA[8] = int(255 - STEER // 256)
            self.DATA[9] = int(255 - STEER % 256)

        self.DATA[5] = data.Gear    # GEAR
        self.DATA[6] = int(SPEED // 256)
        self.DATA[7] = int(SPEED % 256)
        self.DATA[10] = data.brake   # BREAK
        self.DATA[11] = self.ALIVE

        self.ser.write(self.DATA)

        self.ALIVE = self.ALIVE + 1
        if self.ALIVE == 256:
            self.ALIVE = 0

    def receive_data(self):

        # Read Serial

        line = []

        while not exitThread:
            try:
                for i in self.ser.read():
                    # print(i)
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

        self.feedback_pub.publish(self.feedback_msg)

        if self.showData is True:
            rospy.loginfo(self.feedback_msg)

    """ Util """

    def kph2mps(self, value):
        return value * 0.277778

    def mps2kph(self, value):
        return value * 3.6


if __name__ == "__main__":
    rospy.init_node("erp42_serial")

    # Params
    port = rospy.get_param("/erp42_serial/erp_port", "/dev/ttyUSB1")
    print(port)

    # Objects
    control = Control(port_num=port)

    rate = rospy.Rate(50.0)
    while not rospy.is_shutdown():
        control.send_data(data=control.cmd_msg)
        rate.sleep()
