#!/usr/bin/python3
__author__ = 'pragathip'

# ------------------------------------------------------
""" This file sends joint angles to UR5 via sockets. """
# ------------------------------------------------------

import numpy as np
import socket
import math as M
import sys
import rospy
import signal
from relaxed_ik_ros1.msg import JointAngles

HOST = '10.130.229.172'
PORT = 30002
STARTING_CONFIG = [2.775576054239318, -0.28314665537862765, -2.010683613407321,
                   1.8286544095874662, 1.6760841024252917, -0.052737232866035225]

OFFSET = [-M.pi, -M.pi / 2, 0, -M.pi / 2, 0, M.pi / 4]


class JointAnglesHandler():
    def __init__(self):
        self.ja_sub = rospy.Subscriber(
            '/relaxed_ik/joint_angle_solutions', JointAngles, self.get_ja)
        self.ja_solution = ''

    def get_ja(self, data):
        self.ja_solution = [a for a in data.angles.data]


def signal_handler(signal, frame):
    sys.exit()


if __name__ == '__main__':

    #Start the node
    print('initialized')
    rospy.init_node('move_ur5', anonymous=True)
    print('node initialized')

    #Initialize joint angle handler
    ja_handler = JointAnglesHandler()

    #Connect to the UR5
    print('attempting to connect')
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    print('connecting to robot...')
    sock.connect((HOST, PORT))

    #Reset robot position to starting configuration
    print('resetting robot!')
    q = np.array(STARTING_CONFIG) + np.array(OFFSET)
    command = "servoj([{0},{1},{2},{3},{4},{5}],t={7},gain={6})".format(str(
        q[0]), str(q[1]), str(q[2]), str(q[3]), str(q[4]), str(q[5]), '300', 4.) + "\n"
    sock.send(command.encode('utf8'))

    #Wait for initial movement command
    rate = rospy.Rate(125)
    idx = 1
    while ja_handler.ja_solution == '':
        if idx % 200 == 0:
            print('waiting for robot solution...')
        if idx == 1000:
            idx = 1
        idx += 1
        signal.signal(signal.SIGINT, signal_handler)
        rate.sleep()

    #Move the robot based on movement commands
    while not rospy.is_shutdown():
        q = np.array(ja_handler.ja_solution) + np.array(OFFSET)
        command = "servoj([{0},{1},{2},{3},{4},{5}],t={7},gain={6})".format(str(
            q[0]), str(q[1]), str(q[2]), str(q[3]), str(q[4]), str(q[5]), '300', .1) + "\n"
        sock.send(command.encode('utf8'))

        rate.sleep()
