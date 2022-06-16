#!/usr/bin/python3
__author__ = 'saimtiaz'

# ------------------------------------------------------
""" This file sends joint angles to UR5 via sockets.
    It moves the UR5 in predefined locations in order to scan the environment
"""
# ------------------------------------------------------

from turtle import position
import numpy as np
import readchar
import socket
import math as M
import sys
import rospy
import signal
from relaxed_ik_ros1.msg import JointAngles
from std_msgs.msg import Bool

HOST = '10.130.229.165'
PORT = 30002

#TODO Populate with actual positions
POSITIONS = [
    [4.775576054239318, -0.28314665537862765, -2.010683613407321, 1.8286544095874662, 1.6760841024252917, -0.052737232866035225],
    [3.775576054239318, -0.28314665537862765, -2.010683613407321, 1.8286544095874662, 1.6760841024252917, -0.052737232866035225],
    [2.775576054239318, -0.28314665537862765, -2.010683613407321, 1.8286544095874662, 1.6760841024252917, -0.052737232866035225]
    
]
OFFSET = [-M.pi, -M.pi / 2, 0, -M.pi / 2, 0, M.pi / 4]

#TODO publish out to get next scan
SCAN_TOPIC = "/getNextScan";

quit_pub = rospy.Publisher('/quitScan', Bool, queue_size=5)
scan_pub = rospy.Publisher('/getNextScan', Bool, queue_size=5)


# class JointAnglesHandler():
#     def __init__(self):
#         self.ja_sub = rospy.Subscriber(
#             '/relaxed_ik/joint_angle_solutions', JointAngles, self.get_ja)
#         self.ja_solution = ''

#     def get_ja(self, data):
#         self.ja_solution = []
#         for a in data.angles.data:
#             self.ja_solution.append(a)


# def signal_handler(signal, frame):
#     sys.exit()


if __name__ == '__main__':
    #Connect to the UR5 via socket
    print('initialized')
    rospy.init_node('moveScan_ur5', anonymous=True)
    print('node initialized')
    # ja_handler = JointAnglesHandler()
    print('attempting to connect')
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    print('connecting to robot...')
    sock.connect((HOST, PORT))

    #Move the UR5 to initial position
    print('resetting robot!')
    q = np.array(POSITIONS[0]) + np.array(OFFSET)
    command = "servoj([{0},{1},{2},{3},{4},{5}],t={7},gain={6})".format(str(
        q[0]), str(q[1]), str(q[2]), str(q[3]), str(q[4]), str(q[5]), '300', 4.) + "\n"
    sock.send(command.encode('utf8'))

    #Move the UR5 to each position
    rate = rospy.Rate(125)
    idx = 0
    positionCount = len(POSITIONS)
    while not rospy.is_shutdown():
        #Read in character
        key = readchar.readkey()

        #Close scanMove if c is typed in
        if key == 'c':
            q = Bool()
            q.data = True
            quit_pub.publish(q)
            rospy.signal_shutdown()

        #Else, move to the next position
        elif key == 'n':
            if idx < positionCount - 1:
                idx = idx + 1
            else:
                idx = idx - 1
            #q = np.array(ja_handler.ja_solution) + np.array(OFFSET)
            q = np.array(POSITIONS[idx]) + np.array(OFFSET)
            command = "servoj([{0},{1},{2},{3},{4},{5}],t={7},gain={6})".format(str(
                q[0]), str(q[1]), str(q[2]), str(q[3]), str(q[4]), str(q[5]), '300', 4.) + "\n"
            sock.send(command.encode('utf8'))
        
        #Send command to take a scan
        elif key == 'p':
            takeScan = Bool()
            takeScan.data = True
            scan_pub.publish(takeScan)

        rate.sleep()
