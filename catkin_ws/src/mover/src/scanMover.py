#!/usr/bin/python3
__author__ = 'saimtiaz'

# ------------------------------------------------------
""" This file sends joint angles to UR5 via sockets.
    It moves the UR5 in predefined locations in order to scan the environment
"""
# ------------------------------------------------------

from std_msgs.msg import Bool, String
import numpy as np
import readchar
import rospy



HOST = '10.130.229.172'
PORT = 30002

POSITIONS = [
[0.06668996810913086, -3.1270230452166956, -0.6116192976581019, 0.6962059736251831, -0.0242694059955042, 0.6914036870002747],
[0.06671393662691116, -3.1265795866595667, -0.6116192976581019, 0.6961339712142944, -0.024317089711324513, 0.69139164686203],
[-0.7593143622027796, -1.7873900572406214, -1.6221469084369105, -0.38633805910219365, 1.4246536493301392, 0.6930927038192749],
[-1.3812254110919397, -1.8471029440509241, -1.6256230513202112, -0.38639766374696904, 1.4246536493301392, 0.6931046843528748],
[-1.7027786413775843, -1.7620485464679163, -1.9336312452899378, -0.3864339033709925, 1.4246296882629395, 0.6931526064872742],
[-1.5275867621051233, -0.8122738043414515, -2.640677038823263, -0.3827174345599573, 1.4246536493301392, 0.6930687427520752],
[-1.5276950041400355, -0.8122618834124964, -2.640221659337179, -0.382789436970846, 1.4246655702590942, 0.6931406259536743],
[-0.9854205290423792, -0.9759777227984827, -2.4800408522235315, -0.3828495184527796, 1.4246296882629395, 0.6930687427520752],
[-0.7639525572406214, -1.9109762350665491, -1.65991717973818, -0.3829692045794886, 1.4246655702590942, 0.6931406259536743],
[-0.9076097647296351, -2.3709686438189905, -1.110657040272848, -0.38304120699037725, 1.4246296882629395, 0.6931166648864746]
]

OFFSET = [0, 0, 0, 0, 0, 0]
quit_pub = rospy.Publisher('/quitScan', Bool, queue_size=5)
scan_pub = rospy.Publisher('/getNextScan', Bool, queue_size=5)
urPub = rospy.Publisher(  '/ur_hardware_interface/script_command', 
                    String, 
                    queue_size=10000
)

if __name__ == '__main__':

    #Initialize the node
    rospy.init_node('moveScan_ur5', anonymous=True)
    print('node initialized')

    #Move the UR5 to initial position
    print('resetting robot!')
    q = np.array(POSITIONS[0]) + np.array(OFFSET)
    command = "servoj([{0},{1},{2},{3},{4},{5}],t={7},gain={6})".format(str(
        q[0]), str(q[1]), str(q[2]), str(q[3]), str(q[4]), str(q[5]), '300', 4.) + "\n"
    urPub.publish(command)

    #Move the UR5 to each position
    rate = rospy.Rate(125)
    idx = 0
    positionCount = len(POSITIONS)
    print('Robot reset!')
    print('Press c to close!')
    print('Press n to move to the next position!')
    print('Press p to take a frame!')

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
                idx = 0
            print("Moving to next position")
            q = np.array(POSITIONS[idx]) + np.array(OFFSET)
            command = "servoj([{0},{1},{2},{3},{4},{5}],t={7},gain={6})".format(str(
                q[0]), str(q[1]), str(q[2]), str(q[3]), str(q[4]), str(q[5]), '300', 4.) + "\n"
            urPub.publish(command)
        
        #Send command to take a scan
        elif key == 'p':
            print("Sending scan request")
            takeScan = Bool()
            takeScan.data = True
            scan_pub.publish(takeScan)

        rate.sleep()
