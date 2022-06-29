#!/usr/bin/env python

from encodings import utf_8
import time
import rospy
from std_msgs.msg import String
import moveit_commander
import moveit_msgs.msg
import sys
import cv2
import cv_bridge


class MoveGroupTutorial(object):
    def __init__(self):
        super(MoveGroupTutorial, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('test_move', anonymous=True)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "manipulator"
        group = moveit_commander.MoveGroupCommander(group_name)
        display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)
        planning_frame = group.get_planning_frame()
        eef_link = group.get_end_effector_link()
        group_names = robot.get_group_names()
        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.group = group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def get_current_pos(self):
        current_joints = self.group.get_current_joint_values()
        print(" ")
        print("here is the current joint position added to the list")
        print(current_joints)
        print(" ")
        return current_joints


def talker():
    runAgain = True
    while(runAgain):
        pub = rospy.Publisher(
            '/ur_hardware_interface/script_command', String, queue_size=1000000)
        #rospy.init_node('talker', anonymous=True)
        rate = rospy.Rate(125)  # 10hz
        hello_str = "freedrive_mode()"
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

        val = raw_input("run again? Y or N : ")
        val = val.lower()
        if(val == "y"):
            print("running program again")
            runAgain = True
        else:
            hello_str = "end_freedrive_mode()"
            pub.publish(hello_str)
            rate.sleep()
            runAgain = False


if __name__ == '__main__':
    try:
        posList = []
        numberOfPositions = 10
        tutorial = MoveGroupTutorial()
        for i in range(0, numberOfPositions):
            talker()
            posList.append(tutorial.get_current_pos())
        with open('/home/simtiaz/ur5_ws/src/python_code_for_calibration/posListForCalibration.txt', 'w') as f:
            for item in posList:
                f.write("%s\n" % item)
        f.close()


        
    except rospy.ROSInterruptException:
        pass
