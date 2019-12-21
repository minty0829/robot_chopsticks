#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
import numpy as np
from numpy import linalg
from baxter_interface import gripper as robot_gripper

def move_hand_init():  
    #Initiaize other variables
    # right_gripper = robot_gripper.Gripper('right')
    # left_gripper = robot_gripper.Gripper('left')

    # #right_gripper.calibrate()
    # right_gripper.close()

    #Move to outside position
    #Move right hand to outside position
    raw_input('Press [Enter]:')

    #Construct the request for right hand
    request = GetPositionIKRequest()
    request.ik_request.group_name = "right_arm"
    request.ik_request.ik_link_name = "right_gripper"
    request.ik_request.attempts = 500
    request.ik_request.pose_stamped.header.frame_id = "base"
        
    #Set the desired orientation for the end effector HERE
    request.ik_request.pose_stamped.pose.position.x = 0.672
    request.ik_request.pose_stamped.pose.position.y = -0.3
    request.ik_request.pose_stamped.pose.position.z = 0.02
    request.ik_request.pose_stamped.pose.orientation.x = 1.0
    request.ik_request.pose_stamped.pose.orientation.y = 0.0
    request.ik_request.pose_stamped.pose.orientation.z = 0.0
    request.ik_request.pose_stamped.pose.orientation.w = 0.0

    try:

        group = MoveGroupCommander("right_arm")

        # Setting position and orientation target
        group.set_pose_target(request.ik_request.pose_stamped)

        # Plan IK and execute
        group.go()

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        
    #Move left hand to outside position
    raw_input('Press [Enter]:')

    #Construct the request for left hand
    request = GetPositionIKRequest()
    request.ik_request.group_name = "left_arm"
    request.ik_request.ik_link_name = "left_gripper"
    request.ik_request.attempts = 500
    request.ik_request.pose_stamped.header.frame_id = "base"
            
    #Set the desired orientation for the end effector HERE
    request.ik_request.pose_stamped.pose.position.x = 0.672
    request.ik_request.pose_stamped.pose.position.y = 0.113
    request.ik_request.pose_stamped.pose.position.z = 0.02 
    request.ik_request.pose_stamped.pose.orientation.x = 1.0
    request.ik_request.pose_stamped.pose.orientation.y = 0.0
    request.ik_request.pose_stamped.pose.orientation.z = 0.0
    request.ik_request.pose_stamped.pose.orientation.w = 0.0
        
        
    try:
        group = MoveGroupCommander("left_arm")
        # Setting position and orientation target
        group.set_pose_target(request.ik_request.pose_stamped)

        # Plan IK and execute
        group.go()

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e