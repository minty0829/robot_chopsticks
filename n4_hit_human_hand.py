#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
import numpy as np
from numpy import linalg
from baxter_interface import gripper as robot_gripper

def move_hand(robot_hand, human_hand):  
    #Initiaize other variables
    right_gripper = robot_gripper.Gripper('right')
    left_gripper = robot_gripper.Gripper('left')

    #right_gripper.calibrate()
    right_gripper.close()

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
    request.ik_request.pose_stamped.pose.position.x = 0.5
    request.ik_request.pose_stamped.pose.position.y = -0.35
    request.ik_request.pose_stamped.pose.position.z = 0.4
    request.ik_request.pose_stamped.pose.orientation.x = 0.0
    request.ik_request.pose_stamped.pose.orientation.y = 1.0
    request.ik_request.pose_stamped.pose.orientation.z = 0.0
    request.ik_request.pose_stamped.pose.orientation.w = 0.0

    try:

        group = MoveGroupCommander("right_arm")

        # Setting position and orientation target
        group.set_pose_target(request.ik_request.pose_stamped)

        print("Trying to move the right arm out of the way!")
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
    request.ik_request.pose_stamped.pose.position.x = 0.5
    request.ik_request.pose_stamped.pose.position.y = 0.35
    request.ik_request.pose_stamped.pose.position.z = 0.4
    request.ik_request.pose_stamped.pose.orientation.x = 0.0
    request.ik_request.pose_stamped.pose.orientation.y = 1.0
    request.ik_request.pose_stamped.pose.orientation.z = 0.0
    request.ik_request.pose_stamped.pose.orientation.w = 0.0
        
        
    try:
        group = MoveGroupCommander("left_arm")
        # Setting position and orientation target
        group.set_pose_target(request.ik_request.pose_stamped)

        print("Trying to move the left arm out of the way!")
        # Plan IK and execute
        group.go()

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


    #Move to human hand
    raw_input('Press [Enter]:')

    if robot_hand == 'right':
        #Construct the request for right hand
        request = GetPositionIKRequest()
        request.ik_request.group_name = "right_arm"
        request.ik_request.ik_link_name = "right_gripper"
        request.ik_request.attempts = 500
        request.ik_request.pose_stamped.header.frame_id = "base"
        if human_hand == 'right':
            #Set the desired orientation for the end effector
            request.ik_request.pose_stamped.pose.position.x = 0.743
            request.ik_request.pose_stamped.pose.position.y = 0.0677
            request.ik_request.pose_stamped.pose.position.z = 0.045
            request.ik_request.pose_stamped.pose.orientation.x = 0.0
            request.ik_request.pose_stamped.pose.orientation.y = 1.0
            request.ik_request.pose_stamped.pose.orientation.z = 0.0
            request.ik_request.pose_stamped.pose.orientation.w = 0.0
        elif human_hand == 'left':
            #Set the desired orientation for the end effector
            request.ik_request.pose_stamped.pose.position.x = 0.745
            request.ik_request.pose_stamped.pose.position.y = -0.055
            request.ik_request.pose_stamped.pose.position.z = -0.137
            request.ik_request.pose_stamped.pose.orientation.x = 0.0
            request.ik_request.pose_stamped.pose.orientation.y = 1.0
            request.ik_request.pose_stamped.pose.orientation.z = 0.0
            request.ik_request.pose_stamped.pose.orientation.w = 0.0

    elif robot_hand == 'left':
        #Construct the request for left hand
        request = GetPositionIKRequest()
        request.ik_request.group_name = "left_arm"
        request.ik_request.ik_link_name = "left_gripper"
        request.ik_request.attempts = 500
        request.ik_request.pose_stamped.header.frame_id = "base"

        if human_hand == 'right':
            #Set the desired orientation for the end effector
            request.ik_request.pose_stamped.pose.position.x = 0.740
            request.ik_request.pose_stamped.pose.position.y = 0.319
            request.ik_request.pose_stamped.pose.position.z = -0.137
            request.ik_request.pose_stamped.pose.orientation.x = 0.0
            request.ik_request.pose_stamped.pose.orientation.y = 1.0
            request.ik_request.pose_stamped.pose.orientation.z = 0.0
            request.ik_request.pose_stamped.pose.orientation.w = 0.0
        elif human_hand == 'left':
            #Set the desired orientation for the end effector
            request.ik_request.pose_stamped.pose.position.x = 0.735
            request.ik_request.pose_stamped.pose.position.y = -0.037
            request.ik_request.pose_stamped.pose.position.z = -0.137
            request.ik_request.pose_stamped.pose.orientation.x = 0.0
            request.ik_request.pose_stamped.pose.orientation.y = 1.0
            request.ik_request.pose_stamped.pose.orientation.z = 0.0
            request.ik_request.pose_stamped.pose.orientation.w = 0.0

    try:
        if robot_hand == 'left':
            group = MoveGroupCommander("left_arm")

        elif robot_hand == 'right':
            group = MoveGroupCommander("right_arm")

        # Setting position and orientation target
        group.set_pose_target(request.ik_request.pose_stamped)

        if robot_hand == "left":
            print("Trying to move the left arm to human hand!")
        else:
            print("Trying to move the right arm to human hand!")
        # Plan IK and execute
        group.go()
            
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


    #Move back to neutral/camera position
    
    #Move right hand to neutral position
    if robot_hand == 'right':
        raw_input('Press [Enter]:')
        #Construct the request for right hand
        request = GetPositionIKRequest()
        request.ik_request.group_name = "right_arm"
        request.ik_request.ik_link_name = "right_gripper"
        request.ik_request.attempts = 500
        request.ik_request.pose_stamped.header.frame_id = "base"
            
        #Set the desired orientation for the end effector
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
            print("Trying to move the right arm back to neutral position!")
            # Plan IK and execute
            group.go()

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        
        #Move left hand to neutral position
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
            print("Trying to move the left arm back to neutral position!")
            # Plan IK and execute
            group.go()

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
    else:
        #Move left hand to neutral position
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
            print("Trying to move the left arm back to neutral position!")
            # Plan IK and execute
            group.go()

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        raw_input('Press [Enter]:')
        #Construct the request for right hand
        request = GetPositionIKRequest()
        request.ik_request.group_name = "right_arm"
        request.ik_request.ik_link_name = "right_gripper"
        request.ik_request.attempts = 500
        request.ik_request.pose_stamped.header.frame_id = "base"
            
        #Set the desired orientation for the end effector
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
            print("Trying to move the right arm back to neutral position!")
            # Plan IK and execute
            group.go()

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
                  
    # left_gripper.open()
    right_gripper.open()