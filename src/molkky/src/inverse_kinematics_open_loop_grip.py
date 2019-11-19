#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
import numpy as np
from numpy import linalg
import sys
from baxter_interface import gripper as robot_gripper

def main(robo):
    #Wait for the IK service to become available
    rospy.wait_for_service('compute_ik')
    rospy.init_node('service_query')
    arm = 'left'
    #Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    if robo == 'sawyer':
    	arm = 'right'
    while not rospy.is_shutdown():
        raw_input('Press [ Enter ]: ')
        
        #Construct the request
        request = GetPositionIKRequest()
        request.ik_request.group_name = arm + "_arm"

        #Alan does not have a gripper so replace link with 'right_wrist' instead
        link = arm + "_gripper"
        if robo == 'sawyer':
        	link += '_tip'

        request.ik_request.ik_link_name = link
        request.ik_request.attempts = 20
        request.ik_request.pose_stamped.header.frame_id = "base"
        
        #Set the desired orientation for the end effector HERE
        request.ik_request.pose_stamped.pose.position.x = 0.771 
        request.ik_request.pose_stamped.pose.position.y = 0.616
        request.ik_request.pose_stamped.pose.position.z = -0.24      
        request.ik_request.pose_stamped.pose.orientation.x = 0.020
        request.ik_request.pose_stamped.pose.orientation.y = 0.998
        request.ik_request.pose_stamped.pose.orientation.z = -0.020
        request.ik_request.pose_stamped.pose.orientation.w = 0.060
        left_gripper = robot_gripper.Gripper('left')
        
        try:
            #Send the request to the service
            response = compute_ik(request)
            
            #Print the response HERE
            print(response)
            group = MoveGroupCommander(arm + "_arm")

            left_gripper.calibrate()

            left_gripper.open()

            rospy.sleep(1)
            # Setting position and orientation target
            group.set_pose_target(request.ik_request.pose_stamped)

            # TRY THIS
            # Setting just the position without specifying the orientation

            # group.set_position_target([0.771, 0.616, -0.261])

            # Plan IK and execute
            group.go()
            # rospy.sleep(1)

            # group.set_position_target([0.763, 0.622, -0.143])

            # # Plan IK and execute
            # group.go()

            # rospy.sleep(1)

            left_gripper.close()

            rospy.sleep(1)
            # group.set_position_target([0.771, 0.616, -0.261])

            # Plan IK and execute
            # group.go()
            
            
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

#Python's syntax for a main() method
if __name__ == '__main__':
    main(sys.argv[1])

