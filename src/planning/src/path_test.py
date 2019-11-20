#!/usr/bin/env python
"""
Path Planning Script for Lab 7
Author: Valmik Prabhu
"""

import sys
import rospy
import numpy as np

from moveit_msgs.msg import OrientationConstraint
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header

from path_planner import PathPlanner
from controller import Controller
from baxter_interface import Limb
# from intera_interface import Limb

def main():
    """
    Main Script
    """

    # Make sure that you've looked at and understand path_planner.py before starting

    planner = PathPlanner("right_arm")

    Kp = 0.2 * np.array([0.4, 2, 1.7, 1.5, 2, 2, 3])
    Kd = 0.01 * np.array([2, 1, 2, 0.5, 0.8, 0.8, 0.8])
    Ki = 0.01 * np.array([1.4, 1.4, 1.4, 1, 0.6, 0.6, 0.6])
    Kw = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9])
    controller = Controller(Kp, Ki, Kd, Kw, Limb("right"))
# 
    ##
    # ## Add the obstacle to the planning scene here
    # table_size = np.array([0.40, 1.20, 0.10])
    # table_name = "table"

    # table_pose = PoseStamped()
    # table_pose.header.frame_id = "base"

    # #x, y, and z position
    # table_pose.pose.position.x = 0.5
    # table_pose.pose.position.y = 0.0
    # table_pose.pose.position.z =  0.0

    # #Orientation as a quaternion
    # table_pose.pose.orientation.x = 0.0
    # table_pose.pose.orientation.y = 0.0
    # table_pose.pose.orientation.z = 0.0
    # table_pose.pose.orientation.w = 1.0
    # planner.add_box_obstacle(table_size, table_name, table_pose)    

    # wall_size = np.array([1.0, 0.1, 1.0])
    # wall_name = "wall"
    # wall_pose = PoseStamped()
    # wall_pose.header.frame_id = "base"

    # #x, y, and z position
    # wall_pose.pose.position.x = 0.2
    # wall_pose.pose.position.y = -0.7
    # wall_pose.pose.position.z =  -.2

    # #Orientation as a quaternion
    # wall_pose.pose.orientation.x = 0.0
    # wall_pose.pose.orientation.y = 0.0
    # wall_pose.pose.orientation.z = 0.0
    # wall_pose.pose.orientation.w = 1.0
    # planner.add_box_obstacle(wall_size, wall_name, wall_pose)
    # #

    # #Create a path constraint for the arm
    # #UNCOMMENT FOR THE ORIENTATION CONSTRAINTS PART
    # orien_const = OrientationConstraint()
    # orien_const.link_name = "right_gripper";
    # orien_const.header.frame_id = "base";
    # orien_const.orientation.y = -1.0;
    # orien_const.absolute_x_axis_tolerance = 0.1;
    # orien_const.absolute_y_axis_tolerance = 0.1;
    # orien_const.absolute_z_axis_tolerance = 0.1;
    # orien_const.weight = 1.0;
    orientation_constraints = []

    while not rospy.is_shutdown():

        while not rospy.is_shutdown():
            try:
                goal_1 = PoseStamped()
                goal_1.header.frame_id = "base"

                #x, y, and z position
                # goal_1.pose.position.x = 0.8
                # goal_1.pose.position.y = 0.05
                # goal_1.pose.position.z = -0.23                
                goal_1.pose.position.x = 0.6
                goal_1.pose.position.y = -0.1
                goal_1.pose.position.z = 0.12

                #Orientation as a quaternion
                goal_1.pose.orientation.x = 0.0
                goal_1.pose.orientation.y = -1.0
                goal_1.pose.orientation.z = 0.0
                goal_1.pose.orientation.w = 0.0

                # Might have to edit this . . . 
                # plan = planner.plan_to_pose(goal_1, list())
                plan = planner.plan_to_pose(goal_1, orientation_constraints)
                # print("P",plan)
                raw_input("Press <Enter> to move the right arm to goal pose 1: ")
                # if not planner.execute_plan(plan):
                if not controller.execute_path(plan):
                    raise Exception("Execution failed")
            except Exception as e:
                print e
            else:
                break

        while not rospy.is_shutdown():
            try:
                goal_2 = PoseStamped()
                goal_2.header.frame_id = "base"

                #x, y, and z position
                goal_2.pose.position.x = 0.6
                goal_2.pose.position.y = -0.2
                goal_2.pose.position.z = 0.5
                # goal_2.pose.position.x = 0.6
                # goal_2.pose.position.y = -0.3
                # goal_2.pose.position.z = 0.0

                #Orientation as a quaternion
                goal_2.pose.orientation.x = 0.0
                goal_2.pose.orientation.y = -1.0
                goal_2.pose.orientation.z = 0.0
                goal_2.pose.orientation.w = 0.0

                # plan = planner.plan_to_pose(goal_2, list())
                plan = planner.plan_to_pose(goal_2, orientation_constraints)

                raw_input("Press <Enter> to move the right arm to goal pose 2: ")
                # if not planner.execute_plan(plan):
                if not controller.execute_path(plan):
                    raise Exception("Execution failed")
            except Exception as e:
                print e
            else:
                break

        while not rospy.is_shutdown():
            try:
                goal_3 = PoseStamped()
                goal_3.header.frame_id = "base"

                #x, y, and z position
                goal_3.pose.position.x = 0.6
                goal_3.pose.position.y = -0.1
                goal_3.pose.position.z = 0.1

                #Orientation as a quaternion
                goal_3.pose.orientation.x = 0.0
                goal_3.pose.orientation.y = -1.0
                goal_3.pose.orientation.z = 0.0
                goal_3.pose.orientation.w = 0.0

                # plan = planner.plan_to_pose(goal_3, list())
                plan = planner.plan_to_pose(goal_3, orientation_constraints)

                raw_input("Press <Enter> to move the right arm to goal pose 3: ")
                # if not planner.execute_plan(plan):
                if not controller.execute_path(plan):
                    raise Exception("Execution failed")
            except Exception as e:
                print e
            else:
                break

if __name__ == '__main__':
    rospy.init_node('moveit_node')
    main()
