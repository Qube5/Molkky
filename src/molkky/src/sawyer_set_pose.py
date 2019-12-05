#!/usr/bin/env python
"""
Path Planning Script for Lab 7
Author: Valmik Prabhu
"""

import sys
assert sys.argv[1] in ("sawyer", "baxter")
ROBOT = sys.argv[1]

if ROBOT == "baxter":
    from baxter_interface import Limb
else:
    from intera_interface import Limb

import rospy
import numpy as np
import traceback

from moveit_msgs.msg import OrientationConstraint
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3

from path_planner import PathPlanner

def set_pose(goal_vec):
    planner = PathPlanner("right_arm") # MoveIt path planning class

    # we can add obstacles if we choose to do so

    ## Create a path constraint for the arm
    orien_const = OrientationConstraint()
    orien_const.link_name = "right_gripper";
    orien_const.header.frame_id = "base";
    orien_const.orientation.y = -1.0; # choose orientation constraint during path
    orien_const.absolute_x_axis_tolerance = 0.1;
    orien_const.absolute_y_axis_tolerance = 0.1;
    orien_const.absolute_z_axis_tolerance = 0.1;
    orien_const.weight = 1.0;

    while not rospy.is_shutdown():
        try:
            goal = PoseStamped()
            goal.header.frame_id = "base"

            #x, y, and z position
            goal.pose.position.x = goal_vec.x
            goal.pose.position.y = 0.05
            goal.pose.position.z = -0.23

            #Orientation as a quaternion
            goal.pose.orientation.x = 0.0
            goal.pose.orientation.y = -1.0
            goal.pose.orientation.z = 0.0
            goal.pose.orientation.w = 0.0

            # plan = planner.plan_to_pose(goal_1, list()) # no constraints
            plan = planner.plan_to_pose(goal_1, [orien_const])

            if not planner.execute_plan(plan):
                raise Exception("Execution failed")
        except Exception as e:
            print e
        else:
            break

if __name__ == '__main__':
    rospy.init_node('moveit_node')

    ## TODO : determine goal_positions from computer vision code
    x_goal = float(raw_input("Enter desired x position: \n"))

    set_pose(Vector3(x_goal, 0.0, 0.0))
