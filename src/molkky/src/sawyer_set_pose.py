#!/usr/bin/env python
"""
Path Planning Script for Lab 7
Author: Valmik Prabhu
"""

import sys
# assert sys.argv[1] in ("sawyer", "baxter")
# ROBOT = sys.argv[1]

# if ROBOT == "baxter":
#     from baxter_interface import Limb
# else:
from intera_interface import Limb

import rospy
import numpy as np
import traceback

from moveit_msgs.msg import OrientationConstraint
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3

from path_planner import PathPlanner


# At time 1575926809.076
# - Translation: [0.467, -0.433, 0.256]
# - Rotation: in Quaternion [0.921, 0.020, 0.355, 0.157]
#             in RPY (radian) [2.732, -0.705, 0.197]
#             in RPY (degree) [156.531, -40.390, 11.270]


# At time 1575926891.256
# - Translation: [0.598, 0.227, 0.114]
# - Rotation: in Quaternion [0.920, -0.067, 0.383, 0.053]
#             in RPY (radian) [3.077, -0.791, -0.118]
#             in RPY (degree) [176.290, -45.333, -6.758]


# def set_pose(goal_vec):
#     planner = PathPlanner("right_arm") # MoveIt path planning class

#     # we can add obstacles if we choose to do so

#     ## Create a path constraint for the arm
#     orien_const = OrientationConstraint()
#     orien_const.link_name = "right_hand";
#     orien_const.header.frame_id = "base";
#     # orien_const.orientation.x = 0.92;
#     orien_const.orientation.y = -1;
#     # orien_const.orientation.y = -1;
#     # orien_const.orientation.z = 0.35; 
#     # orien_const.orientation.w = 0.1; 
#     orien_const.absolute_x_axis_tolerance = 0.1;
#     orien_const.absolute_y_axis_tolerance = 0.1;
#     orien_const.absolute_z_axis_tolerance = 0.1;
#     orien_const.weight = 1.0;

#     while not rospy.is_shutdown():
#         try:
#             goal = PoseStamped()
#             goal.header.frame_id = "base"

#             #x, y, and z position
#             goal.pose.position.x = 0.6 # save this one: 0.6
#             goal.pose.position.y = goal_vec.y
#             goal.pose.position.z = -.2 # save this one: -.2
#             # goal.pose.position.x = 0.5
#             # goal.pose.position.y = goal_vec.y
#             # goal.pose.position.z = -0.2

#             #Orientation as a quaternion
#             goal.pose.orientation.x = 0
#             goal.pose.orientation.y = -1
#             goal.pose.orientation.z = 0
#             goal.pose.orientation.w = 0
#             # goal.pose.orientation.x = 0.92
#             # goal.pose.orientation.y = 0.0
#             # goal.pose.orientation.z = 0.35
#             # goal.pose.orientation.w = 0.1

#             # plan = planner.plan_to_pose(goal, list()) # no constraints
#             plan = planner.plan_to_pose(goal, [orien_const])

#             if not planner.execute_plan(plan):
#                 raise Exception("Execution failed")
#         except Exception as e:
#             print e
#         else:
#             break

from path_planner import set_pose
if __name__ == '__main__':
    rospy.init_node('moveit_node')

    ## TODO : determine goal_positions from computer vision code
    # y_goal = float(raw_input("Enter desired y position: \n"))
    y_per = float(raw_input("Enter desired y percentage (0, 1): \n"))

    # interpolate percentage between -0.5 and 0.3
    min_lim = -0.2
    max_lim = 0.5
    y_des = min_lim + y_per * (max_lim - min_lim)

    # set_pose(Vector3(0.0, y_goal, 0.0))
    set_pose(Vector3(0.0, y_des, 0.0))

# from game import Game

# def play_molkky(num_players, robot_turn):
#     rospy.init_node('molkky')
#     game = Game(num_players, robot_turn)

#     r = rospy.Rate(1000)
#     print("Start game. Num_players: " + str(game.state.num_players)
#           + ". Robot_turn: " + str(game.state.robot_turn))

#     while (game.state.is_game_over() == -1 and not rospy.is_shutdown()):
#         game.take_turn()

#     winner = game.state.is_game_over()
#     if winner == -1:
#         print("There is no winner")
#     else:
#         print("Player " + str(winner) + " wins")
#     print("Scores:", game.state.get_scores())

# if __name__ == "__main__":
#     # raw_input("Enter once throw is done")
#     num_players = 2
#     robot_turn = 1
#     if len(sys.argv) > 2:
#         num_players = int(sys.argv[1])
#         robot_turn = int(sys.argv[2])

#     play_molkky(num_players, robot_turn)
