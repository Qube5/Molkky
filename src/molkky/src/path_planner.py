#!/usr/bin/env python
"""
Path Planner Class for Lab 7
Author: Valmik Prabhu
"""

import sys
import rospy
import moveit_commander
from moveit_msgs.msg import OrientationConstraint, Constraints, CollisionObject
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3

from shape_msgs.msg import SolidPrimitive
assert sys.argv[1] in ("sawyer", "baxter")
ROBOT = sys.argv[1]

if ROBOT == "baxter":
    from baxter_interface import Limb
else:
    from intera_interface import Limb
# from sawyer_set_pose import *

class PathPlanner(object):
    """
    Path Planning Functionality for Baxter/Sawyer

    We make this a class rather than a script because it bundles up
    all the code relating to planning in a nice way thus, we can
    easily use the code in different places. This is a staple of
    good object-oriented programming

    Fields:
    _robot: moveit_commander.RobotCommander; for interfacing with the robot
    _scene: moveit_commander.PlanningSceneInterface; the planning scene stores a representation of the environment
    _group: moveit_commander.MoveGroupCommander; the move group is moveit's primary planning class
    _planning_scene_publisher: ros publisher; publishes to the planning scene
    _goal_position_subscriber: ros subscriber; subcribes to the goal position


    """
    def __init__(self, group_name):
        """
        Constructor.

        Inputs:
        group_name: the name of the move_group.
            For Baxter, this would be 'left_arm' or 'right_arm'
            For Sawyer, this would be 'right_arm'
        """

        # If the node is shutdown, call this function
        rospy.on_shutdown(self.shutdown)
        # print(sys.argv)
        # Initialize moveit_commander
        moveit_commander.roscpp_initialize(sys.argv)

        # Initialize the robot
        self._robot = moveit_commander.RobotCommander()

        # Initialize the planning scene
        self._scene = moveit_commander.PlanningSceneInterface()

        # This publishes updates to the planning scene
        # self._planning_scene_publisher = rospy.Publisher('/collision_object', CollisionObject, queue_size=10)

        # # This subscribes to the desired x position from the computer vision
        # self._goal_position_subscriber = rospy.Subscriber('/goal_position', Vector3, set_pose)

        # Instantiate a move group
        self._group = moveit_commander.MoveGroupCommander(group_name)

        # Set the maximum time MoveIt will try to plan before giving up
        self._group.set_planning_time(5)

        # Set the bounds of the workspace
        self._group.set_workspace([-2, -2, -2, 2, 2, 2])

        # Sleep for a bit to ensure that all inititialization has finished
        rospy.sleep(0.5)

    def shutdown(self):
        """
        Code to run on shutdown. This is good practice for safety

        Currently deletes the object's MoveGroup, so that further commands will do nothing
        """
        self._group = None
        rospy.loginfo("Stopping Path Planner")

    def plan_to_pose(self, target, orientation_constraints):
        """
        Generates a plan given an end effector pose subject to orientation constraints

        Inputs:
        target: A geometry_msgs/PoseStamped message containing the end effector pose goal
        orientation_constraints: A list of moveit_msgs/OrientationConstraint messages

        Outputs:
        path: A moveit_msgs/RobotTrajectory path
        """

        self._group.set_pose_target(target)
        self._group.set_start_state_to_current_state()

        constraints = Constraints()
        constraints.orientation_constraints = orientation_constraints
        self._group.set_path_constraints(constraints)

        plan = self._group.plan()

        return plan

    def execute_plan(self, plan):
        """
        Uses the robot's built-in controllers to execute a plan

        Inputs:
        plan: a moveit_msgs/RobotTrajectory plan
        """

        return self._group.execute(plan, True)


    def add_box_obstacle(self, size, name, pose):
        """
        Adds a rectangular prism obstacle to the planning scene

        Inputs:
        size: 3x' ndarray; (x, y, z) size of the box (in the box's body frame)
        name: unique name of the obstacle (used for adding and removing)
        pose: geometry_msgs/PoseStamped object for the CoM of the box in relation to some frame
        """

        # Create a CollisionObject, which will be added to the planning scene
        co = CollisionObject()
        co.operation = CollisionObject.ADD
        co.id = name
        co.header = pose.header

        # Create a box primitive, which will be inside the CollisionObject
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = size

        # Fill the collision object with primitive(s)
        co.primitives = [box]
        co.primitive_poses = [pose.pose]

        # Publish the object
        self._planning_scene_publisher.publish(co)

    def remove_obstacle(self, name):
        """
        Removes an obstacle from the planning scene

        Inputs:
        name: unique name of the obstacle
        """

        co = CollisionObject()
        co.operation = CollisionObject.REMOVE
        co.id = name

        self._planning_scene_publisher.publish(co)

def set_pose(goal_vec):
    planner = PathPlanner("right_arm") # MoveIt path planning class

    # we can add obstacles if we choose to do so

    ## Create a path constraint for the arm
    orien_const = OrientationConstraint()
    orien_const.link_name = "right_hand";
    orien_const.header.frame_id = "base";
    # orien_const.orientation.x = 0.92;
    orien_const.orientation.y = -1;
    # orien_const.orientation.y = -1;
    # orien_const.orientation.z = 0.35; 
    # orien_const.orientation.w = 0.1; 
    orien_const.absolute_x_axis_tolerance = 0.1;
    orien_const.absolute_y_axis_tolerance = 0.1;
    orien_const.absolute_z_axis_tolerance = 0.1;
    orien_const.weight = 1.0;

    while not rospy.is_shutdown():
        try:
            goal = PoseStamped()
            goal.header.frame_id = "base"

            #x, y, and z position
            goal.pose.position.x = 0.6 # save this one: 0.6
            goal.pose.position.y = goal_vec.y
            goal.pose.position.z = -.2 # save this one: -.2
            # goal.pose.position.x = 0.5
            # goal.pose.position.y = goal_vec.y
            # goal.pose.position.z = -0.2

            #Orientation as a quaternion
            goal.pose.orientation.x = 0
            goal.pose.orientation.y = -1
            goal.pose.orientation.z = 0
            goal.pose.orientation.w = 0
            # goal.pose.orientation.x = 0.92
            # goal.pose.orientation.y = 0.0
            # goal.pose.orientation.z = 0.35
            # goal.pose.orientation.w = 0.1

            # plan = planner.plan_to_pose(goal, list()) # no constraints
            plan = planner.plan_to_pose(goal, [orien_const])

            if not planner.execute_plan(plan):
                raise Exception("Execution failed")
        except Exception as e:
            print e
        else:
            break

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