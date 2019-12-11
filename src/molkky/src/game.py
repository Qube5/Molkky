import numpy as np
import rospy
import sys
import serial
import time

from game_state import GameState
from constants import *
from segmentation.msg import ImageInfo
# from path_planner import set_pose
from path_planner import PathPlanner
from geometry_msgs.msg import Vector3
from moveit_msgs.msg import OrientationConstraint, Constraints, CollisionObject
import math

def numpy_to_move_msg(move):
    return ros_numpy.msgify(String, move)

class Game:
    def __init__(self, num_players, robot_turn):
        self.num_players = num_players
        self.scores = [0] * num_players
        self.robot_turn = robot_turn
        self.turn = 0
        self.state = GameState(num_players, robot_turn, initial_board_state)

        self.current_image_info = None

        self.planner = PathPlanner("right_arm") # MoveIt path planning class

        # #Change port/baud rate later
        # self.arduino = serial.Serial('COM1', 9600, timeout=.1)
        # time.sleep(1) #give the connection a second to settle
        # self.arduino.write("Hello from Python!")

        self.current_pos = 0
        self.set_initial_pose(self.current_pos)

        rospy.Subscriber('image_info', ImageInfo, self.info_callback)

    def info_callback(self, image_info):
        try:
            info = []
            for i in range(len(image_info.cluster_g)):
                info.append([
                    image_info.cluster_r[i],
                    image_info.cluster_g[i],
                    image_info.cluster_b[i],
                    image_info.cluster_x[i],
                    image_info.cluster_y[i]])
        except Exception as e:
            rospy.logerr(e)
            return
        self.current_image_info = info

    def take_turn(self):
        print("Current Score: " + str(self.state.get_scores()))

        if not self.current_image_info:
            print("not getting image info")
            return

        print("Getting pin state")
        board_state = self.get_new_board_state()
        self.state.update_board_state(board_state)

        # Turn starts with strategy then a throw
        if (self.state.turn == self.state.robot_turn):
            print("Starting Baxter's Turn")
            print("Analyze board to get the best move")
            # using the previous board state to figure out the best move
            next_move = self.get_best_expected_move(board_state)
            print("Next move acquired (x position)", next_move)

            # Send move to make move
            print("Move baxter to position")
            self.baxter_make_move(next_move)

            print("Baxter is moving to position")
            input("Enter once Baxter is in position")
            # raw_input("Enter once Baxter is in position")

            # Baxter throw using pneumatic launcher
            print("Baxter throw")
            self.baxter_throw()
        else:
            print("Player " + str(self.state.turn) + "'s turn")
            print("Player " + str(self.state.turn) + ". Throw")
            # input("Enter once throw is done")
            raw_input("Enter once throw is done")
            # raw_input("Enter desired y percentage (0, 1): \n")

        print("Please set pins upright after the throw")
        raw_input("Enter once pins are righted")

        points_gained = self.score_board()

        print("Updating state with new score")
        self.state.increment_score(points_gained)
        print("Score updated")

        self.state.next_turn()
        self.prev_board_state = board_state

    def score_board(self):
        score = -1
        while score == -1:
            score = raw_input("Enter number of points earned ")
            try:
                score = int(score)
                if score >= 0 and score <= max(largest_pin_value, len(color_dict)):
                    return score
                else:
                    print("Invalid number of points. Please try again")
                    score = -1
            except:
                print("Invalid number of points. Please try again")
                score = -1

        return score

    def get_new_board_state(self):
        if not self.current_image_info:
            return None

        board_state = self.current_image_info
        return board_state

    def get_best_expected_move(self, board_state):

        ## Get best pin to knock down
        current_score = self.scores[self.robot_turn]
        optimal_points = winning_score - current_score

        if largest_pin_value <= optimal_points:
            best_pin_points = largest_pin_value
        else:
            best_pin_points = optimal_points

        key_list = list(color_points.keys())
        val_list = list(color_points.values())
        best_pin_color = key_list[val_list.index(best_pin_points)]
        best_pin_rgb = np.array(color_dict[best_pin_color])

        ## Get index of best pin
        closest_pin_index = 0
        closest_pin_distance = 1000000000
        for i in range(len(board_state)):
            pin_rgb = np.array(board_state[i][:3])
            pin_rgb_distance = np.linalg.norm(pin_rgb - best_pin_rgb)
            if pin_rgb_distance < closest_pin_distance:
                closest_pin_distance = pin_rgb_distance
                closest_pin_index = i

        closest_pin_loc = board_state[i][3:]
        closest_pin_x = closest_pin_loc[0]

        return closest_pin_x

    def get_best_expected_move(self, pins):
        current_score = self.scores[self.robot_turn]
        max_possible = 0
        max_desired = winning_score - current_score
        best_pin_idx = 0
        for i in range(len(pins)):
            curr_pin_value = get_pin_value(pins[i][3:])
            num_near_pins = num_pins_near_pin(i, pins)
            if num_near_pins > 1:
                curr_pin_value = 0
            if curr_pin_value > max_possible or num_near_pins > max_possible:
                if curr_pin_value <= max_desired or num_near_pins <= max_desired:
                    best_pin_idx = i
                    max_possible = max(curr_pin_value, num_near_pins)

        return pins[best_pin_idx][0]

    def get_pin_value(self, rgb):
        rgb = np.array(rgb)
        closest_pin_color = ""
        closest_pin_distance = 100000000000000
        for color in color_dict.keys():
            pin_rgb = np.array(color_dict[color])
            pin_rgb_distance = np.linalg.norm(pin_rgb - rgb)
            if pin_rgb_distance < closest_pin_distance:
                closest_pin_distance = pin_rgb_distance
                closest_pin_color = color

        pin_value = color_points[closest_pin_color]
        return pin_value

    def num_pins_near_pin(self, pin_idx, pins):
        x = pins[pin_idx][0]
        buffer = 30
        num_pins = 0
        for px in range(x - buffer, x + buffer):
            for p in range(len(pins))
                if p != pin_idx:
                    if pins[p][0] == px:
                        num_pins = num_pins + 1

        return num_pins

    def baxter_make_move(self, move):
        pixel_x = move
        x_ratio = (pixel_x - max_x_pixel_left) / (max_x_pixel_right - max_x_pixel_left)

        y_per = x_ratio

        # # interpolate percentage between -0.5 and 0.3
        min_lim = max_sawyer_left
        max_lim = max_sawyer_right

        y_des = min_lim + y_per * (max_lim - min_lim)
        # set_pose(Vector3(0.0, y_goal, 0.0))
        # self.set_pose(Vector3(0.0, y_des, 0.0))
        self.set_pose_incrementally(Vector3(0.0, y_des, 0.0), self.current_pos, 3)
        self.current_pos = y_des
        # TODO BIANCA
        return None

    def set_pose_incrementally(goal_vec, current_pos, num_steps):
        new_pos = goal_vec.y
        for inc in np.linspace(current_pos, new_pos, num=num_steps, endpoint=True):
            y_des = current_pos + inc
            self.set_pose(Vector3(0.0, y_des, 0.0))

    def set_initial_pose(initial_y = 0):
        self.set_pose(Vector3(0.0, initial_y, 0.0), False)

    def add_obstacle(size, x, y, z):
        return

    def remove_obstacle():
        self.planner.remove_obstacle("table")

    def get_constraints(add_constraints = False):
        constraints = []
        if add_constraints:
            ## Create a path constraint for the arm
            orien_const = OrientationConstraint()
            orien_const.link_name = "right_hand";
            orien_const.header.frame_id = "base";
            # orien_const.orientation.x = 0.92;
            orien_const.orientation.y = -1;
            # orien_const.orientation.y = -1;
            # orien_const.orientation.z = 0.35;
            # orien_const.orientation.w = 0.1;
            orien_const.absolute_x_axis_tolerance = math.pi / 4;
            orien_const.absolute_y_axis_tolerance = math.pi / 4;
            orien_const.absolute_z_axis_tolerance = 2 * math.pi;
            # orien_const.absolute_x_axis_tolerance = 0.1;
            # orien_const.absolute_y_axis_tolerance = 0.1;
            # orien_const.absolute_z_axis_tolerance = 0.1;
            orien_const.weight = 1.0;
            constraints.append(orien_const)
        return constraints

    def set_pose(goal_vec, obstacle = True):

        # we can add obstacles if we choose to do so
        if obstacle:
            self.add_obstacle([], 0, 0, 0)
        else:
            self.remove_obstacle()

        constraints = self.get_constraints(False)

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

                # # This time WITH MATH!
                # theta = -math.pi/8
                # a = math.sqrt((1 - (math.cos(theta))^2) / (2 * math.sin(theta)^2))
                # goal.pose.orientation.x = a * math.sin(theta)
                # goal.pose.orientation.y = 0
                # goal.pose.orientation.z = a * math.sin(theta)
                # goal.pose.orientation.w = math.cos(theta)

                # # This time to have everything constant
                # goal.pose.orientation.x = 0
                # goal.pose.orientation.y = 0
                # goal.pose.orientation.z = 0
                # goal.pose.orientation.w = 1

                # plan = planner.plan_to_pose(goal, list()) # no constraints
                # plan = self.planner.plan_to_pose(goal, [orien_const])
                plan = self.planner.plan_to_pose(goal, constraints)

                if not self.planner.execute_plan(plan):
                    raise Exception("Execution failed")
            except Exception as e:
                print e
            else:
                break

    def baxter_throw(self):
        raw_input("Press enter for Baxter to throw")
        print("Throwing")
        # for i in range(25):
        #     self.arduino.write("throw".encode())
        # self.arduino.write("".encode())
        return True
