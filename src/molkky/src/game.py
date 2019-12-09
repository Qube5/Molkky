import numpy as np
import rospy
import sys

from game_state import GameState
from constants import *
from segmentation.msg import ImageInfo
from path_planner import set_pose
from geometry_msgs.msg import Vector3


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

        rospy.Subscriber('image_info', ImageInfo, self.info_callback)

    def info_callback(self, image_info):
        try:
            info = []
            for i in range(len(image_info.cluster_g)):
                info.append([image_info.cluster_r[i],
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

            print("Baxter is moved to position")
            input("Enter once Baxter is in position")
            # raw_input("Enter once Baxter is in position")

            # Baxter throw using pneumatic launcher
            print("Baxter throw")
            self.baxter_throw()
        else:
            print("Player " + str(self.state.turn) + "'s turn")
            print("Player " + str(self.state.turn) + ". Throw")
            # input("Enter once throw is done")
            a = raw_input("Enter once throw is done")
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

    def baxter_make_move(self, move):
        pixel_x = move
        x_ratio = (pixel_x - max_x_pixel_left) / (max_x_pixel_right - max_x_pixel_left)
        
        ## CONVERT move (x coordinate) to robot-x-axis coordinate
        print("MOVE", move, x_ratio)
        # print(move)

        # rospy.init_node('moveit_node')
        # raw_input("Enter once throw is done")

        # ## TODO : determine goal_positions from computer vision code
        # # y_goal = float(raw_input("Enter desired y position: \n"))
        y_per = x_ratio

        # # interpolate percentage between -0.5 and 0.3
        min_lim = -0.2
        max_lim = 0.5
        y_des = min_lim + y_per * (max_lim - min_lim)
        # set_pose(Vector3(0.0, y_goal, 0.0))
        set_pose(Vector3(0.0, y_des, 0.0))
        ## CONVERT robot-x-axis coordinate to robot move vector

        # move_msg = numpy_to_move_msg(move)
        # self.move_pub.publish(move_msg)
        # self.move_pub.publish(move)
        # TODO BIANCA
        return None

    def baxter_throw(self):
        raw_input("Press enter for Baxter to throw")
        print("THROW")
        # TODO WILLIAM
        return True
