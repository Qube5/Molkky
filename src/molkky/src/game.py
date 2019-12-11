import numpy as np
import rospy
import sys

from game_state import GameState
from constants import *
from segmentation.msg import ImageInfo
from path_planner import PathPlanner
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

        self.planner = PathPlanner("right_arm") # MoveIt path planning class

        # # Change port/baud rate later
        # self.arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=.1)
        # time.sleep(1) #give the connection a second to settle
        # self.arduino.write("Hello from Python!")

        self.current_pos = 0
        raw_input("Enter to set initial position ")
        self.set_initial_pose(self.current_pos)

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

    def get_current_image_info(self):
        print("Waiting for current image info...")
        while not self.current_image_info:
            print("wait")
        return self.current_image_info

    def take_turn(self):
        print("\nCurrent Score: " + str(self.state.get_scores()))

        if not self.current_image_info:
            print("not getting image info")
            return


        print("Getting pin state")
        board_state = self.get_new_board_state()
        self.state.update_board_state(board_state)

        # Turn starts with strategy then a throw
        if (self.state.turn == self.state.robot_turn):
            print("Starting Sawyer's Turn")
            print("Analyze board to get the best move")
            # using the previous board state to figure out the best move
            next_move = self.get_best_expected_move(board_state)
            print("Next move acquired (x position)", next_move)

            # Send move to make move
            raw_input("Enter for Sawyer to move to position ")
            self.sawyer_make_move(next_move)

            print("Sawyer is moving to position")
            raw_input("Enter once Sawyer is in position ")

            # Sawyer throw using pneumatic launcher
            print("Sawyer throw")
            self.sawyer_throw()
        else:
            print("Player " + str(self.state.turn) + "'s turn")
            print("Player " + str(self.state.turn) + ". Throw")
            # input("Enter once throw is done")
            a = raw_input("Enter once throw is done")
            # raw_input("Enter desired y percentage (0, 1): \n")

        print("Please set pins upright after the throw")
        raw_input("Enter once pins are righted ")

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

<<<<<<< HEAD
    def baxter_make_move(self, move):
=======
    def get_best_expected_move(self, pins):
        current_score = self.scores[self.robot_turn]
        max_possible = 0
        max_desired = winning_score - current_score
        best_pin_idx = 0
        for i in range(len(pins)):
            curr_pin_value = self.get_pin_value(pins[i][2:])
            num_near_pins = self.num_pins_near_pin(i, pins)
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
        closest_pin_distance = np.Inf
        for color in color_dict.keys():
            pin_rgb = np.array(color_dict[color])
            pin_rgb_distance = np.linalg.norm(pin_rgb - rgb)
            if pin_rgb_distance < closest_pin_distance:
                closest_pin_distance = pin_rgb_distance
                closest_pin_color = color

        pin_value = color_points[closest_pin_color]
        return pin_value

    def num_pins_near_pin(self, pin_idx, pins):
        x = int(math.floor(pins[pin_idx][0]))
        buf = 30
        num_pins = 0
        # print(pins)
        print(x, type(x))
        for px in range(x - buf, x + buf):
            for p in range(len(pins)):
                if p != pin_idx:
                    if pins[p][0] == px:
                        num_pins = num_pins + 1

        return num_pins

    def sawyer_make_move(self, move):
>>>>>>> c6c00cea0b92cda53d0403fd96a67fe3f748eae4
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
        self.sawyer_move_percent(y_per)
        # # # interpolate percentage between -0.5 and 0.3
        # min_lim = max_sawyer_left
        # max_lim = max_sawyer_right

        # y_des = min_lim + y_per * (max_lim - min_lim)
        # # set_pose(Vector3(0.0, y_goal, 0.0))
        # # self.set_pose(Vector3(0.0, y_des, 0.0))
        # self.set_pose_incrementally(Vector3(0.0, y_des, 0.0), self.current_pos, 3)
        # self.current_pos = y_des
        # TODO BIANCA
        # return None

    def sawyer_move_percent(self, move_percent):
        y_per = move_percent

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

<<<<<<< HEAD
    def baxter_throw(self):
        raw_input("Press enter for Baxter to throw")
        print("THROW")
        # TODO WILLIAM
=======
    def set_pose_incrementally(self, goal_vec, current_pos, num_steps):
        goal_pos = goal_vec.y
        for inc in np.linspace(0, goal_pos-current_pos, num=3, endpoint=True):
            if inc != 0:
                self.set_pose(Vector3(0.0, current_pos + inc, 0.0))

    def set_initial_pose(self, initial_y = 0):
        goal_vec = Vector3(0.0, initial_y, 0.0)
        self.set_pose(goal_vec, False)

    def add_obstacle(self, size = [0.60, 1.0, 0.10], x = .15, y=0, z=0.7):
        table_size = np.array(size)
        table_name = "table"

        table_pose = PoseStamped()
        table_pose.header.frame_id = "base"

        #x, y, and z position
        table_pose.pose.position.x = x
        table_pose.pose.position.y = y
        table_pose.pose.position.z = z

        #Orientation as a quaternion
        table_pose.pose.orientation.x = 0.0
        table_pose.pose.orientation.y = 0.0
        table_pose.pose.orientation.z = 0.0
        table_pose.pose.orientation.w = 1.0
        self.planner.add_box_obstacle(table_size, table_name, table_pose)

    def remove_obstacle(self, obstacle_name = "table"):
        self.planner.remove_obstacle(obstacle_name)

    def get_constraints(self, add_constraints = False):
        constraints = []
        if add_constraints:
            ## Create a path constraint for the arm
            orien_const = OrientationConstraint()
            orien_const.link_name = "right_hand";
            orien_const.header.frame_id = "base";
            # orien_const.orientation.x = 0.92;
            orien_const.orientation.y = -1;
            orien_const.absolute_x_axis_tolerance = 0.1;
            orien_const.absolute_y_axis_tolerance = 0.1;
            orien_const.absolute_z_axis_tolerance = 0.1;
            orien_const.weight = 1.0;
            constraints.append(orien_const)
        return constraints

    def set_pose(self, goal_vec, obstacle = True):

        # we can add obstacles if we choose to do so
        if obstacle:
            self.add_obstacle()
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

                #Orientation as a quaternion
                goal.pose.orientation.x = 0
                goal.pose.orientation.y = -1
                goal.pose.orientation.z = 0
                goal.pose.orientation.w = 0

                plan = self.planner.plan_to_pose(goal, constraints)

                if not self.planner.execute_plan(plan):
                    raise Exception("Execution failed")
            except Exception as e:
                print e
            else:
                break

    def sawyer_throw(self):
        raw_input("LAUNCH pin! Press Enter when done ")
        print("Thrown")
        # for i in range(25):
        #     self.arduino.write(1)
        #     # arduino.write("throw".encode())
        # self.arduino.write("".encode())
>>>>>>> c6c00cea0b92cda53d0403fd96a67fe3f748eae4
        return True
