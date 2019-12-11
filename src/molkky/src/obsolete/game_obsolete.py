import numpy as np
from game_state import GameState
from constants import initial_board_state, color_dict, color_points, largest_pin_value, winning_score
# from board_vision import get_new_board_state
# from strategy import get_best_expected_move
# from move_baxter import baxter_make_move
# from move_baxter import baxter_throw

def numpy_to_move_msg(move):
    # print(img.shape)
    # print(np.array([img, img, img]).shape)
    return ros_numpy.msgify(String, move)

def

class Game:
    def __init__(self, num_players, robot_turn):
        self.num_players = num_players
        self.scores = [0] * num_players
        self.robot_turn = robot_turn
        self.turn = 0
        self.state = GameState(num_players, robot_turn, initial_board_state)
        self.prev_board_state = None #self.get_new_board_state()

        self.current_image_info = None

        # info_sub = message_filters.Subscriber(points_sub_topic, PointCloud2)
        # rospy.Subscriber("image_info", String, self.info_callback)

        # self.move_pub = rospy.Publisher("points_pub_topic", String, queue_size=10)

    def info_callback(self, image_info):
        try:
            info = image_info
        except Exception as e:
            rospy.logerr(e)
            return
        self.current_image_info = info

    def take_turn(self):
        # if not self.current_image_info:
        #     return
        # Turn starts with strategy then a throw
        print()
        print("Current Score: " + str(self.state.get_scores()))
        if (self.state.turn == self.state.robot_turn):
            print("Starting Baxter's Turn")
            print("Analyze board to get the best move")
            # using the previous board state to figure out the best move
            next_move = self.get_best_expected_move(self.prev_board_state)
            # next_move = get_best_expected_move(self.state)
            print("Next move acquired (some sort of angle vector)", next_move)

            # Send move to make move
            print("Move baxter to position")
            self.baxter_make_move(next_move)

            print("Baxter is moved to position")
            input("Enter once Baxter is in position")

            # Baxter throw using pneumatic launcher
            print("Baxter throw")
            self.baxter_throw()
        else:
            print("Player " + str(self.state.turn) + "'s turn")
            print("Player " + str(self.state.turn) + ". Throw")
            turn = input("Enter once throw is done")

        print("Please set pins upright after the throw")
        input("Enter once done")

        print("About to use CV to see pins")
        board_state = self.get_new_board_state()
        self.state.update_board_state(board_state)
        # print("Used CV to see pins. Created a knocked down board")
        print("scoring pins to see points earned")
        points_gained = self.score_board(board_state, self.prev_board_state)

        print("Updating state with new score")
        self.state.increment_score(points_gained)
        print("Score updated")

        self.state.next_turn()
        self.prev_board_state = board_state
        # break # to break out while testing

    def score_board(self, board, prev_board):
        # board_state has {"cluster_centers": centers, "cluster_colors": cluster_colors}
        ## first match up previous clusters to new clusters based on color.
        new_prev_board = {"cluster_centers": {}, "cluster_colors": {}}
        for prev_gray in prev_board["cluster_colors"].keys():
            prev_cluster_rgb = prev_board["cluster_colors"][prev_gray]

            closest_cluster = 0 #prev_cluster
            closest_cluster_dist = 1000000000
            for gray in board["cluster_colors"].keys():
                cluster_rgb = board["cluster_colors"][gray]
                cluster_dist = np.linalg.norm(prev_cluster_rgb - cluster_rgb)
                if cluster_dist < closest_cluster_dist:
                    closest_cluster_dist = cluster_dist
                    closest_cluster = gray

            new_prev_board["cluster_centers"][closest_cluster] = prev_board["cluster_centers"][prev_gray]
            new_prev_board["cluster_colors"][closest_cluster] = prev_board["cluster_colors"][prev_gray]
        print(prev_board)
        print(board)
        print(new_prev_board)

        ## figure out which clusters have moved significantly
        moved_clusters = [] # list of gray indexes of clusters
        dist_moved = 10
        for gray in board["cluster_colors"].keys():
            old_pos = prev_board["cluster_centers"][gray]
            new_pos =      board["cluster_centers"][gray]

            if np.linalg.norm(old_pos - new_pos) > dist_moved:
                moved_clusters.append(gray)

        if len(moved_clusters) > 1:
            score = len(moved_clusters)
            return score
        if len(moved_clusters) < 1:
            print("ERROR")
            return -1

        moved_pin = None # rgb color from color dict
        moved_cluster = moved_clusters[0]

        rgb = board["cluster_colors"][moved_cluster]

        # find closest color in color dict
        closest_color = "red"
        closest_rgb_dist = 1000000000
        for color in color_dict.keys():
            color_rgb = color_dict[color]
            color_rgb_dist = np.linalg.norm(rgb - color_rgb)
            if color_rgb_dist < closest_rgb_dist:
                closest_rgb_dist = color_rgb_dist
                closest_rgb = color_rgb
                closest_color = color

        score = color_points[closest_color]
        return score
        # TODO
        # pins = [False] * 13
        # score = 0
        # for i in range(len(board)):
        #     for j in range(len(board)):
        #         pins[board[i][j]] = True
        # num_pins = pins[1:].count(False)
        # if (num_pins == 1):
        #     score = pins.index(False)
        # else:
        #     score = pins[1:].count(False)
        score = 0
        return score

    def get_new_board_state(self):
        if not self.current_image_info:
            return None

        # else process self.current_image_info into a board state and return it
        board_state = None
        # print("Use computer vision to get board state. Call some function. Make this a subscriber")
        # # TODO
        # board_state = np.array([
        #   [ 0,  1,  2,  3,  0],
        #   [ 0,  0, 10,  0,  0],
        #   [ 0,  4,  6,  5,  0],
        #   [ 0,  7,  8,  9,  0],
        #   [ 0, 11,  0, 12,  0]
        # ])
        # print("Process board_state and try to make it into some sort of file type")
        return board_state

    def get_best_expected_move(self, board_state):
        # board_state has {"cluster_centers": centers, "cluster_colors": cluster_colors}
        # each is indexed by the grayscale value corresponding to that cluster.
        # Each color maps to a point value. maximize points.

        print(largest_pin_value)
        ## Get best pin to knock down
        current_score = self.scores[self.robot_turn]
        # print(current_score)
        optimal_points = winning_score - current_score

        if largest_pin_value <= optimal_points:
            best_pin_points = largest_pin_value
        else:
            best_pin_points = optimal_points

        key_list = list(color_points.keys())
        val_list = list(color_points.values())
        best_pin_color = key_list[val_list.index(best_pin_points)]
        best_pin_rgb = np.array(color_dict[best_pin_color])

        ## GET grayscale cluster index of best pin

        best_pin_cluster = 0
        closest_rgb = best_pin_rgb
        closest_rgb_distance = 1000000000
        for gray in board_state["cluster_colors"].keys():
            cluster_rgb = board_state["cluster_colors"][gray]
            cluster_rgb_distance = np.linalg.norm(best_pin_rgb - cluster_rgb)
            if cluster_rgb_distance < closest_rgb_distance:
                closest_rgb_distance = cluster_rgb_distance
                closest_rgb = cluster_rgb
                best_pin_cluster = gray

        best_pin_cluster_center = board_state["cluster_centers"][gray]
        best_pin_cluster_x = best_pin_cluster_center[0]

        ## CONVERT best_pin_cluster_center to x-axis coordinate


        # board_state = state
        # board_state = state.board_state
        # TODO
        return None

    def baxter_make_move(self, move):
        print("MOVE")
        # move_msg = numpy_to_move_msg(move)
        # self.move_pub.publish(move_msg)
        # self.move_pub.publish(move)
        # TODO QUENTIN/BIANCA
        return None

    def baxter_throw(self):
        print("THROW")
        # TODO WILLIAM
        return True
