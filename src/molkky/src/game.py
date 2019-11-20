import numpy as np
from game_state import GameState
from constants import initial_board_state
from board_vision import get_new_board_state
from strategy import get_best_expected_move
from move_baxter import baxter_make_move
from move_baxter import baxter_throw

class Game:
    def __init__(self, num_players, robot_turn):
        self.num_players = num_players
        self.scores = [0] * num_players
        self.robot_turn = robot_turn
        self.turn = 0
        self.state = GameState(num_players, robot_turn, initial_board_state)

    def play_game(self):
        print("Start game. Num_players: " + str(self.state.num_players)
              + ". Robot_turn: " + str(self.state.robot_turn))
        while (self.state.is_game_over() == -1):
            if (self.state.turn == self.state.robot_turn):
                print("Starting Baxter's Turn")
                print("Analyze board to get the best move")
                next_move = get_best_expected_move(self.state)
                print("Next move acquired (some sort of angle vector)")
                print("Move baxter to position")
                baxter_make_move(next_move)
                print("Baxter is moved to position")
                print("Baxter throw")
                baxter_throw()
            else:
                print("Player " + str(self.state.turn) + "'s turn")
                print("Player " + str(self.state.turn) + ". Throw")
                turn = input("Enter once throw is done")
            print("About to use CV to see pins")
            knocked_down_board_state = get_new_board_state()
            print("Used CV to see pins. Created a knocked down board")
            print("scoring pins to see points earned")
            points_gained = self.score_board(knocked_down_board_state)
            print("updating state with new score")
            self.state.increment_score(points_gained)
            print("Score updated")
            print("Please reset pins")
            turn = input("Press enter once pins are put upright")
            print("Use CV to see new board_state. Update state")
            self.state.board_state = get_new_board_state()
            print("move on to next player's turn")
            self.state.next_turn()
            break # to break out while testing

        winner = self.state.is_game_over()
        print("Player " + str(winner) + " wins")
        print("Scores:", self.state.scores)

    def score_board(self, board):
        pins = [False] * 13
        score = 0
        for i in range(len(board)):
            for j in range(len(board)):
                pins[board[i][j]] = True
        num_pins = pins[1:].count(False)
        if (num_pins == 1):
            score = pins.index(False)
        else:
            score = pins[1:].count(False)
        return score
