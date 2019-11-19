#!/usr/bin/python
from game import Game
import sys
import numpy as np
from game_state import GameState
from constants import initial_board_state

def checkStr(actual, expected, msg = ""):
    if (not str(actual) == str(expected)):
        print(msg + " Failed. Actual: " + str(actual) + " Expected: " + str(expected))
        return False
    else:
        return True

def check(actual, expected, msg = ""):
    if (not actual == expected):
        print(msg + " Failed. Actual: " + str(actual) + " Expected: " + str(expected))
        return False
    else:
        return True

def checkBool(actual, expected, msg = ""):
    if (not actual == expected):
        print(msg + " Failed. Actual: " + str(actual) + " Expected: " + str(expected))
        return False
    else:
        return True

def test_next_turn():
    game_state = GameState(3, 1, initial_board_state)
    game_state.next_turn()
    turn0 = game_state.get_current_turn()
    if not check(turn0, 1, "Turn 0"):
        return False
    game_state.next_turn()
    turn1 = game_state.get_current_turn()
    if not check(turn1, 2, "Turn 1"):
        return False
    game_state.next_turn()
    turn2 = game_state.get_current_turn()
    if not check(turn2, 0, "Turn 2"):
        return False
    return True

def is_a_game_over(state, score1, score2):
    state.update_score(score1)
    state.next_turn()
    state.update_score(score2)
    state.next_turn()
    is_game_over = state.is_game_over()
    return is_game_over

def test_is_game_over():
    game_state = GameState(2, 0, initial_board_state)
    game_over1 = is_a_game_over(game_state, 0, 3)
    if not check(game_over1, -1, "game_over1"):
        return False
    game_over2 = is_a_game_over(game_state, 30, 40)
    if not check(game_over2, -1, "game_over2"):
        return False
    game_over3 = is_a_game_over(game_state, 51, 40)
    if not check(game_over3, -1, "game_over3"):
        return False
    game_over4 = is_a_game_over(game_state, 50, 40)
    if not check(game_over4, 0, "game_over4"):
        return False
    return True

def test_update_a_score(state, score1, score2):
    state.update_score(score1)
    state.next_turn()
    state.update_score(score2)
    state.next_turn()
    return state.get_scores()

def test_update_score():
    game_state = GameState(2, 0, initial_board_state)
    scores1 = test_update_a_score(game_state, 40, 10)
    if not check(scores1, [40,10], "test_update_scores 1"):
        return False
    scores2 = test_update_a_score(game_state, 40, 5)
    if not check(scores2, [40,5], "test_update_scores 2"):
        return False
    scores3 = test_update_a_score(game_state, 40, 11)
    if not check(scores3, [40,11], "test_update_scores 3"):
        return False
    return True

def test_score_board():
    game = Game(2, 0)
    board_state = np.array([
        [ 0,  1,  2,  3,  0],
        [ 0,  0, 10,  0,  0],
        [ 0,  4,  6,  5,  0],
        [ 0,  7,  8,  9,  0],
        [ 0, 11,  0, 12,  0]
    ])
    score1 = game.score_board(board_state)
    if not check(score1, 0, "test_score_board 1"):
        return False
    board_state = np.array([
        [ 0,  1,  2,  3,  0],
        [ 0,  0,  0,  0,  0],
        [ 0,  4,  6,  5,  0],
        [ 0,  7,  8,  9,  0],
        [ 0, 11,  0, 12,  0]
    ])
    score2 = game.score_board(board_state)
    if not check(score2, 10, "test_score_board 2"):
        return False
    board_state = np.array([
        [ 0,  1,  0,  3,  0],
        [ 0,  0,  0,  0,  0],
        [ 0,  4,  6,  5,  0],
        [ 0,  7,  8,  9,  0],
        [ 0, 11,  0, 12,  0]
    ])
    score3 = game.score_board(board_state)
    if not check(score3, 2, "test_score_board 3"):
        return False
    return True

def run_tests():
    if not test_next_turn():
        print("test_next_turn failed")
        return False
    if not test_is_game_over():
        print("test_is_game_over failed")
        return False
    if not test_update_score():
        print("test_update_score failed")
        return False
    if not test_score_board():
        print("test_score_board failed")
        return False
    return True

if __name__ == "__main__":
    num_players = 3
    robot_turn = 1
    if len(sys.argv) > 2:
        num_players = int(sys.argv[1])
        robot_turn = int(sys.argv[2])

    if not run_tests():
        print("Tests Failed")
    else:
        print("Tests passed")
