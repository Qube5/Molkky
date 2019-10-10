#!/usr/bin/python
import sys
import numpy as np

winning_score = 50
initial_board_state = np.array([
    [ 0,  1,  2,  3,  0],
    [ 0,  0, 10,  0,  0],
    [ 0,  4,  6,  5,  0],
    [ 0,  7,  8,  9,  0],
    [ 0, 11,  0, 12,  0]
])

def next_turn(last_turn, num_players):
    return (last_turn + 1) % num_players

def get_board_state():
    print("use computer vision to get board state")
    board_state = np.array([
        [ 0,  1,  2,  3,  0],
        [ 0,  0, 10,  0,  0],
        [ 0,  4,  6,  5,  0],
        [ 0,  7,  8,  9,  0],
        [ 0, 11,  0, 12,  0]
    ])
    return board_state

def is_game_over(scores):
    game_over = False
    for i in range(len(scores)):
        if scores[i] == winning_score:
            print("Player " + str(i) + " wins")
            game_over = True
    return game_over

def score_board(board_state):
    pins = [False] * 13
    score = 0
    for i in range(len(board_state)):
        for j in range(len(board_state[0])):
            pins[board_state[i][j]] = True
    num_pins = pins[1:].count(False)
    if (num_pins == 1):
        score = pins.index(False)
    else:
        score = pins[1:].count(False)
    return score

def update_score(curr_score, points_gained):
    score = curr_score + points_gained
    score = score if score <= 50 else 25
    return score

def get_best_expected_move(board_state):
    return None

def baxter_make_move(move):
    return None

def play_game(num_players, robot_turn):
    scores = []
    player_turn = 0

    board_state = initial_board_state

    for i in range(num_players):
        scores.extend([0])

    while (not is_game_over(scores)):
        if (player_turn != robot_turn):
            print("Player " + str(i) + "'s turn")
            turn = input("Enter once turn is done")
        else:
            board_state = get_board_state()
            next_move = get_best_expected_move(board_state)
            baxter_make_move(next_move)
        board_state = get_board_state()
        points_gained = score_board(board_state)
        scores[player_turn] = update_score(scores[player_turn], points_gained)
        player_turn = next_turn(player_turn, num_players)
        break

    print(scores)
    print(num_players, robot_turn)

def check(actual, expected):
    if (not str(actual) == str(expected)):
        print("Test Failed. Actual: " + str(actual) + " Expected: " + str(expected))
        return False
    else:
        return True

def test_next_turn():
    turn0 = next_turn(0, 3)
    turn1 = next_turn(1, 3)
    turn2 = next_turn(2, 3)
    return (check(turn0, 1) and
            check(turn1, 2) and
            check(turn2, 0))

def test_is_game_over():
    game_over1 = is_game_over([0, 3])
    game_over2 = is_game_over([30, 40])
    game_over3 = is_game_over([51, 40])
    game_over4 = is_game_over([50, 40])
    return (check(game_over1, False) and
            check(game_over2, False) and
            check(game_over3, False) and
            check(game_over4, True))

def test_update_score():
    score1 = update_score(40, 10)
    score2 = update_score(40, 5)
    score3 = update_score(40, 11)
    return (check(score1, 50) and
            check(score2, 45) and
            check(score3, 25))

def test_score_board():
    board_state = np.array([
        [ 0,  1,  2,  3,  0],
        [ 0,  0, 10,  0,  0],
        [ 0,  4,  6,  5,  0],
        [ 0,  7,  8,  9,  0],
        [ 0, 11,  0, 12,  0]
    ])
    score1 = score_board(board_state)
    board_state = np.array([
        [ 0,  1,  2,  3,  0],
        [ 0,  0,  0,  0,  0],
        [ 0,  4,  6,  5,  0],
        [ 0,  7,  8,  9,  0],
        [ 0, 11,  0, 12,  0]
    ])
    score2 = score_board(board_state)
    board_state = np.array([
        [ 0,  1,  0,  3,  0],
        [ 0,  0,  0,  0,  0],
        [ 0,  4,  6,  5,  0],
        [ 0,  7,  8,  9,  0],
        [ 0, 11,  0, 12,  0]
    ])
    score3 = score_board(board_state)
    return (check(score1, 0) and
            check(score2, 10) and
            check(score3, 2))

def run_tests():
    if (not test_next_turn() or
        not test_is_game_over() or
        not test_update_score() or
        not test_score_board()):
        return False
    else:
        return True

if __name__ == "__main__":
    # num_players = int(sys.argv[1])
    # robot_index = int(sys.argv[2])
    num_players = 3
    robot_turn = 1

    if not run_tests():
        print("Tests Failed")
    else:
        print("Tests passed")

    play_game(num_players, robot_turn)
