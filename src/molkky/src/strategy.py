import numpy as np
from constants import winning_score

def get_best_expected_move(state):
    board_state = state.board_state
    return None

def best_play(board, score):
    row = 0
    col = 0
    max_possible = 0

    max_desired = winning_score - score

    for i in range(len(board)):
        for j in range(len(board[0])):
            one_pin = board[i][j]
            mult_pins = num_pins_near_pin(i, j, board)
            if mult_pins > 1:
                one_pin = 0
            if one_pin > max_possible or mult_pins > max_possible:
                if one_pin <= max_desired or mult_pins <= max_desired:
                    row = i
                    col = j
                    max_possible = board[i][j]

    return (row, col)

def num_pins_near_pin(row, col, board):
    buffer = 5
    num_pins = 0
    for i in range(row - buffer, row + buffer):
        for j in range(col - buffer, col + buffer):
            if board[i][j] > 0:
                num_pins = num_pins + 1

    return num_pins

if __name__ == "__main__":
    print('Baxter Molkky')
    n = 10
    board = [[0 for _ in range(n)] for _ in range(n)]
    board[(n // 2) - 2][(n // 2) - 2:(n // 2) + 1] = [7, 9, 8]
    board[(n // 2) - 1][(n // 2) - 2:(n // 2) + 2] = [5, 11, 12, 6]
    board[(n // 2)][(n // 2) - 2:(n // 2) + 1] = [3, 10, 4]
    board[(n // 2) + 1][(n // 2) - 1:(n // 2) + 1] = [1, 2]
    print(board)
