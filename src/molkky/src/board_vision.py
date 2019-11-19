import numpy as np

def get_new_board_state():
      print("Use computer vision to get board state. Call some function. Make this a subscriber")
      board_state = np.array([
          [ 0,  1,  2,  3,  0],
          [ 0,  0, 10,  0,  0],
          [ 0,  4,  6,  5,  0],
          [ 0,  7,  8,  9,  0],
          [ 0, 11,  0, 12,  0]
      ])
      print("Process board_state and try to make it into some sort of file type")
      return board_state

def read_board():
    board = [[0 for _ in range(n)] for _ in range(n)]
    # use vision to determine placement of pins in grid
    return board
