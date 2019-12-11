import numpy as np

winning_score = 20
back_score = 10

# initial board state:
#    7 9 8
#  5 11 12 6
#   3 10 4
#    1 2
# IRL this will be much larger. possibly on the scale of camera resolution.
initial_board_state = np.array([
    [ 0,  0,  0,  0,  0,  0,  0,  0,  0],
    [ 0,  0,  7,  0,  9,  0,  8,  0,  0],
    [ 0,  5,  0, 11,  0, 12,  0,  6,  0],
    [ 0,  0,  3,  0, 10,  0,  4,  0,  0],
    [ 0,  0,  0,  1,  0,  2,  0,  0,  0],
    [ 0,  0,  0,  0,  0,  0,  0,  0,  0]
])

color_dict = {
    "white":    [  0,   0,     0],
    "red":      [255,   0,     0],
    "yellow":   [  0, 255,   255],
    "blue":     [  0,   0,   255]
}

color_points = {
    "white":    4,
    "red":      3,
    "yellow":   2,
    "blue":     1
}

max_x_pixel_left = 0
max_x_pixel_right = 700

max_sawyer_left = -0.2
max_sawyer_right = 0.5

largest_pin_value = max(color_points.values())
