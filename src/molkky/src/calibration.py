'''
Run using py calibration.py

Moves to the right, shoots.
Waits for marking,
at which human attendant should point place BLUE pin on location of hit.


Moves to the left, shoots.
Waits for marking,
at which human attendant should point place RED pin on location of hit.


Uses camera to capture 2 different pins.
Choose left red pin and right blue pin as calibration points.
Records points and euclidean distance between them, returns

When in game.py x-location of pin found, send back ratio of
(x-loc - leftmost)/distance
'''
from game import Game
import sys
import rospy

import numpy as np

def capture_loc(game):
    current_image_info = np.array(game.current_image_info)
    current_image_info = np.array(game.get_current_image_info())
    current_image_info = np.array(current_image_info)
    print(current_image_info)
    xy = current_image_info[:,3:]
    x = xy[:,0]
    return x[0], x[1]

def calibrate():
    rospy.init_node('molkky')
    game = Game(2, 1)

    r = rospy.Rate(1000)

    raw_input("Press enter to move to left limit")
    game.sawyer_move_percent(0)
    game.sawyer_throw()

    raw_input("Press enter when done placing left marker")

    raw_input("Press enter to move to right limit")
    game.sawyer_move_percent(1)
    game.sawyer_throw()

    raw_input("Press enter when done placing right marker")

    left_loc, right_loc = capture_loc(game)
    print(left_loc, right_loc)

    return left_loc, right_loc, np.linalg.norm(np.array(right_loc) - np.array(left_loc))

if __name__ == "__main__":
    calibrate()
