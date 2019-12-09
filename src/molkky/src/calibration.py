'''
Move to the right, shoot
Move to the left, shoot

Wait for marking

Use camera to capture 2 different pins
Choose leftmost and rightmost as calibration
Record that distance

When in game.py x-location of pin found, send back ratio of
(x-loc - leftmost)/distance
'''
from game import Game
from move_baxter.py import *
import sys
import rospy

import numpy as np

def calibrate():
    baxter_make_move(5, max=True)
    baxter_throw()

    raw_input("Press any key when done placing right marker")

    baxter_make_move(-5, max=True)
    baxter_throw()

    raw_input("Press any key when done placing left marker")

    ## Do vision processing to recognize two separate, non-background clusters
    left_loc = None ## Record left pixel center
    right_loc = None ## Record right pixel center
    return left_loc, right_loc, np.linalg.norm(np.array(right_loc) - np.array(left_loc)


if __name__ == "__main__":
    calibrate()
