#!/usr/bin/python
from game import Game
import sys
import numpy as np

# python main.py num_players robot_turn_index

def play_molkky(num_players, robot_turn):
    game = Game(num_players, robot_turn)
    game.play_game()

if __name__ == "__main__":
    num_players = 2
    robot_turn = 1
    if len(sys.argv) > 2:
        num_players = int(sys.argv[1])
        robot_turn = int(sys.argv[2])

    play_molkky(num_players, robot_turn)
