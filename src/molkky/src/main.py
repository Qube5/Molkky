#!/usr/bin/python
from game import Game
import sys
import numpy as np

# python main.py num_players robot_turn_index

def play_molkky(num_players, robot_turn):
    # rospy.init_node('molkky')
    game = Game(num_players, robot_turn)

    # r = rospy.Rate(1000)
    print("Start game. Num_players: " + str(game.state.num_players)
          + ". Robot_turn: " + str(game.state.robot_turn))

    while (game.state.is_game_over() == -1):
    # while (self.state.is_game_over() == -1 and not rospy.is_shutdown()):
        game.take_turn()
        # r.sleep()

    winner = game.state.is_game_over()
    print("Player " + str(winner) + " wins")
    print("Scores:", game.state.get_scores())

if __name__ == "__main__":
    num_players = 2
    robot_turn = 1
    if len(sys.argv) > 2:
        num_players = int(sys.argv[1])
        robot_turn = int(sys.argv[2])

    play_molkky(num_players, robot_turn)
