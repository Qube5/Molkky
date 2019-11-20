# src Directory
- contains publishers and subscribers

main.py
- the main program runner. takes command line inputs

test.py
- unit tests

game.py
- encapsulates the game and runs everything within the class

move_baxter.py
- encapsulates sending commands to baxter including moving and throwing

strategy.py
- computes the best move for the given board_state

board_vision.py
- reads the board and creates a board

game_state.py
- object holds the game state such as board positions and scores
- in theory this could be saved to a file

constants.py
- contains universal constants such as the winning score
