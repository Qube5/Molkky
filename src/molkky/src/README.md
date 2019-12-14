# src Directory
- contains publishers and subscribers

main.py
- the main program runner. takes optional command line inputs (default 2, 1)
- python main.py num_players robot_turn_index

game.py
- encapsulates the game (not runnable)

constants.py
- contains universal constants such as the winning score and colors
- Tuning parameters are all in this file

calibration.py
- If Sawyer movement is fixed, calibration can help find camera limits
- However, it is generally easier to tune them in parallel

path_planner.py
- This class executes path_planning for a given position.
- Allows adding obstacles and contraints
- Running allows user to move Sawyer to specific position percentage which is useful for tuning
- python path_planner.py
- path planning functionality for Sawyer
- class with functions that uses MoveIt to set Sawyer to goal position, add obstacles, set constraints, etc.
- publishes obstacles
- subscribes to desired x position (topic: "/goal_position") and passes position vector into set_pose function from sawyer_set_pose.py
- sawyer_set_pose.py is a script that contains the function set_pose(goal_position) called by the subscriber in path_planner.py
  - goal_position is a Vector3 message from geometry_msgs with the x_des, y_des, z_des position
  - the set_pose function establishes orientation constraints for Sawyer so that the pin doesn't fall out of the launcher
  - the set_pose function calls functions in path_planner.py to set the sawyer arm to the desired x position
  - the x position is determined by interpolating between two points on the edges of the game board
