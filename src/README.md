# Mölkky Workspace src folder

## Getting started (run once)
- clone repository
- catkin_make
- ln -s /scratch/shared/baxter_ws/baxter.sh ~/ros_workspaces/Molkky/
- Tune paramters in molkky/src/constants
- rosviz and path_planner.py are helpful

## To Run game
source devel/setup.bash
- run this in each terminal window

./baxter Ada.local
- run this in each terminal window
- ssh into Sawyer robot Ada

roslaunch realsense2_camera rs_camera.launch filters:=pointcloud
- Launch realsense camera node

python src/segmentation/src/main.py
- Start image segmentation node

rosrun intera_interface joint_trajectory_action_server.py
- Starts joint trajectory action server

roslaunch sawyer_moveit_config sawyer_moveit.launch electric_gripper:=false
- starts MoveIt
- Also opens rviz
  - Add image display and set topic to /camera/color/image_raw
  - Add image display and set topic to /segmented_image

rosrun intera_interface enable_robot.py -e
- Enable Sawyer

python src/molkky/src/main.py
- runs game

## Mölkky package
Currently has following dependencies
- rospy
- roscpp
- std_msgs
- geometry_msgs
- sensor_msgs

## Node Interaction
Realsense Camera node publishes image data to several topics.

segmentation/src/main.py subscibes to Camera data. Publishes processed image data

molkky/src/main. runs the game
- while other nodes run asynchronously, this method synchronously runs the game
- subscribes to processed image data
- Chooses pin to aim for using strategy module
- handles path planning
- launches pin
- handles turns and scores

## Useful Commands

Run rosbag (useful when realsense camera is unavailable)
- Download file here: https://drive.google.com/file/d/1h-DLFYY-WIDuKtqrG9ljAPNUhrgM8Eue/view?usp=sharing
- move file to src/bagfiles directory
- roscore
- rosbag play -l src/bagfiles/realsense.bag

Allow ssh into Sawyer bots (run once)
- ln -s /scratch/shared/baxter_ws/baxter.sh ~/ros_workspaces/Molkky/

Measure sawyer arm position (useful for tuning)
- rosrun tf tf_echo base right_hand

python src/planning/src/path_planner.py
- allows user to set sawyer's position

rosrun rviz rviz
- Launch rviz - see topics above
