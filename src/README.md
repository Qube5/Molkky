# Mölkky Workspace src folder

## Mölkky package
Currently has following dependencies
- rospy
- roscpp
- std_msgs
- geometry_msgs
- sensor_msgs

Note that other packages in here are not necessarily used but are in here as examples

## Node Interaction
Realsense Camera node publishes image data to several topics.

segmentation/src/main.py subscibes to Camera data. Publishes processed image data to a TOPIC TBD

MAIN.PY runs the game
- while other nodes run asynchronously, this method synchronously runs the game
- subscribes to processed image data TOPIC
- determines if any of the pins have moved. Updates the score.
- Runs strategy on board. Strategy chooses which pin to aim for.
- Main finds coordinate of pin in image
- Main maps pin coordinates to x-position
- Main maps x-position to Joint Position
- Main publishes Joint Position to TOPIC
- Launches bozo

path_planner
- subscribes to joint Position
- moves arm to that Position

### Useful Notes
to run rosbag: https://drive.google.com/file/d/1h-DLFYY-WIDuKtqrG9ljAPNUhrgM8Eue/view?usp=sharing
roscore
rosbag play -l src/bagfiles/realsense.bag
ln -s /scratch/shared/baxter_ws/baxter.sh ~/ros_workspaces/Molkky/
rosrun tf tf_echo base right_hand

At time 1575926809.076
- Translation: [0.467, -0.433, 0.256]
- Rotation: in Quaternion [0.921, 0.020, 0.355, 0.157]
            in RPY (radian) [2.732, -0.705, 0.197]
            in RPY (degree) [156.531, -40.390, 11.270]

At time 1575926891.256
- Translation: [0.598, 0.227, 0.114]
- Rotation: in Quaternion [0.920, -0.067, 0.383, 0.053]
            in RPY (radian) [3.077, -0.791, -0.118]
            in RPY (degree) [176.290, -45.333, -6.758]

- python src/planning/src/path_planner.py
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

## to run
- catkin_make
- source devel/setup.bash
  - run this in each terminal window

- roslaunch realsense2_camera rs_camera.launch filters:=pointcloud

- rosrun rviz rviz
	- Change the fixed frame to be camera_depth_optical_frame. Next, add a new Display of type PointCloud2. Set the topic for this display to /camera/depth/color/points. You may have to wait a bit for Rviz to begin registering pointcloud messages (as you can imagine, point-cloud messages tend to be pretty heavy).
	- Now, also add two Image displays. Set the topic for one to /camera/color/image_raw. Set the topic for the other one to /camera/depth/image_rect_raw. These two displays will display the RGB and the Depth images respectively.

- python main.py
	- run from /ros_workspaces/Molkky/src/segmentation/src
	- OR: python src/segmentation/src/main.py
	- Set the fixed frame to camera_depth_optical_frame. In the right hand side window, check the box labelled Invert z-axis. Create an Image display and set its topic to camera/color/image_raw. Now create a Pointcloud2 display, and set its topic to /segmented_points (you may not able to do this until after you have started up main.py).
	- Add an image display and set its topic to /segmented_image

- python src/molkky/src/main.py
	- runs game


At time 1575926809.076
- Translation: [0.467, -0.433, 0.256]
- Rotation: in Quaternion [0.921, 0.020, 0.355, 0.157]
            in RPY (radian) [2.732, -0.705, 0.197]
            in RPY (degree) [156.531, -40.390, 11.270]


At time 1575926891.256
- Translation: [0.598, 0.227, 0.114]
- Rotation: in Quaternion [0.920, -0.067, 0.383, 0.053]
            in RPY (radian) [3.077, -0.791, -0.118]
            in RPY (degree) [176.290, -45.333, -6.758]

v2 higher

At time 1576107439.441
- Translation: [0.329, -0.591, 0.351]
- Rotation: in Quaternion [0.995, -0.003, 0.029, 0.093]
            in RPY (radian) [2.955, -0.059, 0.000]
            in RPY (degree) [169.308, -3.385, 0.007]
left

At time 1576107497.481
- Translation: [0.502, 0.288, 0.247]
- Rotation: in Quaternion [0.989, -0.045, 0.124, -0.061]
            in RPY (radian) [-3.006, -0.242, -0.108]
            in RPY (degree) [-172.210, -13.842, -6.182]
right



rosrun intera_interface enable_robot.py -e

rosrun intera_interface joint_trajectory_action_server.py

roslaunch sawyer_moveit_config sawyer_moveit.launch electric_gripper:=false

