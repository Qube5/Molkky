# Mölkky Workspace src folder

## Mölkky package
Currently has following dependencies
- rospy 
- roscpp 
- std_msgs 
- geometry_msgs 
- sensor_msgs

Note that other packages in here are not necessarily used but are in here as examples

## to run

- catkin_make
- source devel/setup.bash

- roslaunch realsense2_camera rs_camera.launch filters:=pointcloud

- rosrun rviz rviz
	- Change the fixed frame to be camera_depth_optical_frame. Next, add a new Display of type PointCloud2. Set the topic for this display to /camera/depth/color/points. You may have to wait a bit for Rviz to begin registering pointcloud messages (as you can imagine, point-cloud messages tend to be pretty heavy).
	- Now, also add two Image displays. Set the topic for one to /camera/color/image_raw. Set the topic for the other one to /camera/depth/image_rect_raw. These two displays will display the RGB and the Depth images respectively.

- python main.py
	- run from /ros_workspaces/Molkky/src/segmentation/src
	- Set the fixed frame to camera_depth_optical_frame. In the right hand side window, check the box labelled Invert z-axis. Create an Image display and set its topic to camera/color/image_raw. Now create a Pointcloud2 display, and set its topic to /segmented_points (you may not able to do this until after you have started up main.py).
	- Add an image display and set its topic to /segmented_image