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
from move_baxter import *
# from segmentation.main import PointcloudProcess, isolate_object_of_interest
import sys
import rospy
from geometry_msgs.msg import Vector3

import numpy as np

def capture_loc(game):
    # CAM_INFO_TOPIC = '/camera/color/camera_info'
    # RGB_IMAGE_TOPIC = '/camera/color/image_raw'
    # POINTS_TOPIC = '/camera/depth/color/points'
    # POINTS_PUB_TOPIC = 'segmented_points'

    # rospy.init_node('realsense_listener')
    # process = PointcloudProcess(POINTS_TOPIC, RGB_IMAGE_TOPIC,
    #                             CAM_INFO_TOPIC, POINTS_PUB_TOPIC)
    # r = rospy.Rate(1000)

    # lcenters = np.array([])

    # for i in range(5):
    #     info, segmented_image, segmented_image_binary = \
    #         process.capture_once_calibration()

    #     #Calculate center of left and of right, using nearest neighbors to
    #     #red and blue.
    #     c1col = info['cluster_colors'][0]
    #     lind = 0 if (abs(c1col[0] -255)  < abs(c1col[2] - 255)) else 1 #Nearest neighbor of cluster_colors to red, (255, 0, 0)
    #     rind = 1 - lind #Opposite index of lind

    #     lcenters.append(np.array(info['cluster_centers'][l_ind]))
    #     rcenters.append(np.array(info['cluster_centers'][r_ind]))


    #     r.sleep()
    # return np.mean(lcenters), np.mean(rcenters)
    current_image_info = np.array(game.current_image_info)
    current_image_info = np.array(game.get_current_image_info())
    print(current_image_info)
    xy = current_image_info[2:]
    print(xy)
    return 1, 0

def calibrate():
    # baxter_make_move(5, max=True)
    # baxter_throw()

    # raw_input("Press any key when done placing right marker")

    # baxter_make_move(-5, max=True)
    # baxter_throw()

    # raw_input("Press any key when done placing left marker")

    # ## Do vision processing to recognize two separate, non-background clusters

    rospy.init_node('molkky')
    game = Game(2, 1)

    r = rospy.Rate(1000)

    # game.baxter_move_percent(0)
    game.baxter_throw()

    raw_input("Press enter when done placing right marker")

    # game.baxter_move_percent(1)
    game.baxter_throw()

    raw_input("Press enter when done placing left marker")

    left_loc, right_loc = capture_loc(game)
    # print(left_loc, right_loc)
    # print(game.current_image_info, "current_image_info")
    return left_loc, right_loc, np.linalg.norm(np.array(right_loc) - np.array(left_loc))


if __name__ == "__main__":
    calibrate()

