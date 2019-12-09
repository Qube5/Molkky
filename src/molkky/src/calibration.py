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
from move_baxter.py import *
from segmentation.main import PointcloudProcess, isolate_object_of_interest
import sys
import rospy

import numpy as np

def capture_loc():
    CAM_INFO_TOPIC = '/camera/color/camera_info'
    RGB_IMAGE_TOPIC = '/camera/color/image_raw'
    POINTS_TOPIC = '/camera/depth/color/points'
    POINTS_PUB_TOPIC = 'segmented_points'

    rospy.init_node('realsense_listener')
    process = PointcloudProcess(POINTS_TOPIC, RGB_IMAGE_TOPIC,
                                CAM_INFO_TOPIC, POINTS_PUB_TOPIC)
    r = rospy.Rate(1000)

    lcenters = np.array([])

    for i in range(5):
        info, segmented_image, segmented_image_binary =
            process.capture_once_calibration()

        #Calculate center of left and of right, using nearest neighbors to
        #red and blue.
        c1col = info['cluster_colors'][0]
        lind = 0 if (abs(c1col[0] -255)  < abs(c1col[2] - 255)) else 1 #Nearest neighbor of cluster_colors to red, (255, 0, 0)
        rind = 1 - lind #Opposite index of lind

        lcenters.append(np.array(info['cluster_centers'][l_ind]))
        rcenters.append(np.array(info['cluster_centers'][r_ind]))


        r.sleep()
    return np.mean(lcenters), np.mean(rcenters)

def calibrate():
    baxter_make_move(5, max=True)
    baxter_throw()

    raw_input("Press any key when done placing right marker")

    baxter_make_move(-5, max=True)
    baxter_throw()

    raw_input("Press any key when done placing left marker")

    ## Do vision processing to recognize two separate, non-background clusters
    left_loc, right_loc = capture_loc()
    return left_loc, right_loc, np.linalg.norm(np.array(right_loc) - np.array(left_loc))


if __name__ == "__main__":
    calibrate()
