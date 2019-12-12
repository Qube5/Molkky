#!/usr/bin/env python


import rospy
import numpy as np
import itertools
from intera_interface import Limb

if __name__ == '__main__':
    rospy.init_node('fix_alan')

    limb = Limb('right')
    while not rospy.is_shutdown():
        limb.set_joint_positions(dict(itertools.izip(limb.joint_names(), np.zeros(len(limb.joint_names())))))
    rospy.spin()
