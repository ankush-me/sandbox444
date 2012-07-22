#!/usr/bin/env python

import roslib
roslib.load_manifest('rospy')
import rospy

import utils.conversions as conv
from brett2.PR2 import PR2
from brett2.PR2 import mirror_arm_joints
import math   
import numpy
 


if __name__=='__main__':
    rospy.init_node('ankush')
    bot = PR2()

    bot.lgrip.open()
    bot.rgrip.open()
    fold_reset = [math.pi/32,  math.pi/12, 0,
                  -math.pi/2,  math.pi ,  -math.pi/6,  -math.pi/4]    
    bot.larm.goto_joint_positions(fold_reset)
    bot.rarm.goto_joint_positions(mirror_arm_joints(fold_reset))

    rospy.spin()
