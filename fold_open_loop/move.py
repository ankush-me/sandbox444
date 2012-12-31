#!/usr/bin/env python

import roslib
roslib.load_manifest('rospy')
import rospy

roslib.load_manifest("move_base_msgs");
import move_base_msgs.msg as mbm

roslib.load_manifest("trajectory_msgs");
import trajectory_msgs.msg as tm

import utils.conversions as conv
from brett2.PR2 import *
from brett2.ghost_PR2 import GhostPR2

from joints import Joints, ljoints_final, rjoints_final, ljoints_fwd, rjoints_fwd
from math import pi
import tf
import numpy
import sys


def show_arms_info(robot):
    """
    Prints the joint angle values and the hand tip transformations
    for both the arms.
    """
    print "L arm joints:"
    print robot.larm.get_joint_positions()

    print "R arm joints:"
    print robot.rarm.get_joint_positions()

    print "L arm pose matrix from base_footprint to l_gripper_tool_frame:"
    print get_arm_pose_matrix(robot, 'l')

    print "R arm pose matrix from base_footprint to r_gripper_tool_frame:"
    print get_arm_pose_matrix(robot, 'r')


def prompt(str):
    """
    Prompts the user for input, using STR.
    """
    print str
    return raw_input()
    

def get_arm_pose_matrix(robot, lr, ref_frame='base_footprint'):
    """
    the transform for the LR hand tip from REF_FRAME.
    """
    if lr == "l":
        arm = robot.larm
    else:
        arm = robot.rarm
    return arm.get_pose_matrix(ref_frame, ('%s_gripper_tool_frame'%lr))


def hold_towel(robot):
    prompt("Place towel in PR2's left hand and then press return:")
    robot.lgrip.close()

    prompt("Place towel in PR2's right hand and then press return:")
    robot.rgrip.close()

    prompt("Press return when ready to proceed.")



# Constants for finding the towel's other end.
TIME_LIMIT = 10    # [seconds]
CHECK_START = 0.3  # [seconds]
STEP_SIZE = 0.1    # [meters]
THRESHOLD = 0.003  # [meters]

VERBOSE = True


if __name__=='__main__':

    if len(sys.argv) > 1:
        VERBOSE = (sys.argv[1] in ['1', 'yes', 'true', 'TRUE', 'y', 'Y', 'T', 't'])

    rospy.init_node('towel_fold')
    bot = PR2()
    rospy.loginfo(bot.base.get_pose())

    # Move the base
    #bot.base.goto_pose(10,5.0,pi/2, '/map')


    # Move the Torso
    #bot.torso.go_up()


    # Move the Head
    #p = numpy.array([1,0,0.5])
    #bot.head.look_at(p, 'base_footprint', 'narrow_stereo_link')

    if VERBOSE:
        show_arms_info(bot)

    bot.lgrip.open()
    bot.rgrip.open()
    rospy.sleep(1)

    fold_reset = [pi/32,  pi/12,   0, -pi/2,  pi ,  -pi/6,  -pi/4]
    bot.larm.goto_joint_positions(fold_reset)
    bot.rarm.goto_joint_positions(mirror_arm_joints(fold_reset))
    rospy.sleep(3)

    hold_towel(bot)
    rospy.sleep(3)

    delta1 = [0, 0, pi/6.2, 0, 0, 0, pi/2]
    bot.larm.goto_delta_joint_positions(delta1)
    bot.rarm.goto_delta_joint_positions(mirror_arm_joints(delta1))

    rospy.sleep(5)

    delta2 = [0, 0, 0, 0, 0, 0, pi/4]
    bot.larm.goto_delta_joint_positions(delta2)
    bot.rarm.goto_delta_joint_positions(mirror_arm_joints(delta2))

    rospy.sleep(3)
    bot.lgrip.set_angle(0.01,10)
    rospy.sleep(3)

    delta3 = [0, pi/50, pi/20, pi/20, 0, 0, -pi/2]
    bot.larm.goto_delta_joint_positions(delta3)

    rospy.sleep(3)

    delta4 = [0, 0, 0, 0, 0, 0, pi/1.6]
    bot.rarm.goto_delta_joint_positions(delta4)

    rospy.sleep(3)
    # get the current left hand's tip position
    pos_mat = get_arm_pose_matrix(bot, 'l')
    # pos_mat[2,3] -= 0.05  # Change the z
    # pos_mat[1,3] -= 0.03  # Change the y

    bot.larm.follow_joint_trajectory(Joints, True)
    #rospy.sleep(0.5)


    t = 0
    pos_mat = get_arm_pose_matrix(bot, 'l')
    z_dist_earlier = pos_mat[2,3]

    while t <= TIME_LIMIT:
        rospy.sleep(STEP_SIZE)

        pos_mat = get_arm_pose_matrix(bot, 'l')
        z_dist_later = pos_mat[2,3]
        #print abs(z_dist_later-z_dist_earlier) # print the distance moved
        if abs(z_dist_later-z_dist_earlier) < THRESHOLD and t > CHECK_START:
            bot.larm.controller_pub.publish(tm.JointTrajectory())
            break

        z_dist_earlier = z_dist_later
        t += STEP_SIZE

    bot.larm.controller_pub.publish(tm.JointTrajectory())

    delta5 = [0, 0, 0, 0, 0, 0, -pi/1.1]
    bot.larm.goto_delta_joint_positions(delta5)

    rospy.sleep(8)
    bot.lgrip.close()
    
    rospy.sleep(2)

    bot.larm.goto_joint_positions(ljoints_final)
    bot.rarm.goto_joint_positions(rjoints_final)
    rospy.sleep(3)

    delta6 = [0, 0, 0, 0, 0, -pi/4, 0]
    bot.larm.goto_delta_joint_positions(delta6)
    bot.rarm.goto_delta_joint_positions(delta6)
    rospy.sleep(3)

    bot.larm.goto_joint_positions(ljoints_fwd)
    bot.rarm.goto_joint_positions(rjoints_fwd)
    rospy.sleep(6)

    bot.larm.goto_joint_positions(ljoints_final)
    bot.rarm.goto_joint_positions(rjoints_final)
    rospy.sleep(5)
    

    if VERBOSE:
        show_arms_info(bot)    

    # Move the Torso
    #bot.torso.go_up()


    """
    rospy.sleep(5)

    z1 = bot.larm.get_pose_matrix('base_footprint','r_gripper_palm_link')
    z2 = bot.larm.get_pose_matrix('base_footprint','l_gripper_palm_link')
    y1 = conv.hmat_to_pose(z1)
    y2 = conv.hmat_to_pose(z2)
    print y1
    print y2
    print 'dist: ' + str(y1.position.y-y2.position.y)

    print "Left arm joint angles: "
    print bot.larm.get_joint_positions()
    print "Right arm joint angles: "
    print bot.rarm.get_joint_positions()
    """
    rospy.loginfo('Done. Back to you!')
    rospy.spin()
