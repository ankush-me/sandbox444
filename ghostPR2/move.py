#!/usr/bin/env python

import roslib
roslib.load_manifest('rospy')
import rospy

roslib.load_manifest("actionlib"); 
import actionlib

roslib.load_manifest("geometry_msgs"); 
import geometry_msgs.msg as gm           

roslib.load_manifest("move_base_msgs"); 
import move_base_msgs.msg as mbm        

roslib.load_manifest("trajectory_msgs"); 
import trajectory_msgs.msg as tm

import utils.conversions as conv
from brett2.PR2 import *
#from brett2.PR2 import mirror_arm_joints
from brett2.ghost_PR2 import GhostPR2

from math import pi
import tf
import numpy
 

action_client = []

if __name__=='__main__':
    rospy.init_node('ankush')
    bot = PR2()
    rospy.loginfo(bot.base.get_pose())

    #bot.base.goto_pose(10,5.0,pi/2, '/map')
    #bot.torso.go_up()
    p = numpy.array([1,0,0])
    bot.head.look_at(p, 'base_link', 'narrow_stereo_link')


    fold_reset = [pi/32,  pi/12,   0, -pi/2,  pi ,  -pi/6,  -pi/4]    
    bot.larm.goto_joint_positions(fold_reset)
    bot.rarm.goto_joint_positions(mirror_arm_joints(fold_reset))

    rospy.sleep(4)

    bot.lgrip.open()
    bot.rgrip.open()

    rospy.sleep(3)

    bot.lgrip.close()
    bot.rgrip.close()

    rospy.sleep(6)

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


    delta3 = [0, pi/50, 0, 0, 0, 0, -pi/2]
    bot.larm.goto_delta_joint_positions(delta3)

    #rospy.sleep(3)

    #delta4 = [0, 0, 0, 0, 0, 0, -pi/4]
    #bot.larm.goto_delta_joint_positions(delta4)

    rospy.sleep(5)
    # get the current left palm position
    pos_mat = bot.larm.get_pose_matrix('base_link','l_gripper_tool_frame')
    #pos_mat[2,3] -= 0.05
    #pos_mat[1,3] -= 0.03

    ghost = GhostPR2()
    """
    j = ghost.get_joints_from_cart("l", bot.larm.get_joint_positions(),
                                   bot.larm.rave_joint_inds,
                                   pos_mat, 
                                   'base_link',MAX_ERROR=0.005,MAX_NUM_ITERS=5000)
    """
    j = bot.larm.get_joint_positions()
    ghost = GhostPR2()
    pos_mat = bot.larm.get_pose_matrix('base_link','l_gripper_tool_frame')

    iter = 0
    z_dist = 0.02
    Joints = []
    while iter < 20:
        iter += 1
        print iter
#        z_dist += 0.02
        pos_mat[2,3] -= z_dist        
        j = ghost.get_joints_from_cart("l", j,
                                       bot.larm.rave_joint_inds,
                                       pos_mat, 
                                       'base_link',MAX_ERROR=0.005,MAX_NUM_ITERS=5000)
        Joints.append(j.tolist())        


    print Joints
    bot.larm.follow_joint_trajectory(Joints)
    rospy.sleep(0.5)
    bot.larm.controller_pub.publish(tm.JointTrajectory())
    #bot.larm.follow_joint_trajectory([j,j,j])
#    bot.larm.goto_joint_positions(j.tolist())
#    rospy.sleep(0.3)


    
    

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
    rospy.loginfo('done_twisting')
    rospy.spin()
