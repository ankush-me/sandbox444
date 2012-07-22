import numpy as np
import openravepy as rave
from numpy import inf, zeros, dot, r_, transpose
from numpy.linalg import norm, inv
import math

  
class GhostPR2(object):
    """
    Creates a GHOST PR2: an openrave instance of PR2.
    for simulation purposes.
    """
    def __init__(self):
        # set up openrave
        self.env = rave.Environment()
        self.env.Load("robots/pr2-beta-static.zae")
        self.robot = self.env.GetRobots()[0]

        self.larm = Arm(self, "l")
        self.rarm = Arm(self, "r")


    def update_rave(self, rave_values, rave_inds):
        """
        Update the rave's internal joints.
        @param:
        rave_values = Values of the joint angles.
        rave_inds   = Indices corresponding to the angle values.
        """
        self.robot.SetJointValues(rave_values,rave_inds)


    def get_arm(self, lr):
        """
        Returns the arm depending on L/R.
        @param:
        lr \in {'l', 'r'}
        """
        if lr == "l":
            return  self.larm
        else:
            return self.rarm


    def get_current_EE_pose(self, lr, ref_frame='base_footprint'):
        """
        Return the tool_frame of the arm LR, w.r.t. the ref_frame.
        @param:
        LR \in {'l', 'r'} : Left or right arm
        ref_frame         : reference frame.
        """
        arm = self.get_arm(lr)
        manip = arm.manip
        worldFromRef = manip.GetRobot().GetLink(ref_frame).GetTransform()
        worldFromTarg = manip.GetRobot().GetLink(arm.tool_frame).GetTransform()
        return dot(inv(worldFromRef), worldFromTarg)


    def get_joints_from_cart(self, lr, starting_joint_vals, joint_indices, target_mat,
                             ref_frame='base_footprint', alpha=0.01, MAX_NUM_ITERS=5000,
                             MAX_ERROR=0.005):
        """
        @params:
        1. lr            : STRING = "l" if left arm, "r" if right arm.
        2. starting_joint_vals : LIST = the current angle values of the joints.
        3. joint_indices : LIST = the indices (rave indices) to which the starting_joint_vals correspond to.
        4. target_mat    : NUMPY.NDARRAY : the desired target location (transformation matrix).
        5. ref_frame     : STRING = the reference frame in which the position is measured.
                           Set to "world", if it is the openrave world coordinate.
        6. alpha         : FLOAT  = Gradient Descent learning rate.
        7. MAX_NUM_ITERS : INTEGER= Maximum number of iterations.
        8. MAX_ERROR     : float  = Maximum allowed error.
        """
        init_str = "Pose Gradient Descent Initialized:"
        print "%s\n%s"%(init_str,'-'*len(init_str))

        arm_manipulator = self.get_arm(lr).manip
        self.update_rave(starting_joint_vals, joint_indices)

        goal_xyz = target_mat[0:3,3]
        current_xyz = self.get_current_EE_pose(lr, ref_frame)[0:3,3]

        error = norm(goal_xyz - current_xyz)
        print "Initial error = %f"%error
        joints = np.array(starting_joint_vals)

        num_iters = 0
        while error > MAX_ERROR and num_iters < MAX_NUM_ITERS:
            num_iters += 1

            pose_err = (current_xyz - goal_xyz)
            J = arm_manipulator.CalculateJacobian()
            joints -= alpha*dot(transpose(J), pose_err)  
            
            self.update_rave(joints, joint_indices)

            current_xyz = self.get_current_EE_pose(lr, ref_frame)[0:3,3]
            error = norm(goal_xyz - current_xyz)

        print "Took %d iterations." % num_iters
        print "Final error = %f\n------\n" % error
        return joints


class Arm(object):
    """
    Instances of arms: for openrave simulation
    """
    def __init__(self, pr2, lr):
        self.lr = lr
        self.lrlong = {"r":"right", "l":"left"}[lr]
        self.tool_frame = "%s_gripper_tool_frame"%lr
        self.manip = pr2.robot.GetManipulator("%sarm"%self.lrlong)


