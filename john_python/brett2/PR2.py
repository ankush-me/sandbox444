
import numpy as np
__builtins__["__openravepy_version__"] = '0.8'
import openravepy as rave
from numpy import inf, zeros, dot, r_
from numpy.linalg import norm, inv
import roslib
import jds_utils.conversions as conv
from jds_utils.func_utils import once
from kinematics import retiming
import kinematics.kinematics_utils as ku
from brett2 import ros_utils
from threading import Thread
import brett2
import os.path as osp
import jds_utils.math_utils as mu

import trajectory_msgs.msg as tm
import sensor_msgs.msg as sm
import pr2_controllers_msgs.msg as pcm
import actionlib
import rospy
import geometry_msgs.msg as gm
import move_base_msgs.msg as mbm

try:
    from bulletsim_msgs.srv import PlanTraj, PlanTrajRequest, PlanTrajResponse
except ImportError:
    print "could not import bulletsim_msgs needed for planning"


VEL_RATIO = .1
ACC_RATIO = .15

class IKFail(Exception):
    pass

class TopicListener(object):
    "stores last message"
    last_msg = None
    def __init__(self,topic_name,msg_type):
        self.sub = rospy.Subscriber(topic_name,msg_type,self.callback)        

        rospy.loginfo('waiting for the first message: %s'%topic_name)
        while self.last_msg is None: rospy.sleep(.01)
        rospy.loginfo('ok: %s'%topic_name)

    def callback(self,msg):
        self.last_msg = msg


class JustWaitThread(Thread):

    def __init__(self, duration):
        Thread.__init__(self)
        self.wants_exit = False
        self.duration = duration

    def run(self):
        t_done = rospy.Time.now() + rospy.Duration(self.duration)        
        while not self.wants_exit and rospy.Time.now() < t_done and not rospy.is_shutdown():
            rospy.sleep(.01)


class GripperTrajectoryThread(Thread):

    def __init__(self, gripper, times, angles):
        Thread.__init__(self)
        self.wants_exit = False
        self.gripper = gripper
        self.times = times
        self.angles = angles

    def run(self):
        t_start = rospy.Time.now()
        self.gripper.set_angle_target(self.angles[0])
        for i in xrange(1,len(self.times)):
            if rospy.is_shutdown() or self.wants_exit: break          
            duration_elapsed = rospy.Time.now() - t_start

            target_angle = 0.08*((self.angles[i]/0.08)**3) #hack
            self.gripper.set_angle_target(target_angle)
                
            rospy.sleep(rospy.Duration(self.times[i]) - duration_elapsed)            
            #duration_elapsed = rospy.Time.now() - t_start
            #print "%s %.4e, %.4e, %.4e"%(self.gripper.controller_name, self.angles[i], self.angles[i] - self.gripper.get_angle(),(rospy.Duration(self.times[i]) - duration_elapsed).to_sec())


class PR2(object):

    pending_threads = []
    wait = True # deprecated way of blocking / not blocking

    @once
    def create(rave_only=False):
        return PR2(rave_only)

    def __init__(self, rave_only=False):
        # set up openrave
        self.env = rave.Environment()
        self.env.Load("/home/ankush/sandbox/robots/pr2-beta-static.zae") # todo: use up-to-date urdf

        self.robot = self.env.GetRobots()[0]  

        if not rave_only:
            self.joint_listener = TopicListener("/joint_states", sm.JointState)
            self.tf_listener = ros_utils.get_tf_listener()

            self.planner = rospy.ServiceProxy("/plan_traj", PlanTraj)     

            # rave to ros conversions
            joint_msg = self.get_last_joint_message()
            ros_names = joint_msg.name                
            inds_ros2rave = np.array([self.robot.GetJointIndex(name) for name in ros_names])
            self.good_ros_inds = np.flatnonzero(inds_ros2rave != -1) # ros joints inds with matching rave joint
            self.rave_inds = inds_ros2rave[self.good_ros_inds] # openrave indices corresponding to those joints
            self.update_rave()

            self.larm = Arm(self, "l")
            self.rarm = Arm(self, "r")
            self.lgrip = Gripper(self, "l")
            self.rgrip = Gripper(self, "r")

            self.head = Head(self)
            self.torso = Torso(self)
            self.base = Base(self)
            rospy.on_shutdown(self.stop_all)
        

    def start_thread(self, thread):
        self.pending_threads.append(thread)
        thread.start()

    def get_last_joint_message(self):
        return self.joint_listener.last_msg
    def update_rave(self, use_map = False):
        ros_values = self.get_last_joint_message().position        
        rave_values = [ros_values[i_ros] for i_ros in self.good_ros_inds]        
        self.robot.SetJointValues(rave_values[:20],self.rave_inds[:20])
        self.robot.SetJointValues(rave_values[20:],self.rave_inds[20:])   

        if use_map:
            (trans,rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))            
            self.robot.SetTransform(conv.trans_rot_to_hmat(trans, rot))            

    def update_rave_without_ros(self, joint_vals):
        self.robot.SetJointValues(joint_vals)

    def join_all(self):
        for thread in self.pending_threads:
            thread.join()
        self.pending_threads = []

    def stop_all(self):
        self.larm.stop()
        self.rarm.stop()
        self.head.stop()
        self.torso.stop()
        for thread in self.pending_threads:
            thread.wants_exit = True


class TrajectoryControllerWrapper(object):

    def __init__(self, pr2, controller_name):
        self.pr2 = pr2
        self.controller_name = controller_name

        self.joint_names = rospy.get_param("/%s/joints"%controller_name)

        self.n_joints = len(self.joint_names)

        msg = self.pr2.get_last_joint_message()
        self.ros_joint_inds = [msg.name.index(name) for name in self.joint_names]
        self.rave_joint_inds = [pr2.robot.GetJointIndex(name) for name in self.joint_names]

        self.controller_pub = rospy.Publisher("%s/command"%controller_name, tm.JointTrajectory)        

        all_vel_limits = self.pr2.robot.GetDOFVelocityLimits()
        self.vel_limits = np.array([all_vel_limits[i_rave]*VEL_RATIO for i_rave in self.rave_joint_inds])
        all_acc_limits = self.pr2.robot.GetDOFAccelerationLimits()
        self.acc_limits = np.array([all_acc_limits[i_rave]*ACC_RATIO for i_rave in self.rave_joint_inds])

    def get_joint_positions(self):
        msg = self.pr2.get_last_joint_message()
        return np.array([msg.position[i] for i in self.ros_joint_inds])


    def goto_joint_positions(self, positions_goal):

        positions_cur = self.get_joint_positions()
        assert len(positions_goal) == len(positions_cur)

        duration = norm((r_[positions_goal] - r_[positions_cur])/self.vel_limits, ord=inf)

        jt = tm.JointTrajectory()
        jt.joint_names = self.joint_names
        jt.header.stamp = rospy.Time.now()

        jtp = tm.JointTrajectoryPoint()
        jtp.positions = positions_goal
        jtp.velocities = zeros(len(positions_goal))
        jtp.time_from_start = rospy.Duration(duration)

        jt.points = [jtp]
        self.controller_pub.publish(jt)

        rospy.loginfo("%s: starting %.2f sec traj", self.controller_name, duration)
        self.pr2.start_thread(JustWaitThread(duration))

    def goto_joint_positions_planned(self, jpos):


        req = PlanTrajRequest()
        req.start_joints = self.get_joint_positions()
        req.end_joints = jpos
        req.side = self.lr
        req.joint_states = self.pr2.joint_listener.last_msg

        rospy.loginfo("planning...")
        
        resp = self.pr2.planner.call(req)
        rospy.loginfo("done planning!")
        assert isinstance(resp, PlanTrajResponse)
        traj = np.asarray(resp.trajectory).reshape(-1,7)
        self.follow_joint_trajectory(traj)            

    def follow_joint_trajectory(self, traj):
        traj = np.r_[np.atleast_2d(self.get_joint_positions()), traj]
        traj[:,4] = np.unwrap(traj[:,4])
        traj[:,6] = np.unwrap(traj[:,6])

        times = retiming.retime_with_vel_limits(traj, self.vel_limits)
        times_up = np.arange(0,times[-1],.1)
        traj_up = mu.interp2d(times_up, times, traj)
        vels = ku.get_velocities(traj_up, times_up, .001)
        self.follow_timed_joint_trajectory(traj_up, vels, times_up)


    def follow_timed_joint_trajectory(self, positions, velocities, times):

        jt = tm.JointTrajectory()
        jt.joint_names = self.joint_names
        jt.header.stamp = rospy.Time.now()

        for (position, velocity, time) in zip(positions, velocities, times):
            jtp = tm.JointTrajectoryPoint()
            jtp.positions = position
            jtp.velocities = velocity
            jtp.time_from_start = rospy.Duration(time)
            jt.points.append(jtp)

        self.controller_pub.publish(jt)
        rospy.loginfo("%s: starting %.2f sec traj", self.controller_name, times[-1])
        self.pr2.start_thread(JustWaitThread(times[-1]))

    def stop(self):
        jt = tm.JointTrajectory()
        jt.joint_names = self.joint_names
        jt.header.stamp = rospy.Time.now()
        self.controller_pub.publish(jt)


def mirror_arm_joints(x):
    "mirror image of joints (r->l or l->r)"
    
    return r_[-x[0],x[1],-x[2],x[3],-x[4],x[5],-x[6]]


def transform_relative_pose_for_ik(manip, matrix4, ref_frame, targ_frame):
    robot = manip.GetRobot()


    if ref_frame == "world":
        worldFromRef = np.eye(4)
    else:
        ref = robot.GetLink(ref_frame)
        worldFromRef = ref.GetTransform()

    if targ_frame == "end_effector":        
        targFromEE = np.eye(4)
    else:
        targ = robot.GetLink(targ_frame)
        worldFromTarg = targ.GetTransform()
        worldFromEE = manip.GetEndEffectorTransform()    
        targFromEE = dot(inv(worldFromTarg), worldFromEE)       

    refFromTarg_new = matrix4
    worldFromEE_new = dot(dot(worldFromRef, refFromTarg_new), targFromEE)    

    return worldFromEE_new

def cart_to_joint(manip, matrix4, ref_frame, targ_frame, filter_options = 0):
    robot = manip.GetRobot()
    worldFromEE = transform_relative_pose_for_ik(manip, matrix4, ref_frame, targ_frame)        
    joint_positions = manip.FindIKSolution(worldFromEE, filter_options)
    if joint_positions is None: return joint_positions
    current_joints = robot.GetDOFValues(manip.GetArmIndices())
    joint_positions_unrolled = ku.closer_joint_angles(joint_positions, current_joints)
    return joint_positions_unrolled



class Arm(TrajectoryControllerWrapper):

    L_POSTURES = dict(        
        untucked = [0.4,  1.0,   0.0,  -2.05,  0.0,  -0.1,  0.0],
        tucked = [0.06, 1.25, 1.79, -1.68, -1.73, -0.10, -0.09],
        up = [ 0.33, -0.35,  2.59, -0.15,  0.59, -1.41, -0.27],
        side = [  1.832,  -0.332,   1.011,  -1.437,   1.1  ,  -2.106,  3.074]
    )    

    def __init__(self, pr2, lr):
        TrajectoryControllerWrapper.__init__(self,pr2, "%s_arm_controller"%lr)
        self.lr = lr
        self.lrlong = {"r":"right", "l":"left"}[lr]
        self.tool_frame = "%s_gripper_tool_frame"%lr

        self.manip = pr2.robot.GetManipulator("%sarm"%self.lrlong)
        self.cart_command = rospy.Publisher('%s_cart/command_pose'%lr, gm.PoseStamped)


    def goto_posture(self, name):
        l_joints = self.L_POSTURES[name]        
        joints = l_joints if self.lr == 'l' else mirror_arm_joints(l_joints)
        self.goto_joint_positions(joints)

    def goto_joint_positions(self, positions_goal):
        positions_cur = self.get_joint_positions()   
        positions_goal = ku.closer_joint_angles(positions_goal, positions_cur)
        TrajectoryControllerWrapper.goto_joint_positions(self, positions_goal)
        
    def set_cart_target(self, quat, xyz, ref_frame):
        ps = gm.PoseStamped()
        ps.header.frame_id = ref_frame
        ps.header.stamp = rospy.Time(0)
        ps.pose.position = gm.Point(xyz[0],xyz[1],xyz[2])
        ps.pose.orientation = gm.Quaternion(*quat)
        self.cart_command.publish(ps)

    def cart_to_joint(self, matrix4, ref_frame, targ_frame, filter_options = 0):
        self.pr2.update_rave()
        return cart_to_joint(self.manip, matrix4, ref_frame, targ_frame, filter_options)

    def goto_pose_matrix(self, matrix4, ref_frame, targ_frame, filter_options = 0): 
        """
        IKFO_CheckEnvCollisions = 1
        IKFO_IgnoreSelfCollisions = 2
        IKFO_IgnoreJointLimits = 4
        IKFO_IgnoreCustomFilters = 8
        IKFO_IgnoreEndEffectorCollisions = 16
        """                               
        self.pr2.update_rave()
        joint_positions = cart_to_joint(self.manip, matrix4, ref_frame, targ_frame, filter_options)
        if joint_positions is not None: self.goto_joint_positions(joint_positions)
        else: raise IKFail

    def get_pose_matrix(self, ref_frame, targ_frame):
        self.pr2.update_rave()

        worldFromRef = self.pr2.robot.GetLink(ref_frame).GetTransform()
        worldFromTarg = self.pr2.robot.GetLink(targ_frame).GetTransform()
        refFromTarg = dot(inv(worldFromRef), worldFromTarg)

        return refFromTarg


class Head(TrajectoryControllerWrapper):
    def __init__(self, pr2):
        TrajectoryControllerWrapper.__init__(self,pr2,"head_traj_controller")

    def set_pan_tilt(self, pan, tilt):
        self.goto_joint_positions([pan, tilt])
    def look_at(self, xyz_target, reference_frame, camera_frame):
        self.pr2.update_rave()

        worldFromRef = self.pr2.robot.GetLink(reference_frame).GetTransform()
        worldFromCam = self.pr2.robot.GetLink(camera_frame).GetTransform()
        refFromCam = dot(inv(worldFromRef), worldFromCam)        

        xyz_cam = refFromCam[:3,3]
        ax = xyz_target - xyz_cam # pointing axis
        pan = np.arctan(ax[1]/ax[0])
        tilt = np.arcsin(-ax[2] / norm(ax))
        self.set_pan_tilt(pan,tilt)


class Torso(TrajectoryControllerWrapper):
    def __init__(self,pr2):
        TrajectoryControllerWrapper.__init__(self,pr2, "torso_controller")
        self.torso_client = actionlib.SimpleActionClient('torso_controller/position_joint_action', pcm.SingleJointPositionAction)
        self.torso_client.wait_for_server()

    def set_height(self, h):
        self.torso_client.send_goal(pcm.SingleJointPositionGoal(position = h))
        self.torso_client.wait_for_result() # todo: actually check result
    # def goto_joint_positions(self, positions_goal):
    #     self.set_height(positions_goal[0])
    def go_up(self):
        self.set_height(.3)
    def go_down(self):
        self.set_height(.02)        


class Gripper(object):
    default_max_effort = -1.0
    def __init__(self,pr2,lr):

        assert isinstance(pr2, PR2)
        self.pr2 = pr2
        self.lr = lr
        self.controller_name = "%s_gripper_controller"%self.lr
        self.joint_names = [rospy.get_param("/%s/joint"%self.controller_name)]
        self.n_joints = len(self.joint_names)

        msg = self.pr2.get_last_joint_message()
        self.ros_joint_inds = [msg.name.index(name) for name in self.joint_names]
        self.rave_joint_inds = [pr2.robot.GetJointIndex(name) for name in self.joint_names]

        self.controller_pub = rospy.Publisher("%s/command"%self.controller_name, pcm.Pr2GripperCommand)        
        self.grip_client = actionlib.SimpleActionClient('%s_gripper_controller/gripper_action'%lr, pcm.Pr2GripperCommandAction)
        self.grip_client.wait_for_server()
        self.vel_limits = [0.04]  # was 0.033 HACK
        self.acc_limits = [inf]


        self.diag_pub = rospy.Publisher("/%s_gripper_traj_diagnostic"%lr, tm.JointTrajectory)

        try:
            with open(osp.join(osp.dirname(brett2.__file__),"data/%s_closed_val"%lr),"r") as fh:
                self.closed_angle = float(fh.read())
        except IOError:
            rospy.logwarn("couldn't get closed val. setting to 0")
            self.closed_angle = 0

    def set_angle(self, a, max_effort = default_max_effort):
        self.grip_client.send_goal(pcm.Pr2GripperCommandGoal(pcm.Pr2GripperCommand(position=a,max_effort=max_effort)))
        self.pr2.start_thread(Thread(target=self.grip_client.wait_for_result))
    def open(self, max_effort=default_max_effort):
        self.set_angle(.08, max_effort = max_effort)
    def close(self,max_effort=default_max_effort):
        self.set_angle(-.01, max_effort = max_effort)        
    def is_closed(self): # (and empty)
        return self.get_angle() <= self.closed_angle + .0025
    def set_angle_target(self, position, max_effort = default_max_effort):
        self.controller_pub.publish(pcm.Pr2GripperCommand(position=position,max_effort=max_effort))
    def follow_timed_trajectory(self, times, angs):
        times_up = np.arange(0,times[-1],.1)
        angs_up = np.interp(times_up,times,angs)

        jt = tm.JointTrajectory()
        jt.header.stamp = rospy.Time.now()
        jt.joint_names = ["%s_gripper_joint"%self.lr]
        for (t,a) in zip(times, angs):
            jtp = tm.JointTrajectoryPoint()
            jtp.time_from_start = rospy.Duration(t)
            jtp.positions = [a]
            jt.points.append(jtp)
        self.diag_pub.publish(jt)

        self.pr2.start_thread(GripperTrajectoryThread(self, times_up, angs_up))
        #self.pr2.start_thread(GripperTrajectoryThread(self, times, angs))

    def get_angle(self):
        return self.pr2.joint_listener.last_msg.position[self.ros_joint_inds[0]]

    def get_velocity(self):
        return self.pr2.joint_listener.last_msg.velocity[self.ros_joint_inds[0]]

    def get_effort(self):
        return self.pr2.joint_listener.last_msg.effort[self.ros_joint_inds[0]]


class Base(object):

    def __init__(self, pr2):
        self.pr2 = pr2
        self.action_client = actionlib.SimpleActionClient('move_base',mbm.MoveBaseAction)
        self.command_pub = rospy.Publisher('base_controller/command', gm.Twist)        
        self.traj_pub = rospy.Publisher("base_traj_controller/command", tm.JointTrajectory)

    def goto_pose(self, x, y, theta, frame_id):
        trans,rot = conv.xya_to_trans_rot([x,y,theta])
        pose = conv.trans_rot_to_pose(trans, rot)
        ps = gm.PoseStamped()
        ps.pose = pose
        ps.header.frame_id = frame_id
        ps.header.stamp = rospy.Time(0)

        goal = mbm.MoveBaseGoal()
        goal.target_pose = ps
        rospy.loginfo('Sending move base goal')
        finished = self.action_client.send_goal_and_wait(goal)
        rospy.loginfo('Move base action returned %d.' % finished)
        return finished 

    def get_pose(self, ref_frame):
        trans, rot =  self.pr2.tf_listener.lookupTransform(ref_frame,"/base_footprint",rospy.Time(0))
        return (trans[0], trans[1], conv.quat_to_yaw(rot))

    def set_twist(self,xya):
        vx, vy, omega = xya
        twist = gm.Twist()
        twist.linear.x = vx
        twist.linear.y = vy
        twist.angular.z = omega
        self.command_pub.publish(twist)

    def follow_timed_trajectory(self, times, xyas):
        jt = tm.JointTrajectory()
        n = len(xyas)
        assert len(times) == n
        for i in xrange(n):            
            jtp = tm.JointTrajectoryPoint()
            jtp.time_from_start = rospy.Duration(times[i])
            jtp.positions = xyas[i]
            jt.points.append(jtp)            
        self.traj_pub.publish(jt)    
        
