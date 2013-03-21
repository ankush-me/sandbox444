from brett2.PR2 import PR2
#roslib.load_manifest("nav_msgs"); import nav_msgs.msg as nm
import trajectory_msgs.msg as tm
import numpy as np
import rospy
import scipy.interpolate as si
from Queue import Queue, Empty
from threading import Thread
import jds_utils.conversions as conv
import kinematics.kinematics_utils as ku
from time import time, sleep

class Spline2D(object):
    def __init__(self,x,y):
        self.Fs = [si.InterpolatedUnivariateSpline(x, ycol) for ycol in y.T]
    def __call__(self, x,nu=0):
        return np.array([F(x,nu=nu) for F in self.Fs]).T

class TrajectoryController:
    def __init__(self):        
        self.brett = PR2()        
        self.sub = rospy.Subscriber("base_traj_controller/command", tm.JointTrajectory, self.callback)
        self.q = Queue()
        self.F = None
        self.stop_requested = False
        self.ctrl_loop_running = False
    def callback(self, msg):
        joints = []
        ts = []
        for jtp in msg.points:
            joints.append((jtp.positions[0], jtp.positions[1], jtp.positions[2]))
            ts.append(jtp.time_from_start.to_sec())
        self.q.put( (np.array(joints),np.array(ts)) )
        
    def listen_loop(self):
        while not rospy.is_shutdown():
            try:
                joints, ts = self.q.get(timeout=.01)
                if self.ctrl_loop_running: self.stop_requested = True
                while self.ctrl_loop_running: sleep(.001)
                ctrl_thread = Thread(target = self.control_loop, args=(joints, ts))
                ctrl_thread.start()
                
            except Empty:
                pass                                                                
    def control_loop(self,joints, ts):
        raise
        
class BaseTrajectoryController(TrajectoryController):
    def control_loop(self,joints,ts):                
        print "running control loop with new trajectory"
        
        F = Spline2D(ts, joints)        
        
        t_start = time()        
        duration = ts[-1]
        
        while True:
            
            if rospy.is_shutdown(): 
                return
            if self.stop_requested:
                self.ctrl_loop_running = False
                return
            
            t_elapsed = time() - t_start
            if t_elapsed > duration+1: return
            
            pos_cur = self.brett.base.get_pose("odom_combined")
            
            if t_elapsed > duration: pos_targ = joints[-1]
            else: pos_targ = F(t_elapsed, nu = 0)
            
            pos_targ[2] = ku.closer_ang(pos_targ[2], pos_cur[2])                                                                
            twist = (pos_targ - pos_cur)
            twist *= .5 #np.linalg.norm(np.r_[.5,.5,1] / twist, np.inf)
            
            a = pos_cur[2]
            twist[0:2] = np.dot(
                np.array([[np.cos(a), np.sin(a)],
                          [-np.sin(a), np.cos(a)]]) ,
                twist[0:2])
            
            self.brett.base.set_twist(twist)
            sleep(.01)

    
if __name__ == "__main__":
    import rospy
    rospy.init_node("base_traj_controller", disable_signals = True)
    controller = BaseTrajectoryController()
    controller.listen_loop()