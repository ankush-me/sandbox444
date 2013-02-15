import numpy as np
#__builtins__["__openravepy_version__"] = '0.8'
import openravepy as rave
import kinematics.kinematics_utils as ku

class EasyPR2(object):

    def __init__(self, rave_only=False):
        self.env = rave.Environment()
        self.env.Load("robots/pr2-beta-static.zae") # todo: use up-to-date urdf
        #self.env.SetViewer('qtcoin')
        self.robot = self.env.GetRobots()[0]  
        
        self.lmanip = self.robot.GetManipulator("leftarm")
        self.larm_indices = self.lmanip.GetArmIndices()
        self.lgripper_indices = self.lmanip.GetGripperIndices()
        
        self.rmanip = self.robot.GetManipulator("rightarm")
        self.rarm_indices = self.rmanip.GetArmIndices()
        self.rgripper_indices = self.rmanip.GetGripperIndices()
            

    def set_joints(self, joints):

        j = self.robot.GetDOFValues()
        
        rarm_curr = j[self.rarm_indices]
        rarm_goal = joints[0]
        rarm_goal = ku.closer_joint_angles(rarm_goal, rarm_curr)
        
        larm_curr = j[self.larm_indices]
        larm_goal = joints[1]
        larm_goal = ku.closer_joint_angles(larm_goal, larm_curr)
        
        j[self.larm_indices]     = larm_goal
        j[self.lgripper_indices] = joints[3]
        j[self.rarm_indices]     = rarm_goal
        j[self.rgripper_indices] = joints[2]

        with self.env:
            self.robot.SetJointValues(j)
        


if __name__=='__main__':
    p  = EasyPR2()
    p.env.SetViewer('QtCoin')

    path  = '/home/ankush/Downloads/anchoringtrajs/full3'
    larm_traj  = np.load(path + '_larm.npy')
    rarm_traj  = np.load(path + '_rarm.npy')
    lgrip_traj = np.load(path + '_lgrip.npy')
    rgrip_traj = np.load(path + '_rgrip.npy')
        
    length = len(larm_traj)
    full_traj = np.zeros(length, dtype=[('r_arm', '7float64'), ('l_arm', '7float64'), ('r_gripper', 'float64'), ('l_gripper', 'float64')])

    full_traj['r_arm']     = rarm_traj
    full_traj['l_arm']     = larm_traj
    full_traj['r_gripper'] = rgrip_traj
    full_traj['l_gripper'] = lgrip_traj

    import time
    count  = 0
    for j in full_traj:
        p.set_joints(j)
        time.sleep(0.001)
