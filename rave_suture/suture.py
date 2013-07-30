from openravepy import *
from numpy import *
import time


# class to move an arm
class arm:
    def __init__(self, pr2, lr='left'):
        self.lr = lr
        self.pr2 = pr2
        
        if (lr=='left'):
            self.manipName = 'leftarm'
        else:
            self.manipName = 'rightarm'

        self.pr2.SetActiveManipulator(self.manipName)
        self.manip = self.pr2.GetActiveManipulator()
        self.ikmodel6D = databases.inversekinematics.InverseKinematicsModel(self.pr2, iktype=IkParameterization.Type.Transform6D)
        
        #self.ikmodel3D = databases.inversekinematics.InverseKinematicsModel(self.pr2, iktype=IkParameterization.Type.Translation3D)

        if not self.ikmodel6D.load():
            print("Generating transform6D ik database for %s." % self.manipName)
            self.ikmodel6D.autogenerate()
        else:
            print("Found transform6D ik database for %s." % self.manipName)
        
        '''
        if not self.ikmodel3D.load():
            print("Generating translation3D ik database for %s." % self.manipName)
            self.ikmodel3D.autogenerate()
        else:
            print("Found translation3D ik database for %s." % self.manipName)
        '''

    ''' Goal is a 4x4 numpy matrix for end-effector transform '''
    def move_by_ik6D(self, goal):
        ikparam = IkParameterization(goal,IkParameterizationType.Transform6D)
        sol = self.manip.FindIKSolution(ikparam, IkFilterOptions.CheckEnvCollisions)
        return sol
    
        """
        if sol is not None:
            self.pr2.SetDOFValues(sol,self.manip.GetArmIndices())
            print("!!!!!!!!!!! moved !!!!!!!!!!!!!")
        else:
            print("***** No IK solution found *****")
        """

    ''' Goal is a 3x1 numpy matrix for end-effector translation
    def move_by_ik3D(self, goal):
        ikparam = IkParameterization(goal,IkParameterizationType.Translation3D)
        sol = self.manip.FindIKSolution(ikparam, IkFilterOptions.CheckEnvCollisions)
        if sol is not None:
            self.pr2.SetDOFValues(sol,self.manip.GetArmIndices())
            print("!!!!!!!!!!! moved !!!!!!!!!!!!!")
        else:
            print("***** No IK solution found *****")
   '''


env = Environment()
env.SetViewer('qtcoin')
env.Load('/home/ankush/sandbox/rave_suture/suture_env2.zae')
RaveSetDebugLevel(DebugLevel.Debug)

pr2     = env.GetRobots()[0]
cloth   = env.GetBodies()[1]
table   = env.GetBodies()[2]

rarm = arm(pr2, 'right')
larm = arm(pr2, 'left')

# get the cloth trimesh location
cloth_aabb = cloth.ComputeAABB()
h1         = env.plot3(cloth_aabb.pos(), 10) # plot one point

goal_transform = larm.manip.GetEndEffectorTransform()
goal_transform[0:3,3] = cloth_aabb.pos()
goal_transform[2,3]   += 0.1
env.plot3(goal_transform[0:3,3], 100) # plot one point

raw_input("press any key")
#larm.move_by_ik6D(goal_transform)

time.sleep(1)

goal2 = rarm.manip.GetEndEffectorTransform()
arm_pos = goal2[0:3,3]

taskmanip=interfaces.TaskManipulation(pr2)
basemanip=interfaces.BaseManipulation(pr2)

taskmanip.ReleaseFingers()
time.sleep(1)
taskmanip.CloseFingers()

print(len(pr2.GetActiveManipulator().GetArmIndices()))
sol = larm.move_by_ik6D(goal_transform)
print (len(sol))

env.StopSimulation()
basemanip.MoveManipulator(sol)

while(True):
    #env.LockPhysics(True)
    env.StepSimulation(0.01) # don't know what this line does
    raw_input('press key')
   # env.LockPhysics(False) 

time.sleep(1)
pr2.Grab(cloth)

# this thread must sleep to keep the viewer running
while True:
    time.sleep(0.01)
