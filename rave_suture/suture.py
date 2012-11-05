from openravepy import *
from numpy import *
import time

env = Environment()
env.SetViewer('qtcoin')
env.Load('/home/ankush/sandbox/rave_suture/suture_env2.zae')

robot   = env.GetRobots()[0]
cloth   = env.GetBodies()[1]
table   = env.GetBodies()[2]

# get the cloth trimesh location
cloth_aabb = cloth.ComputeAABB()
h1         = env.plot3(cloth_aabb.pos(), 10) # plot one point

RaveSetDebugLevel(DebugLevel.Debug)

for manip in robot.GetManipulators():
    print manip.GetName()

# Get/generate the ik database for leftarm+torso manipulator
robot.SetActiveManipulator('rightarm_torso')
manip = robot.GetActiveManipulator()
print("---------------------")
print manip.GetName()
print("---------------------")
ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,
                                                             iktype=IkParameterization.Type.Transform6D)
print("Getting the ik database..")
if not ikmodel.load():
    print("Generating pr2 ik database.")
    ikmodel.autogenerate()
else:
    print("Found ik database.")

goal_transform = manip.GetEndEffectorTransform()
#goal_transform[0,3] -= 0.05# np.array(cloth_aabb.pos())
#goal_transform[2,3] += 0.1# np.array(cloth_aabb.pos())
goal_transform[0:3,3] = cloth_aabb.pos()
goal_transform[2,3]   += 0.03
env.plot3(goal_transform[0:3,3], 100) # plot one point

#print goal_transform
#print manip.GetTransform()

ikparam = IkParameterization(goal_transform,IkParameterizationType.Transform6D)
sol = manip.FindIKSolution(ikparam, IkFilterOptions.CheckEnvCollisions)

if sol is not None:
    robot.SetDOFValues(sol,manip.GetArmIndices())
    print("!!!!!!!!!!! moved !!!!!!!!!!!!!")
else:
    print("***** No IK solution found *****")
        

while True:
    #print("\tsleeping..")
    time.sleep(0.01)
