from openravepy import *
import numpy as np
import time

env = Environment()
env.SetViewer('qtcoin')
env.Load('/home/ankush/sandbox/rave_suture/suture_env.zae')

robot   = env.GetRobots()[0]
cloth   = env.GetBodies()[1]
table   = env.GetBodies()[2]
    
RaveSetDebugLevel(DebugLevel.Debug)


manip = robot.GetActiveManipulator()
ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterization.Type.Translation3D)
print("Getting the ik database..")
if not ikmodel.load():
    print("Generating pr2 ik database.")
    ikmodel.autogenerate()
else:
    print("Found ik database.")

with robot: # lock environment and save robot state
    robot.SetDOFValues([2.58, 0.547, 1.5, -0.7],[0,1,2,3]) # set the first 4 dof values
    Tee = manip.GetEndEffectorTransform() # get end effector
    ikparam = IkParameterization(Tee[0:3,3],ikmodel.iktype) # build up the translation3d ik query
    sols = manip.FindIKSolutions(ikparam, IkFilterOptions.CheckEnvCollisions) # get all solutions

h = env.plot3(Tee[0:3,3],10) # plot one point
with robot: # save robot state
    raveLogInfo('%d solutions'%len(sols))
    for sol in sols: # go through every solution
        robot.SetDOFValues(sol,manip.GetArmIndices()) # set the current solution
        env.UpdatePublishedBodies() # allow viewer to update new robot
        time.sleep(10.0/len(sols))

raveLogInfo('restored dof values: '+repr(robot.GetDOFValues())) # robot state is restored to original

"""
pr2.SetActiveManipulator('leftarm_torso') # set the manipulator to leftarm + torso
ikmodel = databases.inversekinematics.InverseKinematicsModel(pr2,iktype=IkParameterization.Type.Transform6D)
if not ikmodel.load():
    ikmodel.autogenerate()

manipprob = interfaces.BaseManipulation(pr2) # create the interface for basic manipulation programs
Tgoal = numpy.array([[0,-1,0,-0.21],[-1,0,0,0.04],[0,0,-1,0.92],[0,0,0,1]])
res = manipprob.MoveToHandPosition(matrices=[Tgoal],seedik=10) # call motion planner with goal joint angles
pr2.WaitForController(0) # wait
"""

while True:
    time.sleep(0.01)
