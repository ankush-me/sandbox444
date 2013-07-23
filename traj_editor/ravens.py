import openravepy as rave

class Ravens(object):
    
    def __init__(self, rave_only=False):
        self.env = rave.Environment()
        self.env.Load("./models/ravens.env.xml")
        self.robot = self.env.GetRobots()[0]  
        
        self.lmanip = self.robot.GetManipulator("l_arm")
        self.larm_indices = self.lmanip.GetArmIndices()
        self.lgripper_indices = self.lmanip.GetChildDOFIndices()
        
        self.rmanip = self.robot.GetManipulator("r_arm")
        self.rarm_indices = self.rmanip.GetArmIndices()
        self.rgripper_indices = self.rmanip.GetChildDOFIndices()
            

    def set_joints(self, joints):
        assert len(joints)==16, "Not enough joints passed to ravens."
        with self.env:
            self.robot.SetJointValues(joints)
