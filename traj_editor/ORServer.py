from openravepy import *
from ravens import Ravens
import cPickle

class ORServer(object):
    '''
    Server to receive [and send] commands from [to] Qt GUI.  
    '''
    
    def __del__(self):
        RaveDestroy()
    
    def __init__(self, pipe):
        self.pipe    = pipe
        self.running = True
        self.robot   = Ravens()
        self.__run__()

    def __run__(self):
        while(self.running):
            functionName,args = self.pipe.recv()
            self.executeFunction(functionName, args)

    def Stop(self):
        self.running = False
        return None,"Stopping!!!"


    def executeFunction(self,name,args):  
        rValue = None
        rMessage = "Function with " + name + " not available"
        
        if name in dir(self):
            if(args is None):
                rValue,rMessage = getattr(self,name)()
            else:
                rValue,rMessage = getattr(self,name)(args)
        return rValue,rMessage


    def StartViewer(self):
        try:
            self.robot.env.SetViewer('qtcoin')
            return True,None
        except:
            pass
        return None,"OpenRave environment not up!"

    
    def SetJoints(self,joints):
        joints = cPickle.loads(joints)
        self.robot.set_joints(joints)       
        return True, "Joints set."
    

    def PlotPoints(self, pts):
        pts = cPickle.loads(pts)
        
