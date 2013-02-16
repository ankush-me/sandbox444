import sys, os, re, logging, signal
from multiprocessing import Process,Pipe
from threading import Thread
from openravepy import *
import numpy as np

from EasyPR2 import EasyPR2


class ORServer(object):
    '''
    Server to receive [and send] commands from [to] Qt GUI.  
    '''
    
    def __del__(self):
        RaveDestroy()
    
    def __init__(self, pipe):
        self.pipe    =  pipe
        self.running =  True
        self.pr2     =  EasyPR2()
        self.f = True
        self._run()
        

    def _run(self):
        while(self.running):
            (functionName,args) = self.pipe.recv()
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
            self.pr2.env.SetViewer('qtcoin')
            return True,None
        except:
            pass
        return None,"OpenRave environment not up!"

    
    def SetJoints(self,joints):
        if self.f:
            self.f = False
        joints = eval(joints)
        self.pr2.set_joints(joints)
        
        return True, "Joints set."
