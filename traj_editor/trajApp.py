from PyQt4          import QtCore, QtGui

import sys, os, re, logging, signal, time
from numpy import random
from multiprocessing import Process,Pipe
from threading import Thread


from traj_editor_ui import Ui_MainWindow
import numpy as np
from ListInteractor import ListInteractor
from EasyPR2        import EasyPR2
from ProcessStarter import *

    

class trajApp(QtGui.QMainWindow,Ui_MainWindow):
    
    @QtCore.pyqtSignature("")
    def on_pbClose_clicked(self):
        self.closeAll()
    
    def __init__(self, pipeOR):
        super(trajApp,self).__init__(None)
        self.pipeOR = pipeOR        
        self.setupUi(self)
        self.syncList = ListInteractor(self.trajList)
        self.addSlots()
        

    def addSlots(self):
        QtCore.QObject.connect(self.playButton,         QtCore.SIGNAL("clicked()"), self.clicked_playButton)
        QtCore.QObject.connect(self.playSelectedButton, QtCore.SIGNAL("clicked()"), self.clicked_playSelectedButton)
        QtCore.QObject.connect(self.removeButton,       QtCore.SIGNAL("clicked()"), self.clicked_removeButton)
        QtCore.QObject.connect(self.addButton,          QtCore.SIGNAL("clicked()"), self.clicked_addButton)
        QtCore.QObject.connect(self.exportButton,       QtCore.SIGNAL("clicked()"), self.clicked_exportButton)
        QtCore.QObject.connect(self.downButton,         QtCore.SIGNAL("clicked()"), self.clicked_downButton)
        QtCore.QObject.connect(self.copyButton,         QtCore.SIGNAL("clicked()"), self.clicked_copyButton)
        QtCore.QObject.connect(self.upButton,           QtCore.SIGNAL("clicked()"), self.clicked_upButton)
        
        QtCore.QObject.connect(self.startSlider,    QtCore.SIGNAL("sliderMoved(int)"), self.moved_startSlider)
        QtCore.QObject.connect(self.endSlider,      QtCore.SIGNAL("sliderMoved(int)"), self.moved_endSlider)
        QtCore.QObject.connect(self.playSlider,     QtCore.SIGNAL("sliderMoved(int)"), self.moved_playSlider)


        QtCore.QObject.connect(self.trajList,       QtCore.SIGNAL("currentRowChanged(int)"), self.changed_trajList)


    def changed_trajList(self, row):
        if row != -1:
            trajItem    = self.syncList.itemList[row]
            self.startSlider.setMinimum(0)
            self.startSlider.setMaximum(trajItem.length-1)

            self.endSlider.setMinimum(0)
            self.endSlider.setMaximum(trajItem.length-1)
            
            self.playSlider.setMinimum(trajItem.start)
            self.playSlider.setMaximum(trajItem.end)
            

    def clicked_playButton(self):
        for i in range(0, self.syncList.length()):
            trajItem = self.syncList.itemList[i]
            (k, joints)  = self.syncList.trajData[trajItem.getStr()]
            jointsSender = JointsPusher(self.pipeOR, joints[trajItem.start:trajItem.end+1,:]) 
            jointsSender.start()


    def clicked_playSelectedButton(self):
        selection = self.trajList.currentRow()
        if selection >=0 and selection < self.syncList.length():
            trajItem    = self.syncList.itemList[selection]
            (k, trajectory) = self.syncList.trajData[trajItem.getStr()] 
            jointsSender = JointsPusher(self.pipeOR, trajectory[trajItem.start:trajItem.end+1,:]) 
            jointsSender.start()
                
        

    def clicked_removeButton(self):
        if self.syncList.length() > 0:
            self.syncList.removeItem(self.trajList.currentRow())
    

    def clicked_addButton(self):
        paths = self.dialog.getOpenFileNames(caption='Add Trajectory')
        self.addTrajectories(paths)
        

    def clicked_exportButton(self):
        pass
    
    
    def clicked_downButton(self):
        selection = self.trajList.currentRow()
        if selection >=0 and selection < self.syncList.length()-1:
            item     = self.trajList.takeItem(selection)
            self.trajList.insertItem(selection+1, item)
            self.syncList.itemList[selection], self.syncList.itemList[selection+1] = self.syncList.itemList[selection+1], self.syncList.itemList[selection]
            self.trajList.setCurrentRow(selection+1)  


    def clicked_upButton(self):
        selection = self.trajList.currentRow()
        if selection > 0 and selection < self.syncList.length():
            item     = self.trajList.takeItem(selection)
            self.trajList.insertItem(selection-1, item)
            self.syncList.itemList[selection], self.syncList.itemList[selection-1] = self.syncList.itemList[selection-1], self.syncList.itemList[selection]  
            self.trajList.setCurrentRow(selection-1)  
           

    def clicked_copyButton(self):
        selection = self.trajList.currentRow()
        if selection >= 0 and selection < self.syncList.length():
            item  = self.syncList.itemList[selection]
            trajectoryItem(self.syncList, item.prefix, item.path, copy=True)


    def updateJoints(self, tickPos):
        selection = self.trajList.currentRow()
        if selection >=0:
            trajItem        = self.syncList.itemList[selection]
            trajectory     = trajItem.getTrajectory()
            jointsSender    = JointsPusher(self.pipeOR, [trajectory[tickPos]]) 
            jointsSender.start()

        
    def moved_startSlider(self, pos):
        selection = self.trajList.currentRow()
        if selection >= 0:
            
            item  = self.syncList.itemList[selection]
        
            endPos = item.length - 1 - self.endSlider.value()
            if pos >= endPos:
                pos = max(0,endPos-1)
                self.startSlider.setValue(pos)

            item.start = max(0,pos)
            self.playSlider.setMinimum(item.start)
            

    def moved_endSlider(self, pos):
        selection = self.trajList.currentRow()
        if selection >= 0:
            item  = self.syncList.itemList[selection]
            
            startPos = self.startSlider.value()
            p  = item.length-1-pos
            if p < startPos:
                pos = max(0,item.length-2-startPos)
                self.endSlider.setValue(pos)

            item.end = item.length -1 - pos
            self.playSlider.setMaximum(min(item.end, item.length-1))
            

    def moved_playSlider(self, pos):
        self.updateJoints(pos)

    
    def addTrajectories(self, paths):
        """
        Extract the prefixes and the paths and add them to the list
        """
        s = set()
        for t in paths:
            p = str(t)
            path   = p[ : p.rfind('/')+1]
            prefix = p[p.rfind('/')+1 : p.rfind('_')]
            s.add((path, prefix))
        for (path, prefix) in s:
            trajectoryItem(self.syncList, prefix, path)
          
    
class trajectoryItem:

    def __init__(self, listObj, traj_name_prefix, path='', copy=False):
        self.path    = path
        self.prefix  = traj_name_prefix
        self.info    = ''
        self.copy    = copy
        self.listObj = listObj
        self.length  = -1
        self.start   = -1
        self.end     = -1
        self.qtItem  = None
        listObj.addItem(self)
        
    def getTrajectory(self):
        (k, traj) = self.listObj.trajData[self.getStr()]
        return traj
            
    
    def getDisplayName(self):
        return self.qtItem.text()
    
    def getStr(self):
        return self.path+self.prefix


class JointsPusher(Thread):
    allPushers = set()
    def __init__(self, pipe, joints, delay=0.01):
        Thread.__init__(self)
        self.pipe = pipe
        self.joints = joints
        self.delay  = delay
        self.daemon = True
        self.stop   = False
        JointsPusher.allPushers.add(self)
        
    def run(self):
        for p in JointsPusher.allPushers:
            p.stop = True
        self.stop = False

        for j in self.joints:
            self.pipe.send(['SetJoints', repr(j)])
            time.sleep(self.delay)
            if self.stop: break
            
        JointsPusher.allPushers.remove(self)

if __name__ == "__main__":
    ProcessStarter()
    
    
    
    
    
"""
ToDo:
1. Play selected should update the playSlider
2. Exporting to new files with interpolation
3. Playing the merged trajectory
4. Need a stop/ pause button
"""