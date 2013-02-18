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
from joints_utils import *
    

class trajApp(QtGui.QMainWindow,Ui_MainWindow):
    
    @QtCore.pyqtSignature("")
    def on_pbClose_clicked(self):
        self.closeAll()
    
    def __init__(self, pipeOR, processStarter):
        super(trajApp,self).__init__(None)
        self.pipeOR = pipeOR        
        self.setupUi(self)
        self.syncList = ListInteractor(self.trajList)
        self.processStarter = processStarter
        self.addSlots()


    def closeEvent(self, event):
        """
        Redefine the close event.
        """
        QtGui.QApplication.quit()
        self.processStarter.terminate()

        
    def addSlots(self):
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
            self.startSlider.setValue(trajItem.start)

            self.endSlider.setMinimum(0)
            self.endSlider.setMaximum(trajItem.length-1)
            self.endSlider.setValue(trajItem.length-1-trajItem.end)

            self.playSlider.setMinimum(trajItem.start)
            self.playSlider.setMaximum(trajItem.end)
            self.playSlider.setValue(trajItem.start)

            

    def setVisibilityModifiers(self, visible):
        self.addButton.setDisabled(visible)
        self.removeButton.setDisabled(visible)
        self.exportButton.setDisabled(visible)
        self.downButton.setDisabled(visible)
        self.upButton.setDisabled(visible)
        self.copyButton.setDisabled(visible)
        self.playButton.setDisabled(visible)
        self.startSlider.setDisabled(visible)                     
        self.endSlider.setDisabled(visible)                    
        self.trajList.setDisabled(visible)

    def clicked_playSelectedButton(self):
        caption = self.playSelectedButton.text()
        if caption=="Play Selected":
            self.playSelectedButton.setText("Pause")
            selection = self.trajList.currentRow()
            if selection >=0 and selection < self.syncList.length():
                self.setVisibilityModifiers(True)
                trajItem        = self.syncList.itemList[selection]
                jTask = JointsPusher(self.playSlider.value(), trajItem.end, self.playSlider, self)
                jTask.start()
        else:
            self.playSelectedButton.setText("Pause")
            jTask = JointsPusher(0, -1, self.playSlider, self)
            jTask.start()
        
        

    def clicked_removeButton(self):
        if self.syncList.length() > 0:
            self.syncList.removeItem(self.trajList.currentRow())
    

    def clicked_addButton(self):
        paths = self.dialog.getOpenFileNames(caption='Add Trajectory')
        self.addTrajectories(paths)
        

        

    def clicked_exportButton(self):
        if self.syncList.length() > 0:
            trajItem = self.syncList.itemList[0]
            prev = trajItem.getTrajectory()
            prev = prev[trajItem.start:trajItem.end+1]
            cummulative = prev
            
            for i in xrange(1, self.syncList.length()):
                lastJoint   = prev[len(prev)-1]
                currItem    = self.syncList.itemList[i]
                current     = currItem.getTrajectory()
                current     = current[currItem.start:currItem.end+1]
                firstJoint  = current[0]
                joined = joinJoints(lastJoint, firstJoint)
                cummulative = np.concatenate([cummulative, joined, current])
                prev        = current            
            path = str(self.dialog.getSaveFileName(caption='Export Trajectory'))
            dpath   = path[ : path.rfind('/')+1]
            prefix = path[path.rfind('/')+1 : ]
            
            np.save(dpath + prefix + '_larm.npy' , cummulative['l_arm'])                                                                      
            np.save(dpath + prefix + '_rarm.npy' , cummulative['r_arm'])                                                                      
            np.save(dpath + prefix + '_lgrip.npy', cummulative['l_gripper'])                                                                     
            np.save(dpath + prefix + '_rgrip.npy', cummulative['r_gripper'])
            self.addTrajectories([dpath+prefix+'_larm.npy'])
    
    
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
            copyItem = trajectoryItem(self.syncList, item.prefix, item.path, copy=True)
            copyItem.start = item.start
            copyItem.end = item.end
            copyItem.length = item.length
            


    def updateJoints(self, tickPos):
        selection = self.trajList.currentRow()
        if selection >=0:
            trajItem        = self.syncList.itemList[selection]
            trajectory      = trajItem.getTrajectory()
            self.pipeOR.send(['SetJoints', repr(trajectory[tickPos])])


    def moved_startSlider(self, pos):
        selection = self.trajList.currentRow()
        if selection >= 0:
            item  = self.syncList.itemList[selection]
            endPos = item.length - self.endSlider.value()
            if pos >= endPos:
                pos = endPos
                self.startSlider.setValue(pos)
            item.start = pos
            self.playSlider.setMinimum(item.start)
            

    def moved_endSlider(self, pos):
        selection = self.trajList.currentRow()
        if selection >= 0:
            item  = self.syncList.itemList[selection]
            startPos = self.startSlider.value()
            p  = item.length-pos
            if p < startPos:
                pos = item.length-startPos
                self.endSlider.setValue(pos)
            item.end = item.length - pos
            self.playSlider.setMaximum(item.end)
            

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
    def __init__(self, start_pos, end_pos, emitter, trajApp, delay=0.01):
        Thread.__init__(self)
        self.end_pos   = end_pos
        self.start_pos = start_pos
        self.emitter   = emitter
        self.trajApp   = trajApp
        self.delay  = delay
        self.daemon = True
        self.stop   = False
        JointsPusher.allPushers.add(self)

    def run(self):
        for p in JointsPusher.allPushers:
            p.stop = True
        self.stop = False

        while self.start_pos <= self.end_pos:
            self.emitter.emit(QtCore.SIGNAL("sliderMoved(int)"), self.start_pos)
            self.trajApp.playSlider.setValue(self.start_pos)
            self.start_pos += 1
            time.sleep(self.delay)
            if self.stop: break

        self.trajApp.setVisibilityModifiers(False)
        self.trajApp.playSelectedButton.setText("Play Selected")
        JointsPusher.allPushers.remove(self)


if __name__ == "__main__":
    try:
        ProcessStarter()
    except:
        sys.exit(0)

