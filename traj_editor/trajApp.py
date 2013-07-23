from PyQt4          import QtCore, QtGui
import sys, os, re, logging, signal, time
from numpy import random
from multiprocessing import Process,Pipe
from threading import Thread
from traj_editor_new import Ui_MainWindow
import numpy as np
from ListInteractor import ListInteractor
from EasyPR2        import EasyPR2
from ProcessStarter import *
from joints_utils import *

from sceneparser import *
import cPickle    

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
        self.startVal.setText('start: ')
        self.endVal.setText('end: ')


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
        QtCore.QObject.connect(self.startSlider,    QtCore.SIGNAL("valueChanged(int)"), self.moved_startSlider)
        QtCore.QObject.connect(self.endSlider,      QtCore.SIGNAL("valueChanged(int)"), self.moved_endSlider)
        QtCore.QObject.connect(self.playSlider,     QtCore.SIGNAL("valueChanged(int)"), self.moved_playSlider)
        QtCore.QObject.connect(self.trajList,       QtCore.SIGNAL("currentRowChanged(int)"), self.changed_trajList)


    def changed_trajList(self, row):
        if row != -1:
            segitem    = self.syncList.segmentList[row]
            self.startSlider.setMinimum(0)
            self.startSlider.setMaximum(segitem.len-1)
            self.startSlider.setValue(segitem.start)

            self.endSlider.setMinimum(0)
            self.endSlider.setMaximum(segitem.len-1)
            self.endSlider.setValue(segitem.len-1-segitem.end)

            self.playSlider.setMinimum(segitem.start)
            self.playSlider.setMaximum(segitem.end)
            self.playSlider.setValue(segitem.start)
            self.startVal.setText('start: %d'% segitem.start)
            self.endVal.setText('end: %d'% segitem.end)
            

    def setVisibilityModifiers(self, visible):
        self.addButton.setDisabled(visible)
        self.removeButton.setDisabled(visible)
        self.exportButton.setDisabled(visible)
        self.downButton.setDisabled(visible)
        self.upButton.setDisabled(visible)
        self.copyButton.setDisabled(visible)
        self.startSlider.setDisabled(visible)                     
        self.endSlider.setDisabled(visible)                    
        self.trajList.setDisabled(visible)


    def clicked_playSelectedButton(self):
        pass
#         caption = self.playSelectedButton.text()
#         if caption=="Play Selected":
#             self.playSelectedButton.setText("Pause")
#             selection = self.trajList.currentRow()
#             if selection >=0 and selection < self.syncList.length():
#                 self.setVisibilityModifiers(True)
#                 trajItem        = self.syncList.itemList[selection]
#                 jTask = JointsPusher(self.playSlider.value(), trajItem.end, self.playSlider, self)
#                 jTask.start()
#         else:
#             self.playSelectedButton.setText("Pause")
#             jTask = JointsPusher(0, -1, self.playSlider, self)
#             jTask.start()
        

    def clicked_removeButton(self):
        if self.syncList.length() > 0:
            self.syncList.removeItem(self.trajList.currentRow())
            if self.syncList.length() > 0:
                selection = self.trajList.currentRow()
                if selection >=0:
                    trajItem        = self.syncList.itemList[selection]
                    self.startVal.setText('start: %d'%trajItem.start)
                    self.endVal.setText('end: %d'%trajItem.end)
            else:
                self.startVal.setText('start: ')
                self.endVal.setText('end: ')
                                 

    def clicked_addButton(self):
        fname = self.dialog.getOpenFileName(caption='Add Trajectory')
        self.add_segments(fname, False)


    def add_segments(self, fname, copy=False):
        """
        Open a scene file and add all segments in that into the list obj.
        """
        segments = parsescene(fname)
        for seg in segments:
            if copy:
                seg['name'] = seg['name'] + '-copy'
            self.syncList.addItem(seg)


    def clicked_exportButton(self):
        pass
#         if self.syncList.length() > 0:
#             trajItem = self.syncList.itemList[0]
#             prev = trajItem.getTrajectory()
#             prev = prev[trajItem.start:trajItem.end+1]
#             cummulative = prev
#             
#             for i in xrange(1, self.syncList.length()):
#                 lastJoint   = prev[len(prev)-1]
#                 currItem    = self.syncList.itemList[i]
#                 current     = currItem.getTrajectory()
#                 current     = current[currItem.start:currItem.end+1]
#                 firstJoint  = current[0]
#                 joined = joinJoints(lastJoint, firstJoint)
#                 cummulative = np.concatenate([cummulative, joined, current])
#                 prev        = current            
#             path = str(self.dialog.getSaveFileName(caption='Export Trajectory'))
#             dpath   = path[ : path.rfind('/')+1]
#             prefix = path[path.rfind('/')+1 : ]
#             
#             np.save(dpath + prefix + '_larm.npy' , cummulative['l_arm'])                                                                      
#             np.save(dpath + prefix + '_rarm.npy' , cummulative['r_arm'])                                                                      
#             np.save(dpath + prefix + '_lgrip.npy', cummulative['l_gripper'])                                                                     
#             np.save(dpath + prefix + '_rgrip.npy', cummulative['r_gripper'])
#             self.addTrajectories([dpath+prefix+'_larm.npy'])
    
    
    def clicked_downButton(self):
        selection = self.trajList.currentRow()
        if selection >=0 and selection < self.syncList.length()-1:
            item     = self.trajList.takeItem(selection)
            self.trajList.insertItem(selection+1, item)
            self.syncList.segmentList[selection], self.syncList.segmentList[selection+1] = self.syncList.segmentList[selection+1], self.syncList.segmentList[selection]
            self.trajList.setCurrentRow(selection+1)  


    def clicked_upButton(self):
        selection = self.trajList.currentRow()
        if selection > 0 and selection < self.syncList.length():
            item     = self.trajList.takeItem(selection)
            self.trajList.insertItem(selection-1, item)
            self.syncList.segmentList[selection], self.syncList.segmentList[selection-1] = self.syncList.segmentList[selection-1], self.syncList.segmentList[selection]  
            self.trajList.setCurrentRow(selection-1)  
           

    def clicked_copyButton(self):
        pass
#         selection = self.trajList.currentRow()
#         if selection >= 0 and selection < self.syncList.length():
#             segitem  = self.syncList.segmentList[selection]
#             copyItem = trajectoryItem(self.syncList, item.prefix, item.path, copy=True)
#             copyItem.start = item.start
#             copyItem.end = item.end
#             copyItem.length = item.length
            

    def moved_startSlider(self, pos):
        selection = self.trajList.currentRow()
        if selection >= 0:
            segitem  = self.syncList.segmentList[selection]
            endPos = segitem.len - self.endSlider.value()
            if pos >= endPos:
                pos = endPos
                self.startSlider.setValue(pos)
            segitem.start = pos
            self.playSlider.setMinimum(segitem.start)
            self.startVal.setText("start: %d"%segitem.start)
            

    def moved_endSlider(self, pos):
        selection = self.trajList.currentRow()
        if selection >= 0:
            segitem  = self.syncList.segmentList[selection]
            startPos = self.startSlider.value()
            p  = segitem.len-pos
            if p < startPos:
                pos = segitem.len-startPos
                self.endSlider.setValue(pos)
            segitem.end = segitem.len - pos
            self.playSlider.setMaximum(segitem.end)
            self.endVal.setText("end: %d"%segitem.end)
            

    def moved_playSlider(self, pos):
        self.updateJoints(pos)


    def updateJoints(self, tickPos):
        selection = self.trajList.currentRow()
        if selection >=0:
            segitem    = self.syncList.segmentList[selection]
            joints = segitem.seg['joints'][tickPos]
            self.pipeOR.send(['SetJoints', cPickle.dumps(joints)])


# class JointsPusher(Thread):
#     allPushers = set()
#     def __init__(self, start_pos, end_pos, emitter, trajApp, delay=0.01):
#         Thread.__init__(self)
#         self.end_pos   = end_pos
#         self.start_pos = start_pos
#         self.emitter   = emitter
#         self.trajApp   = trajApp
#         self.delay  = delay
#         self.daemon = True
#         self.stop   = False
#         JointsPusher.allPushers.add(self)
# 
#     def run(self):
#         for p in JointsPusher.allPushers:
#             p.stop = True
#         self.stop = False
# 
#         while self.start_pos <= self.end_pos:
#             self.emitter.emit(QtCore.SIGNAL("sliderMoved(int)"), self.start_pos)
#             self.trajApp.playSlider.setValue(self.start_pos)
#             self.start_pos += 1
#             time.sleep(self.delay)
#             if self.stop: break
# 
#         self.trajApp.setVisibilityModifiers(False)
#         self.trajApp.playSelectedButton.setText("Play Selected")
#         JointsPusher.allPushers.remove(self)


if __name__ == "__main__":
    try:
        ProcessStarter()
    except:
        sys.exit(0)

