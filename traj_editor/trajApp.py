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
import copy

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
        QtCore.QObject.connect(self.startSlider,    QtCore.SIGNAL("sliderMoved(int)"), self.moved_startSlider)
        QtCore.QObject.connect(self.endSlider,      QtCore.SIGNAL("sliderMoved(int)"), self.moved_endSlider)
        QtCore.QObject.connect(self.playSlider,     QtCore.SIGNAL("valueChanged(int)"), self.moved_playSlider)
        QtCore.QObject.connect(self.trajList,       QtCore.SIGNAL("currentRowChanged(int)"), self.changed_trajList)


    def changed_trajList(self, row):
        ## update the sliders and various labels
        if 0 <= row < self.syncList.length():
            
            print "changed list row : ", row
            
            segitem    = self.syncList.segmentList[row]
            self.startSlider.setMinimum(0)
            self.startSlider.setMaximum(segitem.slen-1)           
            self.endSlider.setMinimum(0)
            self.endSlider.setMaximum(segitem.slen-1)
            self.playSlider.setMinimum(segitem.sstart)
            self.playSlider.setMaximum(segitem.send)
            
            self.startSlider.setValue(segitem.sstart)
            self.endSlider.setValue(segitem.slen-1-segitem.send)
            self.playSlider.setValue(segitem.sstart)

            self.startVal.setText('start: %d'% segitem.sstart)
            self.endVal.setText('end: %d'% segitem.send)
            

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
        caption = self.playSelectedButton.text()
        if caption=="Play Selected":
            self.playSelectedButton.setText("Pause")
            selection = self.trajList.currentRow()
            if selection >=0 and selection < self.syncList.length():
                self.setVisibilityModifiers(True)
                segitem = self.syncList.segmentList[selection]
                jTask = JointsPusher(self.playSlider.value(), segitem.send, self.playSlider, self)
                jTask.start()
        else:
            self.playSelectedButton.setText("Pause")
            jTask = JointsPusher(0, -1, self.playSlider, self)
            jTask.start()


    def clicked_removeButton(self):
        if self.syncList.length() > 0:
            self.syncList.removeItem(self.trajList.currentRow())
            if self.syncList.length() > 0:
                selection = self.trajList.currentRow()
                if selection >=0:
                    trajItem        = self.syncList.segmentList[selection]
                    self.startVal.setText('start: %d'%trajItem.sstart)
                    self.endVal.setText('end: %d'%trajItem.send)
            else:
                self.startVal.setText('start: ')
                self.endVal.setText('end: ')
                                 

    def clicked_addButton(self):
        fname = self.dialog.getOpenFileName(caption='Add Trajectory')
        self.add_segments(fname)


    def add_segments(self, fname):
        """
        Open a scene file and add all segments in that into the list obj.
        """
        segments = parsescene(fname)
        for seg in segments:
            self.syncList.addItem(seg)


    def write_point_data(self, ptypes, psecs, points, tfile):
        n = 0
        for i,ptype in enumerate(ptypes):
            tfile.write('\t%s\n'%ptype)
            ptnum = 0
            while ptnum < psecs[i]:
                x,y,z = points[n, :]
                tfile.write('\t\t%f\t%f\t%f\n'%(x,y,z))
                n += 1
                ptnum += 1
        tfile.write('end-points\n')


    def clicked_exportButton(self):
        starttime = 0
        
        segs_copy = copy.deepcopy(self.syncList.segmentList)
        for i in xrange(len(segs_copy)):
            segitem = segs_copy[i]

            print "i : ", i
            print "segment end : ", segitem.send
            print "len jtimes :", segitem.seg['jtimes'].shape

            seg_starttime = segitem.seg['jtimes'][segitem.sstart]
            seg_endtime   = segitem.seg['jtimes'][segitem.send]

            # filter the joints/ joint-times:        
            segitem.seg['joints'] = segitem.seg['joints'][segitem.sstart:segitem.send+1,:]
            segitem.seg['jtimes'] = segitem.seg['jtimes'][segitem.sstart:segitem.send+1] - seg_starttime + starttime          
            
            # filter the grips based on start and end points of the segment:
            g_start = np.searchsorted(segitem.seg['gtimes'], seg_starttime, 'left')
            g_end   = np.searchsorted(segitem.seg['gtimes'], seg_endtime, 'right')
            segitem.seg['gtimes'] = segitem.seg['gtimes'][g_start:g_end+1]  - seg_starttime + starttime
            segitem.seg['grips']  = segitem.seg['grips'][g_start:g_end+1]
            
            # filter the point clouds based on the start and end points of the segment
            p_start = np.searchsorted(segitem.seg['ptimes'], seg_starttime, 'left')
            p_end   = np.searchsorted(segitem.seg['ptimes'], seg_endtime, 'right')
            segitem.seg['ptimes'] = segitem.seg['ptimes'][p_start:p_end+1]  - seg_starttime + starttime
            segitem.seg['points']  = segitem.seg['points'][p_start:p_end+1,:,:]

            # undo the bulletsim-openrave offset.
            segitem.seg['points'] += np.array((0,0, 0.05))

            # update the start-time for the next segment:
            starttime = seg_starttime


        fname = self.dialog.getSaveFileName(caption='Save Trajectory')
        tfile = open(fname, 'w')

        grip_dict = {0: 'release r', 1:'grab r', 2:'release l', 3:'grab l'}

        for segitem in segs_copy:

            if (segitem.seg['jtimes'].shape[0] > 0):
                
                ## add the look (denoting start of a new segment), only for the original segments.
                #if 'copy' not in segitem.seg['name']:
                looktime = segitem.seg['jtimes'][0]
                tfile.write("%f : look\n"%looktime)

                alltimes = np.unique(np.concatenate([segitem.seg['jtimes'], segitem.seg['ptimes'], segitem.seg['gtimes']]))
                
                ji = pi = gi = 0
                for t in alltimes:
                    
                    # write points
                    if pi < segitem.seg['ptimes'].shape[0] and segitem.seg['ptimes'][pi]==t:
                        tfile.write('%f : points\n'%segitem.seg['ptimes'][pi])
                        self.write_point_data(segitem.seg['ptypes'], segitem.seg['point_secs'], segitem.seg['points'][pi,:,:], tfile)
                        pi += 1

                    # write grips:
                    if gi < segitem.seg['gtimes'].shape[0] and segitem.seg['gtimes'][gi]==t:
                        tfile.write('%f : %s\n'%(segitem.seg['gtimes'][gi], grip_dict[segitem.seg['grips'][gi]]))
                        gi += 1
                    
                    # write joints:
                    if ji < segitem.seg['jtimes'].shape[0] and segitem.seg['jtimes'][ji]==t:
                        joint_msg = '%f : joints :\t'%segitem.seg['jtimes'][ji]
                        for j in segitem.seg['joints'][ji,:]:
                            joint_msg += '%f  '%j
                        joint_msg += '\n'
                        tfile.write(joint_msg)
                        ji += 1
        tfile.close()
        print colorize('DONE EXPORTING THE FILE', 'red', True)
    
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
        selection = self.trajList.currentRow()
        if 0 <= selection < self.syncList.length():
            segitem  = self.syncList.segmentList[selection]
            copy_seg = copy.deepcopy(segitem.seg)
            copy_seg['name'] = segitem.seg['name'] + '-copy'
            self.syncList.addItem(copy_seg, segitem.sstart, segitem.send)


    def moved_startSlider(self, pos):
        selection = self.trajList.currentRow()
        if selection >= 0:
            segitem  = self.syncList.segmentList[selection]
            endPos = segitem.slen - self.endSlider.value()
            if pos >= endPos:
                pos = endPos
                self.startSlider.setValue(pos)
            self.syncList.segmentList[selection].sstart = pos
            self.playSlider.setMinimum(segitem.sstart)
            self.startVal.setText("start: %d"%segitem.sstart)
            

    def moved_endSlider(self, pos):
        selection = self.trajList.currentRow()
        print "end-slider >>>>>> : ", pos, " | selection : ", selection  
        
        if selection >= 0:
            segitem  = self.syncList.segmentList[selection]
            print '\t\t >> : len : ', segitem.slen
            startPos = self.startSlider.value()
            p  = segitem.slen-1-pos
            if p < startPos:
                pos = segitem.slen-1-startPos
                self.endSlider.setValue(pos)
            self.syncList.segmentList[selection].send = segitem.slen- 1 - pos
            self.playSlider.setMaximum(segitem.send)
            self.endVal.setText("end: %d"%segitem.send)
            

    def moved_playSlider(self, pos):
        self.updateJoints(pos)


    def updateJoints(self, tickPos):
        selection = self.trajList.currentRow()
        if selection >=0:
            segitem    = self.syncList.segmentList[selection]
            if 0<= tickPos <  segitem.seg['joints'].shape[0]:
                joints = segitem.seg['joints'][tickPos,:]
                self.pipeOR.send(['SetJoints', cPickle.dumps(joints)])
                
                points_idx = segitem.seg['j2ptimes'][tickPos]
                if  points_idx >=0:
                    self.pipeOR.send(['PlotPoints', cPickle.dumps(segitem.seg['points'][points_idx,:,:])])


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

