from PyQt4 import QtGui
import numpy as np


class segmentWrapper:
    def __init__(self, seg, length, start, end):
        self.seg = seg
        self.slen = length
        self.sstart = start
        self.send = end


class ListInteractor:
    def __init__(self, qtListObj):
        self.listWidget = qtListObj
        self.segmentList   = []

    def addItem(self, segment, s_start=None, s_end=None):
        """
        Add an item to the internal list as well as display list.
        """        
        self.listWidget.addItem(QtGui.QListWidgetItem(segment['name']))
        
        jlen = len(segment['joints'])
        
        if s_start == None:
            s_start = 0
            
        if s_end == None:
            s_end = jlen-1

        segitem = segmentWrapper(segment, jlen, s_start, s_end)
        self.segmentList.append(segitem)

    


    def removeItem(self, idx):
        """
        Remove the item at row IDX.
        """
        if 0 <= idx and idx < self.listWidget.count():
            self.segmentList.pop(idx)           
            return self.listWidget.takeItem(idx)

    def length(self):
        return len(self.segmentList)
