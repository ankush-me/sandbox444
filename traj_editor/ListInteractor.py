from PyQt4 import QtGui
import numpy as np


class segmentWrapper:
    def __init__(self, seg, length, start, end):
        self.seg = seg
        self.len = length
        self.start = start
        self.end = end


class ListInteractor:
    def __init__(self, qtListObj):
        self.listWidget = qtListObj
        self.segmentList   = []

    def addItem(self, segment):
        """
        Add an item to the internal list as well as display list.
        """        
        self.listWidget.addItem(QtGui.QListWidgetItem(segment['name']))
        
        jlen = len(segment['joints'])
        segitem = segmentWrapper(segment, jlen, 0, jlen-1)
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
