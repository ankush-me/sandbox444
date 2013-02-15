from PyQt4 import QtGui
import numpy as np

class ListInteractor:
    def __init__(self, qtListObj):
        self.listWidget = qtListObj
        self.trajData   = {}
        self.itemList   = []

    def addItem(self, trajItem):
        """
        Add an item to the internal list as well as display list.
        """
        itemName = trajItem.path + trajItem.prefix
        itemDisplayName = trajItem.prefix
        
        length = -1
        
        if itemName not in self.trajData:
            
            larm_traj  = np.load(itemName + '_larm.npy')
            rarm_traj  = np.load(itemName + '_rarm.npy')
            lgrip_traj = np.load(itemName + '_lgrip.npy')
            rgrip_traj = np.load(itemName + '_rgrip.npy')
            
            length = len(larm_traj)
            full_traj = np.zeros(length, dtype=[('r_arm', '7float64'), 
                                                ('l_arm', '7float64'),
                                                ('r_gripper', 'float64'), 
                                                ('l_gripper', 'float64')])
            full_traj['r_arm']     = rarm_traj
            full_traj['l_arm']     = larm_traj
            full_traj['r_gripper'] = rgrip_traj
            full_traj['l_gripper'] = lgrip_traj

            self.trajData[itemName] = (1, full_traj)

        else:
            (count, trajArray)      = self.trajData[itemName]
            self.trajData[itemName] = (count+1, trajArray)
            length = len(trajArray)
            itemDisplayName += '_copy'

        item = QtGui.QListWidgetItem(itemDisplayName)
        self.listWidget.addItem(item)

        (trajItem.length, trajItem.start, trajItem.end, trajItem.qtItem) = (length, 0, length, item)
        self.itemList.append(trajItem)


    def removeItem(self, idx):
        """
        Remove the item at row IDX.
        """
        if 0 <= idx and idx < self.listWidget.count():
            itemStr               = self.itemList[idx].getStr()
            (count, trajArray) = self.trajData[itemStr]

            if count == 1:   # remove numpy array
                del self.trajData[itemStr]
            else:
                self.trajData[itemStr] = (count-1, trajArray) 

            self.itemList.pop(idx)           
            return self.listWidget.takeItem(idx)


    def last(self):
        if self.itemList:
            (count, trajArray) = self.trajData[self.itemList[-1].getStr()]
            return  trajArray
        else:
            return None

    def length(self):
        return len(self.itemList)
