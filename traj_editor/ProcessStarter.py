from PyQt4     import   QtCore, QtGui
from trajApp   import   trajApp
from multiprocessing import Process,Pipe
from ORServer  import   *
import sys


class ProcessStarter(object):
    
    def __init__(self):
        '''
        Setup the shared memory data structure model and initialize the control parts.
        '''
        self.running = True
        self.orprocess  = None
        self.guiprocess = None
        (self.pipeGUI, self.pipeOR) = Pipe()

        self.StartProcesses()
        

    def StartProcesses(self):
        self.guiprocess = Process(target=self._StartGUI)
        self.guiprocess.start()        
        self.pipeGUI.send(["StartViewer", None])

        self.orprocess = Process(target=ORServer,args=(self.pipeOR,))
        self.orprocess.start()



        return True

    def _StartGUI(self):
        app = QtGui.QApplication(sys.argv)
        form = trajApp(self.pipeGUI)
        form.show()
        app.exec_()
