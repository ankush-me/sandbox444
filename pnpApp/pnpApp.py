#! /usr/bin/env python

from PyQt4 import QtCore, QtGui
import pnp_ui 
import numpy as np
from os import path as osp
import argparse
import yaml, sys
from rapprentice.colorize import colorize
import cPickle

class pnpApp(QtGui.QMainWindow, pnp_ui.Ui_MainWindow):

    @QtCore.pyqtSignature("")
    def on_pbClose_clicked(self):
        self.closeAll()


    def __init__(self, run_names, run_range, run_dir, out_dir):
        super(pnpApp, self).__init__(None)
        self.setupUi(self)
                
        assert len(run_names)==run_range.shape[0]
        assert run_range.shape[1]==2

        self.run_range = run_range
        self.run_names = run_names
        self.run_dir   = run_dir
        self.out_dir   = out_dir
        self.rangenum  = 0
        self.runnum    = run_range[0,0]
        self.done = False
        
        self.results = {} 
        self.add_slots()
        
        # init display
        self.load_image()
        self.runLabel.setText('%d/%d'%(int(self.runnum), int(self.run_range[self.rangenum,1])))
        
    
    def add_slots(self):
        QtCore.QObject.connect(self.failButton, QtCore.SIGNAL("clicked()"), self.clicked_failButton)
        QtCore.QObject.connect(self.pnpButton,  QtCore.SIGNAL("clicked()"), self.clicked_pnpButton)
        QtCore.QObject.connect(self.passButton, QtCore.SIGNAL("clicked()"), self.clicked_passButton)


    def update_runnums(self, res):
        self.results[int(self.runnum)] = res

        if self.runnum == self.run_range[self.rangenum, 1]:
            self.save_results()
            self.results = {}
            self.rangenum += 1
            if self.rangenum >= self.run_range.shape[0]:
                self.done = True
            else:
                self.runnum = self.run_range[self.rangenum, 0]
        else:
            self.runnum += 1
            
        if not self.done: # update the picture
            self.runLabel.setText('%d/%d'%(int(self.runnum), int(self.run_range[self.rangenum,1])))
            self.load_image()
           
    def load_image(self):
        img_fname = osp.join(self.run_dir, 'run%d-image.jpg'%self.runnum)
        pixmap = QtGui.QPixmap(img_fname)
        if pixmap.isNull():
            print "File not found : %s"%img_fname
        self.imgLabel.setPixmap(pixmap) 
    

    def save_results(self):       
        summary_dict = {}

        for run_id in self.results.keys():
            cost_fname = osp.join(self.run_dir, 'run%d-costs.txt' % run_id)
            run_info   = cPickle.load(open(cost_fname, 'r'))
            run_dict = {}
            run_dict['perturb'] = run_info['perturbations']
            run_dict['result']  = self.results[run_id]
            summary_dict[run_id] = run_dict

        res_fname = osp.join(self.out_dir, 'run-%s-results.cpickle'%self.run_names[self.rangenum])
        print colorize('Saving results to : '+ res_fname, 'blue', bold=True)
        cPickle.dump(summary_dict, open(res_fname, 'w'))


    def clicked_failButton(self):
        if not self.done:
            self.update_runnums(0)
        
    def clicked_pnpButton(self):
        if not self.done:
            self.update_runnums(0.5)
        
    def clicked_passButton(self):
        if not self.done:
            self.update_runnums(1)
        

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="P/NP App")
    parser.add_argument('-c', '--config', help="Config file name.", required=True)  
    parser.add_argument('-d', '--rundir', help="Directory with the run images.", required=True)
    parser.add_argument('-o', '--outdir', help="Directory to save the annotated results", required=True)
    vals = parser.parse_args()

    run_ranges = np.loadtxt(vals.config)
    run_names  = [str(row)[0:2] for row in run_ranges[:,0]]
    app  = QtGui.QApplication(sys.argv)
    form = pnpApp(run_names, run_ranges, vals.rundir, vals.outdir)
    form.show()
    sys.exit(app.exec_())
