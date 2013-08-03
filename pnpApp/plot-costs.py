from __future__ import division
import cPickle
import numpy as np
from rapprentice.colorize import colorize
import os.path as osp

import matplotlib.pyplot as plot
import matplotlib.ticker as tckr


costs_fname = "/home/ankush/sandbox444/pnpApp/run3_results/costs.cpickle"
save_dir    = "/home/ankush/sandbox444/pnpApp/plots/"


class FixedOrderFormatter(tckr.ScalarFormatter):
    """Formats axis ticks using scientific notation with a constant order of 
    magnitude"""
    def __init__(self, order_of_mag=0, useOffset=True, useMathText=False):
        self._order_of_mag = order_of_mag
        tckr.ScalarFormatter.__init__(self, useOffset=useOffset, 
                                 useMathText=useMathText)
    def _set_orderOfMagnitude(self, range):
        """Over-riding this to avoid having orderOfMagnitude reset elsewhere"""
        self.orderOfMagnitude = self._order_of_mag

    def _set_format(self):
        self.format = '%0.2f'


def plot_scatter(wpass, wfail, ppass, pfail, position=True, x_max=0.025):
    """
    Does a scatter plot of the warping costs (y-axis) and position/angle error (x-axis)
    """
    plot.clf()
    p1 = plot.scatter(ppass, wpass, color='black', marker='D',s=20)
    p2 = plot.scatter(pfail, wfail, color='black', marker='o',s=20, facecolors='none')
    plot.legend([p1, p2], ["success", "failure"])

    pname = "position" if position else "orientation"

    if x_max > 0:
        plot.axis((0 if position else 0.3, x_max, 0, 1e-5))

    plot.ticklabel_format(style='sci', axis='x', scilimits=(0,0))
    plot.ticklabel_format(style='sci', axis='y', scilimits=(0,0))
    plot.axhline(y=8.5e-7)
    plot.xlabel('max %s error'%pname, fontsize=18)
    plot.ylabel('warping cost', fontsize=18)

    plot_fname = osp.join(save_dir, 'warp-%s.pdf'%pname)
    plot.savefig(plot_fname)
    print colorize("saved plot: %s"%plot_fname, "green", True)


def plot_prob(pass_dat, fail_data, cost_name='', nbins=50):
    """
    pass_dat, fail_dat : np.arrays of costs of successes and failures respectively
    bins the data into n-bins and plots a probability distribution:
    """
    cmin, cwmax = min(np.min(pass_dat), np.min(fail_dat)), max(np.max(pass_dat), np.max(fail_dat))
    crange = (cmin, cmax)
    sc, sbins = np.histogram(pass_dat, nbins, range=crange)
    fc, fbins = np.histogram(fail_dat, nbins, range=crange)

    assert np.allclose(sbins, fbins)
    tc = sc+fc
    sc = sc/(tc + EPS)

    ## trim the tail of zeros:
    nz = np.max(np.nonzero(sc))
    lim = min(nz+1, len(sc)-1)
    sc = sc[0:lim+1]
    sbins= sbins[0:lim+2]
    wwidth = sbins[1]-sbins[0]
    
    plot.clf()
    plot.bar(left=sbins[0:-1], height=sc, width=wwidth)
    plot.xlabel('%s cost'%cost_name, fontsize=18)

    plot.gca().xaxis.set_major_formatter(FixedOrderFormatter(-6))
    plot.xticks(sbins[0:sbins.shape[0]-1:4])
    plot.ylabel('P(success | %s cost)'%cost_name, fontsize=18)
    plot.savefig('prob-%s.pdf')


costs = cPickle.load(open(costs_fname, 'rb'))
plot_scatter(costs['succ_w'], costs['fail_w'], costs['succ_p'], costs['fail_p'], True)
plot.show()

plot_scatter(costs['succ_w'], costs['fail_w'], costs['succ_a'], costs['fail_a'], False, 2.5)
plot.show()

## plot probability distributions:
#  1. p(succ | warp-cost)
#  2. p(succ | pos-err)
#  3. p(succ | orient-err)


#major_formatter = plot.FormatStrFormatter('%0.2e')
#x_formatter = tckr.ScalarFormatter(useOffset=True)
#plot.gca().xaxis.set_major_formatter(major_formatter)
#plot.gca().xaxis.set_minor_formatter(x_formatter)
#plot.ticklabel_format(style='sci', axis='x', scilimits=(0,0))


#print ["%.2e"%nb for nb in sbins]
