from __future__ import division
import cPickle
import numpy as np
from rapprentice.colorize import colorize
import os.path as osp

import matplotlib.pyplot as plot
import matplotlib.ticker as tckr

EPS = np.spacing(1)

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


def plot_prob(pass_dat, fail_dat, cost_name='', label_order=-6, is_cost=True, nbins=20):
    """
    pass_dat, fail_dat : np.arrays of costs of successes and failures respectively
    bins the data into n-bins and plots a probability distribution:
    """
    cmin, cmax = min(np.min(pass_dat), np.min(fail_dat)), max(np.max(pass_dat), np.max(fail_dat))
    crange = (cmin, cmax)
    sc, sbins = np.histogram(pass_dat, nbins, range=crange)
    fc, fbins = np.histogram(fail_dat, nbins, range=crange)

    assert np.allclose(sbins, fbins)
    tc = sc+fc
    sc = sc/(tc + EPS)

    ## trim the tail of zeros:
    nz, mz = np.max(np.nonzero(sc)), np.min(np.nonzero(sc))
    lim = min(nz+1, len(sc)-1)
    sc = sc[mz:lim+1]
    tc = tc[mz:lim+1]
    
    sd = np.sqrt(sc*(1-sc)/(tc+EPS))

    sbins= sbins[mz:lim+2]
    wwidth = sbins[1]-sbins[0]
     
    plot.clf()
    plot.bar(left=sbins[0:-1], height=sc, width=wwidth, yerr=sd, color=(0.8,0.8,0.8), ecolor=(0,0,0))

    xname = '%s%s %s' % ('' if is_cost else 'max ', cost_name, 'cost' if is_cost else 'error')
    plot.xlabel(xname, fontsize=18)

    plot.gca().xaxis.set_major_formatter(FixedOrderFormatter(label_order))
    ca1,ca2,ca3,ca4 =  plot.axis()
    ca1 = max(0,sbins[0]-wwidth)
    plot.axis((ca1,ca2,ca3,1.))
    plot.xticks(sbins[0:-1:4])
    plot.ylabel('P(success | %s)'%xname, fontsize=18)

    plot_fname = osp.join(save_dir, 'prob-%s.pdf'%cost_name)
    plot.savefig(plot_fname)
    print colorize("saved plot: %s"%plot_fname, "green", True)


costs = cPickle.load(open(costs_fname, 'rb'))


# plot the scatter plots : warping-vs-position/ orientation error:
plot_scatter(costs['succ_w'], costs['fail_w'], costs['succ_p'], costs['fail_p'], True)
plot_scatter(costs['succ_w'], costs['fail_w'], costs['succ_a'], costs['fail_a'], False, 2.5)


## plot probability distributions:
#  1. p(succ | warp-cost)
#  2. p(succ | pos-err)
#  3. p(succ | orient-err)
plot_prob(costs['succ_w'], costs['fail_w'], cost_name='warping', nbins=22)
plot_prob(costs['succ_p'], costs['fail_p'], label_order=-2, cost_name='position', is_cost=False, nbins=27)
plot_prob(costs['succ_a'], costs['fail_a'], label_order=-1, cost_name='orientation', is_cost=False, nbins=15)
