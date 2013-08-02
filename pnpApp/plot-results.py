from __future__ import division
import cPickle
import numpy as np
import matplotlib.pyplot as plot

import Image
from StringIO import StringIO


fformat = "/home/ankush/sandbox444/pnpApp/run2_results/run-%d2-results.cpickle"
merge_fname = "/home/ankush/sandbox444/pnpApp/run2_results/merged-results.cpickle"
results = cPickle.load(open(merge_fname, 'r'))
EPS = np.spacing(1)
"""
# get cube success-ratios
cube_rates = np.zeros(10)
for runnum in range(10):
    runinfo = cPickle.load(open(fformat % (runnum+1), 'r'))
    
    n_pass, n_fail = 0.0,0.0
    for item in runinfo.iteritems():
        if item[1]['result']:
            n_pass += 1
        else:
            n_fail += 1

    cube_rates[runnum] = n_pass/(n_pass + n_fail)

err1 = np.sqrt(cube_rates*(1-cube_rates)/64)
plot.errorbar(np.arange(10)+1, cube_rates, yerr=err1, ecolor='red')
plot.scatter(np.arange(10)+1, cube_rates)
plot.xlabel('scaling steps', fontsize=18)
plot.ylabel('success rate', fontsize=18)
plot.axis((0,11,0,1))
plot.savefig('scaling.png')
plot.show()


# get hamming success-ratios:
n_runs = 640.0
hamm_rates = {}
hamm_tot = {}
pert_scale = np.array([-0.0025, 0.005, 0.005, 5, 5, 5])

for item in results.iteritems():
    hamm_dist = np.sum(np.round(item[1]['perturb']/pert_scale, 3))

    if item[1]['result']:  ## if the run was a success:
        if not hamm_rates.has_key(hamm_dist):
            hamm_rates[hamm_dist] = 1
        else:
            hamm_rates[hamm_dist] += 1

    if not hamm_tot.has_key(hamm_dist):
        hamm_tot[hamm_dist] = 1
    else:
        hamm_tot[hamm_dist] += 1


hx = np.arange(max(hamm_rates.keys())+1)
hy = np.zeros(hx.shape)
htot = np.zeros(hx.shape)
for it in hamm_rates.iteritems():
    hy[int(it[0])]  = it[1] / (hamm_tot[it[0]] + np.spacing(1))
    htot[int(it[0])] = hamm_tot[it[0]]


hx   = hx[np.nonzero(hy)]
htot = htot[np.nonzero(hy)]
hy   = hy[np.nonzero(hy)]


sds = np.sqrt(hy*(1-hy)/htot)
plot.clf()
_, caps,dd = plot.errorbar(hx, hy, yerr=sds, ecolor='red', lw=1.5 )

for lcol in dd:
    lcol.set_linewidths(0.8)   

plot.scatter(hx,hy)
plot.axis((-0.5,35, 0,1.03))
plot.xlabel('hamming length', fontsize=18)
plot.ylabel('success rate', fontsize=18)
plot.savefig('hamming.png')
"""

####################
# plot traj vc. warp : '+' : success, '-' : failure
####################

def getrun_costs(seginfo, dopose = True):
    warp_costs = []
    traj_costs = []
    for seg in seginfo:
        warp_costs.append(sum(seg['warp_costs']))

        if dopose:
            pose_costs = [ctuple[1] for ctuple in seg['larm_costs'][1][0] if 'pose' in ctuple[0]]
            traj_costs.append(max(pose_costs))
        else:
            traj_costs.append(np.max([ seg['larm_costs'][0][0], seg['larm_costs'][0][0]]) )

    return (max(warp_costs), max(traj_costs))

succ_w = []
succ_t = []
fail_w = []
fail_t = []

## separate the successes and failures
succ_runs = []
fail_runs = []
for item in results.iteritems():
    if item[1]['result']:
        succ_runs.append(item[0])
    else:
        fail_runs.append(item[0])

## open the costs file for each run and get the relevant data:
rfformat = "/media/data/run2-merged/run%d-costs.txt"
for runnum in succ_runs:
    info = cPickle.load(open(rfformat% runnum))
    wcost, tcost = getrun_costs(info['segments_info'])
    succ_w.append(wcost)
    succ_t.append(tcost)

for runnum in fail_runs:
    info = cPickle.load(open(rfformat% runnum))
    wcost, tcost = getrun_costs(info['segments_info'])
    fail_w.append(wcost)
    fail_t.append(tcost)

plot.clf()
"""
p1 = plot.scatter(succ_t, succ_w, color='black', marker='D',s=10)
p2 = plot.scatter(fail_t, fail_w, color='black', marker='o', facecolors='none',s=10)
plot.legend([p1, p2], ["success", "failure"])
plot.axis((5,12, 0, 0.00002))
plot.ticklabel_format(style='sci', axis='x', scilimits=(0,0))
plot.ticklabel_format(style='sci', axis='y', scilimits=(0,0))
plot.axhline(y=7e-7)
plot.xlabel('max pose error', fontsize=18)
plot.ylabel('warping cost', fontsize=18)
plot.savefig('tw-pose.pdf')
plot.show()        
"""

nbins = 100
succ_w = np.array(succ_w)
fail_w = np.array(fail_w)
wmin, wmax = min(np.min(succ_w), np.min(fail_w)), max(np.max(succ_w), np.max(fail_w))
wrange = (wmin, wmax)

sc, sbins = np.histogram(succ_w, nbins, range=wrange)
fc, fbins = np.histogram(fail_w, nbins, range=wrange)
assert np.allclose(sbins, fbins)
tc = sc+fc
sc = sc/(tc + EPS)

## trim the tail of zeros:
nz = np.max(np.nonzero(sc))
lim = min(nz+1, len(sc)-1)
sc = sc[0:lim+1]
sbins= sbins[0:lim+2]
wwidth = sbins[1]-sbins[0]


plot.bar(left=sbins[0:-1], height=sc, width=wwidth)
plot.xlabel('warping cost', fontsize=18)

import matplotlib.ticker as tckr
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


#major_formatter = plot.FormatStrFormatter('%0.2e')
#x_formatter = tckr.ScalarFormatter(useOffset=True)
#plot.gca().xaxis.set_major_formatter(major_formatter)
#plot.gca().xaxis.set_minor_formatter(x_formatter)
#plot.ticklabel_format(style='sci', axis='x', scilimits=(0,0))
plot.gca().xaxis.set_major_formatter(FixedOrderFormatter(-6))
plot.xticks(sbins[0:sbins.shape[0]-1:4])


#print ["%.2e"%nb for nb in sbins]
plot.ylabel('P(success | warping cost)', fontsize=18)
plot.show()
