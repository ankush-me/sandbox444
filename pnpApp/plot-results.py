from __future__ import division
import cPickle
import numpy as np
import matplotlib.pyplot as plot
import openravepy as rave
from rapprentice.colorize import colorize
from rapprentice import transformations
import Image
from StringIO import StringIO


fformat     = "/home/ankush/sandbox444/pnpApp/run3_results/run-%d2-results.cpickle"
merge_fname = "/home/ankush/sandbox444/pnpApp/run3_results/merged-results.cpickle"
rfformat    = "/media/data/suturing_runs3/run%d-costs.txt"
costs_fname = "/home/ankush/sandbox444/pnpApp/run3_results/costs.cpickle"

results = cPickle.load(open(merge_fname, 'r'))
EPS = np.spacing(1)

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
plot.errorbar(np.arange(10)+1, cube_rates, yerr=err1, ecolor='red', lw=2)
plot.scatter(np.arange(10)+1, cube_rates)
plot.xlabel('scaling steps', fontsize=20)
plot.ylabel('success rate', fontsize=20)
plot.axis((0,11,0,1))
plot.savefig('scaling.pdf')
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
_, caps,dd = plot.errorbar(hx, hy, yerr=sds, ecolor='red', lw=2 )

for lcol in dd:
    lcol.set_linewidths(1.)   

plot.scatter(hx,hy)
plot.axis((-0.5,35, 0,1.03))
plot.xlabel('Hamming distance', fontsize=20)
plot.ylabel('success rate', fontsize=20)
plot.savefig('hamming.pdf')
plot.show()

####################
# plot traj vc. warp : '+' : success, '-' : failure
####################

def getrun_costs(seginfo, dopose = False):
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


def get_angles(hmats):
    """
    returns the angles of a list of 4x4 transformation matrices
    """
    assert hmats.ndim ==3 and hmats.shape[1:]==(4,4)
    angles = np.empty(hmats.shape[0])
    for i in xrange(hmats.shape[0]):
        angles[i] = np.linalg.norm(rave.axisAngleFromRotationMatrix(hmats[i,0:3,0:3]))
    return angles


def get_position(hmats):
    """
    returns the norm of the translations of a list of 4x4 transformation matrices
    """
    assert hmats.ndim ==3 and hmats.shape[1:]==(4,4)
    pos = np.empty(hmats.shape[0])
    for i in xrange(hmats.shape[0]):
        pos[i] = np.linalg.norm(hmats[i,0:3,3])
    return pos


def get_costs(seginfo):
    """
    Returns a 3-tuple (a numpy array):
    (max warp-cost, max position error, max orientation error) across all segments.
    Orientation error is calculated as the angle of the axis-angle representation of the error transform : F^-1*F
    """
    mwarp  = -1
    mpos   = -1
    mangle = -1

    for seg in seginfo:
        ## update warping costs:
        mwarp = max(mwarp, sum(seg['warp_costs']))
        
        ## update pose errors:
        for arm in 'lr':
            dat = seg['%sarm_costs'%arm][0]
            assert dat.ndim==4 and dat.shape[2:]==(4,4)
            dat = dat.reshape(dat.shape[0]*dat.shape[1], 4,4)
            mpos   = max(mpos, np.max(get_position(dat)))
            mangle = max(mangle, np.max(get_angles(dat)))

    return np.array([mwarp, mpos, mangle])


"""
## separate the successes and failures
succ_runs = []
fail_runs = []
for item in results.iteritems():
    if item[1]['result']:
        succ_runs.append(item[0])
    else:
        fail_runs.append(item[0])
print colorize("succ/ fail : %d/%d" %(len(succ_runs), len(fail_runs)), "red", True)

## open the costs file for each run and get the relevant data:
succ_w, succ_p, succ_a = [], [], []
fail_w, fail_p, fail_a = [], [], []

for runnum in succ_runs:
    info = cPickle.load(open(rfformat% runnum))
    wcost, pcost, acost = get_costs(info['segments_info'])
    succ_w.append(wcost)
    succ_p.append(pcost)
    succ_a.append(acost)

for runnum in fail_runs:
    info = cPickle.load(open(rfformat% runnum))
    wcost, pcost, acost = get_costs(info['segments_info'])
    fail_w.append(wcost)
    fail_p.append(pcost)
    fail_a.append(acost)

runcosts = {}
runcosts['succ_w'] = np.array(succ_w)
runcosts['succ_p'] = np.array(succ_p)
runcosts['succ_a'] = np.array(succ_a)
runcosts['fail_w'] = np.array(fail_w)
runcosts['fail_p'] = np.array(fail_p)
runcosts['fail_a'] = np.array(fail_a)
cPickle.dump(runcosts, open(costs_fname, 'wb'))
print colorize('Saved costs to : %s'%costs_fname, 'green', True)
"""
