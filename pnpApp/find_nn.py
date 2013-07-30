# Script to find the nearest neighbours based on warping cost and tps-rpm-bij
from __future__ import division
import cPickle
import numpy as np
from rapprentice.colorize import *
from rapprentice import registration

nnN = 5

res_fname  = "/home/ankush/sandbox444/pnpApp/run2_results/merged-results.cpickle"
pert_scale = np.array([-0.0025, 0.005, 0.005, 5, 5, 5])

results   = cPickle.load(open(res_fname, 'rb'))

# separate the passed and failed runs:
pass_runs = []
fail_runs = []
for item in results.iteritems():
    if item[1]['result']:
        pass_runs.append(item[0])
    else:
        fail_runs.append(item[0])
print colorize("Found %d pass runs, %d failed runs" %(len(pass_runs), len(fail_runs)), "green", True)

fail_runs = np.sort(np.array(fail_runs, dtype='int'))
pass_runs = np.sort(np.array(pass_runs, dtype='int'))

# this matrix stores the top nnN nearest neighbors in the pass-runs for each failed run
topN = np.empty((len(fail_runs), nnN), dtype='int')

for i,frun in enumerate(fail_runs):
    fpert = results[frun]['perturb']
    norms = np.array([np.linalg.norm((fpert - results[srun]['perturb'])/ pert_scale) for srun in pass_runs])
    nearestN = pass_runs[np.argsort(norms)][:nnN]
    topN[i,:] = nearestN

bestWarp =  np.empty(len(fail_runs), dtype='int')
