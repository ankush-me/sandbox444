from __future__ import division
import cPickle
import numpy as np
import matplotlib.pyplot as plot

fformat = "/home/ankush/sandbox444/pnpApp/run2_results/run-%d2-results.cpickle"
merge_fname = "/home/ankush/sandbox444/pnpApp/run2_results/merged-results.cpickle"

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
    
plot.plot(np.arange(10)+1, cube_rates)
plot.xlabel('scaling steps')
plot.ylabel('success ratio')
plot.savefig("scalings.png")


# get hamming success-ratios:
n_runs = 640.0
hamm_rates = {}
pert_scale = np.array([-0.0025, 0.005, 0.005, 5, 5, 5])

results = cPickle.load(open(merge_fname, 'r'))
print len(results.keys())
for item in results.iteritems():
    hamm_dist = np.sum(np.round(item[1]['perturb']/pert_scale, 3))

    if item[1]['result']:  ## if the run was a success:
        if not hamm_rates.has_key(hamm_dist):
            hamm_rates[hamm_dist] = 1
        else:
            hamm_rates[hamm_dist] += 1
    else:
        if not hamm_rates.has_key(hamm_dist):
            hamm_rates[hamm_dist] = 0

hx = np.arange(max(hamm_rates.keys())+1)
hy = np.zeros(hx.shape)
for it in hamm_rates.iteritems():
    hy[int(it[0])] = it[1]

plot.clf()
plot.scatter(hx,hy)
plot.plot(hx, hy)
plot.xlabel('hamming dist')
plot.ylabel('num successes')
plot.savefig('hamming.png')
