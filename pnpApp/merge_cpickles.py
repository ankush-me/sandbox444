import cPickle
from os import path as osp
from rapprentice.colorize import *

fformat = "/home/ankush/sandbox444/pnpApp/run2_results/run-%d2-results.cpickle"
merge_fname = "/home/ankush/sandbox444/pnpApp/run2_results/merged-results.cpickle"

mergedict = {}

for runnum in range(10):
    runinfo = cPickle.load(open(fformat % (runnum+1), 'r'))
    mergedict.update(runinfo)

cPickle.dump(mergedict, open(merge_fname,'wb'))
print colorize("Saved merged results to : " + merge_fname, "red", True)
