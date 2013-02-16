import numpy as np
import scipy.interpolate as si


def joinJoints(j1, j2):
    assert(j1.dtype==j2.dtype)
    atype = j1.dtype
    param  = np.array([0,0.33, 0.66, 1])



    j1_r  = np.concatenate([j1['r_arm'], [j1['r_gripper']]])
    j2_r  = np.concatenate([j2['r_arm'], [j2['r_gripper']]])
    j1_l  = np.concatenate([j1['l_arm'], [j1['l_gripper']]])
    j2_l  = np.concatenate([j2['l_arm'], [j2['l_gripper']]])
    

    combined_r  = np.concatenate([[j1_r], [0.67*j1_r + 0.33*j2_r], [0.33*j1_r + 0.67*j2_r], [j2_r]])
    combined_l  = np.concatenate([[j1_l], [0.67*j1_l + 0.33*j2_l], [0.33*j1_l + 0.67*j2_l], [j2_l]])
    
    N = 100
    
    (r_tck, _) = si.splprep(combined_r.T, s=0.3, u=param, k=3)                                 
    smooth_r = np.r_[si.splev(np.linspace(0,1,N), r_tck, der=0)].T                               
    
    (l_tck, _) = si.splprep(combined_l.T, s=0.3, u=param, k=3)                                 
    smooth_l   = np.r_[si.splev(np.linspace(0,1,N), l_tck, der=0)].T                               
    

    smooth  = np.zeros(N, dtype=atype)
    for i in xrange(0,N):
        smooth[i]['r_arm'][:]     = smooth_r[i][:-1]
        smooth[i]['r_gripper']    = smooth_r[i][-1]
        smooth[i]['l_arm'][:]     = smooth_l[i][:-1]
        smooth[i]['l_gripper']    = smooth_l[i][-1]
    
    return smooth
