import re
import numpy as np
from rapprentice.colorize import *


class Grips:
    """
    Enums for robot grasps
    """
    RELEASE_R = 0
    GRAB_R    = 1
    RELEASE_L = 2
    GRAB_L    = 3


def getnewseg():
    return {'name':'', 'jtimes' : [], 'joints':[], 'ptimes':[], 'points':[], 'gtimes':[], 'grips':[]}


def readpoints(sfile):
    points = []
    firstDone = False
    secCount = 0
    secCounts = []
    ptypes = []
    for line in sfile:
        if 'end-points' in line:
            secCounts.append(secCount)
            return (points, ptypes, secCounts)
        
        if line.startswith('\t') and not line.startswith('\t\t'):
            ptypes.append(line.split()[0])
            if firstDone:
                secCounts.append(secCount)
                secCount = 0
            else:
                firstDone = True
        else:
            point = [float(coord) for coord in line.split()]
            points.append(point)
            secCount += 1


def parsescene(fname):
    sfile = open(fname, 'r')
    match = re.search('run(\d*)\.txt', fname)
    if match:
        runnum = match.group(1)
    else:
        print colorize('[ERROR : Scene parser] : Not a valid scene file path.', 'red', True)


    # fast-forward to the first look:
    try:
        while('look' not in sfile.next()):
            pass
    except StopIteration:
        print colorize("[ERROR : Scene parser] : First look not found", "red", True)
        raise RuntimeError

    
    segments = []

    currseg = getnewseg()

    for line in sfile:
        splitline = line.split()
        
        if splitline[1] != ":" :
            print colorize("[ERROR : Scene parser] : Error in format -- prompt missing.", "red", True)
            raise RuntimeError
            
        cmd = splitline[2]
        timestamp = float(splitline[0])
            
        if cmd == 'points':
            currseg['ptimes'].append(timestamp)
            points, ptypes, secCounts = readpoints(sfile)
            
            if not currseg.has_key('ptypes'):
                currseg['ptypes'] = ptypes

            # this 'point_secs' is a big hack
            if not currseg.has_key('point_secs'):
                currseg['point_secs'] = secCounts
               
            currseg['points'].append(points)

        elif cmd == 'joints':
            joints = [float(j) for j in splitline[4:]]
            currseg['jtimes'].append(timestamp)
            currseg['joints'].append(joints)
            
        elif cmd == 'grab':
            grab = Grips.GRAB_R if splitline[3]=='r' else Grips.GRAB_L
            currseg['gtimes'].append(timestamp)
            currseg['grips'].append(grab)
            
        elif cmd == 'release':
            release = Grips.RELEASE_R if splitline[3]=='r' else Grips.RELEASE_L
            currseg['gtimes'].append(timestamp)
            currseg['grips'].append(release)

        elif cmd == 'look':
            segments.append(currseg)
            currseg = getnewseg()
    
    segments.append(currseg)


    # post-process each segment after reading them:
    for i,seg in enumerate(segments):
        # name each segment
        seg['name'] = 'run%s-seg%d'%(runnum, i)
        
        # numpy-ze the data for each segment
        seg['joints'] = np.array(seg['joints'])
        
        seg['points'] = np.array(seg['points'])
        # correct for relative transformation in bulletsim
        seg['points'] -= np.array((0,0, 0.05))
        
        seg['jtimes'] = np.array(seg['jtimes'])
        seg['gtimes'] = np.array(seg['gtimes'])
        seg['ptimes'] = np.array(seg['ptimes'])
        
        # associate point-clouds with each joint-set:
        seg['j2ptimes'] = np.searchsorted(seg['ptimes'], seg['jtimes'], side='left') - 1

    return segments
