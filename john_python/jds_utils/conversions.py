import numpy as np
from jds_utils import transformations
try:
    import geometry_msgs.msg as gm
    import rospy
except Exception:
    print "couldn't import ros stuff"


def pose_to_trans_rot(pose):
    return (pose.position.x, pose.position.y, pose.position.z),\
           (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
def trans_rot_to_pose(trans,rot):
    pose = gm.Pose()
    pose.position.x, pose.position.y, pose.position.z = trans
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.z = rot
    return pose
def hmat_to_pose(hmat):
    trans,rot = hmat_to_trans_rot(hmat)
    return trans_rot_to_pose(trans,rot)
def pose_to_hmat(pose):
    trans,rot = pose_to_trans_rot(pose)
    hmat = trans_rot_to_hmat(trans,rot)
    return hmat

def hmat_to_trans_rot(hmat): 
    ''' 
    Converts a 4x4 homogenous rigid transformation matrix to a translation and a 
    quaternion rotation. 
    ''' 
    scale, shear, angles, trans, persp = transformations.decompose_matrix(hmat) 
    rot = transformations.quaternion_from_euler(*angles) 
    return trans, rot 

def trans_rot_to_hmat(trans, rot): 
    ''' 
    Converts a rotation and translation to a homogeneous transform. 

    **Args:** 

        **trans (np.array):** Translation (x, y, z). 

        **rot (np.array):** Quaternion (x, y, z, w). 

    **Returns:** 
        H (np.array): 4x4 homogenous transform matrix. 
    ''' 
    H = transformations.quaternion_matrix(rot) 
    H[0:3, 3] = trans 
    return H


def xya_to_trans_rot(xya):
    x,y,a = xya
    return np.r_[x, y, 0], yaw_to_quat(a)
def trans_rot_to_xya(trans, rot):
    x = trans[0]
    y = trans[1]
    a = quat_to_yaw(rot)
    return (x,y,a)

def quat_to_yaw(q):
    e = transformations.euler_from_quaternion(q)
    return e[2]
def yaw_to_quat(yaw):
    return transformations.quaternion_from_euler(0, 0, yaw)

def quat2mat(quat):
    return transformations.quaternion_matrix(quat)[:3, :3]
def mat2quat(mat33):
    mat44 = np.eye(4)
    mat44[:3,:3] = mat33
    return transformations.quaternion_from_matrix(mat44)
def mats2quats(mats):
    return np.array([mat2quat(mat) for mat in mats])
def quats2mats(quats):
    return np.array([quat2mat(quat) for quat in quats])

def rod2mat(rod):
    theta = np.linalg.norm(rod)
    if theta==0: return np.eye(3)
    
    r = rod/theta
    rx,ry,rz = r
    mat = (
        np.cos(theta)*np.eye(3)
        + (1 - np.cos(theta))*np.outer(r,r)
        + np.sin(theta)*np.array([[0,-rz,ry],[rz,0,-rx],[-ry,rx,0]]))
    return mat
    


def point_stamed_to_pose_stamped(pts,orientation=[0,0,0,1]):
    """convert pointstamped to posestamped"""
    ps = gm.PoseStamped()
    ps.pose.position = pts.point
    ps.pose.orientation = orientation
    ps.header.frame_id = pts.header.frame_id
    return ps

def array_to_pose_array(arr, frame_id):
    pose_array = gm.PoseArray()

    for (x,y,z) in arr:
        pose = gm.Pose()
        pose.position = gm.Point(x,y,z)
        pose.orientation = gm.Quaternion(0,0,0,1)
        pose_array.poses.append(pose)                        
    pose_array.header.frame_id = frame_id
    pose_array.header.stamp = rospy.Time.now()
    return pose_array
    

def trans_rot_to_pose(trans, rot):
    pose = gm.Pose()
    pose.position = gm.Point(*trans)
    pose.orientation = gm.Quaternion(*rot)
    return pose

def hmat_to_pose(hmat):
    quat = transformations.quaternion_from_matrix(hmat)
    trans = hmat[:3,3]
    return trans_rot_to_pose(trans, quat)    


def euler_matrix(ai, aj, ak, axes='sxyz'):
    """Return homogeneous rotation matrix from Euler angles and axis sequence.

    ai, aj, ak : Euler's roll, pitch and yaw angles
    axes : One of 24 axis sequences as string or encoded tuple

    >>> R = euler_matrix(1, 2, 3, 'syxz')
    >>> numpy.allclose(numpy.sum(R[0]), -1.34786452)
    True
    >>> R = euler_matrix(1, 2, 3, (0, 1, 0, 1))
    >>> numpy.allclose(numpy.sum(R[0]), -0.383436184)
    True
    >>> ai, aj, ak = (4.0*math.pi) * (numpy.random.random(3) - 0.5)
    >>> for axes in _AXES2TUPLE.keys():
    ...    R = euler_matrix(ai, aj, ak, axes)
    >>> for axes in _TUPLE2AXES.keys():
    ...    R = euler_matrix(ai, aj, ak, axes)

    """
    try:
        firstaxis, parity, repetition, frame = _AXES2TUPLE[axes]
    except (AttributeError, KeyError):
        _ = _TUPLE2AXES[axes]
        firstaxis, parity, repetition, frame = axes

    i = firstaxis
    j = _NEXT_AXIS[i+parity]
    k = _NEXT_AXIS[i-parity+1]

    if frame:
        ai, ak = ak, ai
    if parity:
        ai, aj, ak = -ai, -aj, -ak

    si, sj, sk = math.sin(ai), math.sin(aj), math.sin(ak)
    ci, cj, ck = math.cos(ai), math.cos(aj), math.cos(ak)
    cc, cs = ci*ck, ci*sk
    sc, ss = si*ck, si*sk

    M = numpy.identity(4)
    if repetition:
        M[i, i] = cj
        M[i, j] = sj*si
        M[i, k] = sj*ci
        M[j, i] = sj*sk
        M[j, j] = -cj*ss+cc
        M[j, k] = -cj*cs-sc
        M[k, i] = -sj*ck
        M[k, j] = cj*sc+cs
        M[k, k] = cj*cc-ss
    else:
        M[i, i] = cj*ck
        M[i, j] = sj*sc-cs
        M[i, k] = sj*cc+ss
        M[j, i] = cj*sk
        M[j, j] = sj*ss+cc
        M[j, k] = sj*cs-sc
        M[k, i] = -sj
        M[k, j] = cj*si
        M[k, k] = cj*ci
    return M


def euler2matrix(ai, aj, ak, axes='sxyz'):
    """Return homogeneous rotation matrix from Euler angles and axis sequence.

    ai, aj, ak : Euler's roll, pitch and yaw angles
    axes : One of 24 axis sequences as string or encoded tuple

    >>> R = euler_matrix(1, 2, 3, 'syxz')
    >>> numpy.allclose(numpy.sum(R[0]), -1.34786452)
    True
    >>> R = euler_matrix(1, 2, 3, (0, 1, 0, 1))
    >>> numpy.allclose(numpy.sum(R[0]), -0.383436184)
    True
    >>> ai, aj, ak = (4.0*math.pi) * (numpy.random.random(3) - 0.5)
    >>> for axes in _AXES2TUPLE.keys():
    ...    R = euler_matrix(ai, aj, ak, axes)
    >>> for axes in _TUPLE2AXES.keys():
    ...    R = euler_matrix(ai, aj, ak, axes)

    """
    try:
        firstaxis, parity, repetition, frame = _AXES2TUPLE[axes]
    except (AttributeError, KeyError):
        _ = _TUPLE2AXES[axes]
        firstaxis, parity, repetition, frame = axes

    i = firstaxis
    j = _NEXT_AXIS[i+parity]
    k = _NEXT_AXIS[i-parity+1]

    if frame:
        ai, ak = ak, ai
    if parity:
        ai, aj, ak = -ai, -aj, -ak

    si, sj, sk = math.sin(ai), math.sin(aj), math.sin(ak)
    ci, cj, ck = math.cos(ai), math.cos(aj), math.cos(ak)
    cc, cs = ci*ck, ci*sk
    sc, ss = si*ck, si*sk

    M = numpy.identity(4)
    if repetition:
        M[i, i] = cj
        M[i, j] = sj*si
        M[i, k] = sj*ci
        M[j, i] = sj*sk
        M[j, j] = -cj*ss+cc
        M[j, k] = -cj*cs-sc
        M[k, i] = -sj*ck
        M[k, j] = cj*sc+cs
        M[k, k] = cj*cc-ss
    else:
        M[i, i] = cj*ck
        M[i, j] = sj*sc-cs
        M[i, k] = sj*cc+ss
        M[j, i] = cj*sk
        M[j, j] = sj*ss+cc
        M[j, k] = sj*cs-sc
        M[k, i] = -sj
        M[k, j] = cj*si
        M[k, k] = cj*ci
    return M