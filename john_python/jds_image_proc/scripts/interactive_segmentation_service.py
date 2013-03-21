#!/usr/bin/env python
from __future__ import division
import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--grabcut", type=int, default=True)
parser.add_argument("--test",action="store_true")
parser.add_argument("--init",choices=["height","middle"],default="height")
parser.add_argument("--erode", type=int, default=0)
parser.add_argument("--plotting", action="store_true")
(args,extra) = parser.parse_known_args()
if extra: print "warning: unrecognized arguments: %s"%",".join(extra)

from point_clouds.tabletop import get_table_height
import sensor_msgs.msg as sm
from brett2.ros_utils import pc2xyzrgb, RvizWrapper, transformPointCloud2,xyzrgb2pc,Marker
import cv2
import numpy as np
import rospy
import roslib
roslib.load_manifest("snazzy_msgs")
from snazzy_msgs.srv import *
roslib.load_manifest('tf')
import tf
from jds_utils import conversions
from scipy import stats
import time
from jds_image_proc import utils_images
import scipy.ndimage as ndi

class Globals:
    handles = []
    rviz = None
    @staticmethod
    def setup():
        cv2.namedWindow("rgb")
        Globals.rviz = RvizWrapper.create()


class GetClick:
    xy = None      
    done = False
    def callback(self,event, x, y, flags, param):
        if self.done:
            return
        elif event == cv2.EVENT_LBUTTONDOWN:
            self.xy = (x,y)
            self.done = True
            
class GetMouse:
    xy = None
    done = False
    def callback(self, event, x, y, flags, param):
        if self.done:
            return
        elif event == cv2.EVENT_MOUSEMOVE:
            self.xy = (x,y)
        elif event == cv2.EVENT_LBUTTONDOWN:
            self.xy = (x,y)
            self.done = True

def callback(req):
    xyz, rgb = pc2xyzrgb(req.cloud_in)
    rgb = rgb.copy()
    gc = GetClick()
    cv2.setMouseCallback("rgb", gc.callback)
    while not gc.done:
        cv2.imshow("rgb", rgb)
        cv2.waitKey(10)

    gm = GetMouse()
    xy_corner1 = np.array(gc.xy)

    cv2.setMouseCallback("rgb", gm.callback)
    while not gm.done:
        img = rgb.copy()

        if gm.xy is not None:
            xy_corner2 = np.array(gm.xy)
            cv2.rectangle(img, tuple(xy_corner1), tuple(xy_corner2), (0,255, 0))
        cv2.imshow("rgb",img)
        cv2.waitKey(10)
        
    xy_tl = np.array([xy_corner1, xy_corner2]).min(axis=0)
    xy_br = np.array([xy_corner1, xy_corner2]).max(axis=0)
    #mask = np.zeros(xyz.shape[:2],dtype='uint8')
    #rectangle = gm.xy + tuple(2*(center - np.r_[gm.xy]))
    #tmp1 = np.zeros((1, 13 * 5))
    #tmp2 = np.zeros((1, 13 * 5))    
    #result = cv2.grabCut(rgb,mask,rectangle,tmp1, tmp2,10,mode=cv2.GC_INIT_WITH_RECT)
        
    #from cv2 import cv
    #mask[mask == cv.GC_BGD] = 0
    #mask[mask == cv.GC_PR_BGD] = 63
    #mask[mask == cv.GC_FGD] = 255  
    #mask[mask == cv.GC_PR_FGD] = 192
    #cv2.imshow('mask',mask)
    #cv2.waitKey(10)


    #table_height = get_table_height(xyz)
    
    if args.grabcut:
        rospy.loginfo("using grabcut")
        xl, yl = xy_tl
        w, h = xy_br - xy_tl
        mask = np.zeros((h,w),dtype='uint8')    
        mask.fill(cv2.GC_PR_BGD)
        if args.init == "height":
            initial_height_thresh = stats.scoreatpercentile(xyz[yl:yl+h, xl:xl+w,2].flatten(), 50)
            mask[xyz[yl:yl+h, xl:xl+w,2] > initial_height_thresh] = cv2.GC_PR_FGD
        elif args.init == "middle":
            mask[h//4:3*h//4, w//4:3*w//4] = cv2.GC_PR_FGD
    
        tmp1 = np.zeros((1, 13 * 5))
        tmp2 = np.zeros((1, 13 * 5))    
        cv2.grabCut(rgb[yl:yl+h, xl:xl+w, :],mask,(0,0,0,0),tmp1, tmp2,10,mode=cv2.GC_INIT_WITH_MASK)
    else:
        rospy.loginfo("using table height")        
        table_height = rospy.get_param("table_height")
        xl, yl = xy_tl
        w, h = xy_br - xy_tl
        mask = np.zeros((h,w),dtype='uint8')
        mask[np.isfinite(xyz[yl:yl+h, xl:xl+w,2]) & 
             (xyz[yl:yl+h, xl:xl+w,2] > table_height+.02)] = 1

    mask = mask % 2
    if (args.erode > 0):
        mask = ndi.binary_erosion(mask, utils_images.disk(args.erode)).astype('uint8')
    contours = cv2.findContours(mask,cv2.RETR_LIST,cv2.CHAIN_APPROX_NONE)[0]
    cv2.drawContours(rgb[yl:yl+h, xl:xl+w, :],contours,-1,(0,255,0),thickness=2)
    
    cv2.imshow('rgb', rgb)
    cv2.waitKey(10)

    zsel = xyz[yl:yl+h, xl:xl+w, 2]
    mask = (mask%2==1) & np.isfinite(zsel)# & (zsel - table_height > -1)

    
    resp = ProcessCloudResponse()
    xyz_sel = xyz[yl:yl+h, xl:xl+w,:][mask.astype('bool')]
    rgb_sel = rgb[yl:yl+h, xl:xl+w,:][mask.astype('bool')]
    resp.cloud_out = xyzrgb2pc(xyz_sel, rgb_sel, req.cloud_in.header.frame_id)
    
    if (args.plotting):
        pose_array = conversions.array_to_pose_array(xyz_sel, req.cloud_in.header.frame_id)
        Globals.handles = [Globals.rviz.draw_curve(pose_array, rgba = (1,1,0,1), type=Marker.CUBE_LIST, width=.001, ns="segmentation")]
    
    return resp
    

if __name__ == "__main__":
    if args.test:        
        if rospy.get_name() == "/unnamed": 
            rospy.init_node("test_interactive_segmentation_service",disable_signals=True)
            listener = tf.TransformListener()
            pc = rospy.wait_for_message("/drop/points", sm.PointCloud2)     
            pc_tf = transformPointCloud2(pc, listener, "base_footprint", pc.header.frame_id)            
            Globals.setup()            
            req = ProcessCloudRequest()
            req.cloud_in = pc_tf
            resp = callback(req)
            xyz, rgb = pc2xyzrgb(resp.cloud_out)
            pose_array = conversions.array_to_pose_array(xyz.reshape(-1,3), 'base_footprint')
            Globals.handles.append(Globals.rviz.draw_curve(pose_array,rgba=(1,0,0,1),type=Marker.CUBE_LIST,width=.002, ns = 'segmentation'))
            
    else:
        rospy.init_node("test_interactive_segmentation_service",disable_signals=True)
        Globals.setup()
        service = rospy.Service("interactive_segmentation", ProcessCloud, callback)
        time.sleep(1000000)
