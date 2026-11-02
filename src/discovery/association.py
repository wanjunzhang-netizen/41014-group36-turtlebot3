"""Monocular pixel-derived range/bearing: no LiDAR or simulator state input."""
from dataclasses import dataclass
import cv2
import numpy as np
from src.perception.camera import DICTIONARY,MARKER_ID,MARKER_SIZE,K,DISTORTION,T_BC
@dataclass
class Measurement:
    distance:float
    bearing:float
    height:float
    normal:np.ndarray
    point:np.ndarray
    corners:np.ndarray
    residual:float
    covariance:np.ndarray

# 在图片中寻找标记四个角点
def marker_corners(rgb):
    params=cv2.aruco.DetectorParameters();params.perspectiveRemovePixelPerCell=8;params.minMarkerDistanceRate=.01;params.cornerRefinementMethod=cv2.aruco.CORNER_REFINE_SUBPIX
    corners,ids,_=cv2.aruco.ArucoDetector(DICTIONARY,params).detectMarkers(cv2.cvtColor(rgb,cv2.COLOR_RGB2GRAY))
    if ids is None or MARKER_ID not in ids.ravel():return None
    matches=[corners[i].reshape(4,2) for i,tag in enumerate(ids.ravel()) if tag==MARKER_ID]
    quad=max(matches,key=lambda q:abs(cv2.contourArea(q))).astype(np.float32)
    side=np.linalg.norm(quad-np.roll(quad,1,axis=0),axis=1).min()
    window=max(3,min(21,int(.12*side)))
    cv2.cornerSubPix(cv2.cvtColor(rgb,cv2.COLOR_RGB2GRAY),quad,(window,window),(-1,-1),
                     (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER,100,.001))
    cv2.cornerSubPix(cv2.cvtColor(rgb,cv2.COLOR_RGB2GRAY),quad,(3,3),(-1,-1),
                     (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER,100,.001))
    return quad.astype(float)


# Known marker size supplies metric scale; no simulator pose enters this function.
HALF=MARKER_SIZE/2
OBJECT_POINTS=np.array([[-HALF,HALF,0],[HALF,HALF,0],
                        [HALF,-HALF,0],[-HALF,-HALF,0]],dtype=float)

def pose_from_pixels(corners):
    ok,rvec,tvec=cv2.solvePnP(OBJECT_POINTS,np.asarray(corners,dtype=float),K,DISTORTION,
                             flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:return None
    rotation,_=cv2.Rodrigues(rvec)
    camera_points=OBJECT_POINTS@rotation.T+tvec.reshape(3)
    if np.min(camera_points[:,2])<.025:return None
    projected,_=cv2.projectPoints(OBJECT_POINTS,rvec,tvec,K,DISTORTION)
    residual=float(np.sqrt(np.mean(np.sum((projected.reshape(4,2)-corners)**2,axis=1))))
    points=camera_points@T_BC[:3,:3].T+T_BC[:3,3]
    return points,residual

def observe(rgb):
    corners=marker_corners(rgb)
    if corners is None:return None,'SEARCH: MARKER NOT SEEN'
    solved=pose_from_pixels(corners)
    if solved is None:return None,'REJECT CAMERA POSE'
    points,residual=solved
    if residual>1.5:return None,'REJECT REPROJECTION ERROR'
    center=points.mean(axis=0)
    n=np.cross(points[1]-points[0],points[3]-points[0]);n/=np.linalg.norm(n)
    if abs(n[2])>.3:return None,'REJECT NONUPRIGHT BOARD'
    normal=n[:2]/np.linalg.norm(n[:2])
    if normal@center[:2]>0:normal=-normal
    distance=float(np.linalg.norm(center[:2]))
    if not .09<distance<3.5 or not .06<center[2]<.32:return None,'REJECT CAMERA RANGE'
    if -normal@(center[:2]/distance)<.4:return None,'REJECT GRAZING VIEW'
    # Design uncertainty, not an extra random perturbation. Range degrades with distance.
    R=np.diag([(.008+.012*distance**2)**2,np.deg2rad(1.)**2])
    return Measurement(distance,float(np.arctan2(center[1],center[0])),float(center[2]),
                       normal,center,corners,residual,R),'PIXELS -> RANGE + BEARING'
