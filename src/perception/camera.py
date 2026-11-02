"""Single calibrated camera: optical x right, y down, z forward."""
import cv2
import numpy as np
WIDTH,HEIGHT=640,480
FOVY=100.
MARKER_ID=7
MARKER_SIZE=.18
DICTIONARY=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
f=HEIGHT/(2*np.tan(np.deg2rad(FOVY)/2))
# Camera intrinsic parameter matrix K
K=np.array([[f,0,(WIDTH-1)/2],[0,f,(HEIGHT-1)/2],[0,0,1.]])
DISTORTION=np.zeros(5)
def extrinsic(height,pitch_deg):
    p=np.deg2rad(pitch_deg);c,s=np.cos(p),np.sin(p)
    T=np.eye(4);T[:3,:3]=np.column_stack(([0,-1,0],[s,0,-c],[c,0,s]))
    T[:3,3]=[.064,0,height]
    return T
T_BC=extrinsic(.125,10.)
TRANSFORMS=(T_BC,)
CAMERA_NAMES=('front_camera',)
PROJECTIONS=(K@np.linalg.inv(T_BC)[:3,:],)
