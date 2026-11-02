"""Disk at marker ground projection; wheels must touch its floor area."""
import numpy as np
# charging area is circular with a radius of 20 centimeters
ZONE_RADIUS=.20
DOCK_OFFSET=.16
WHEEL_HALF_TRACK=.08
def zone_center(landmark,normal=None):return np.asarray(landmark).copy()
def dock_target(landmark,normal):return np.asarray(landmark)+DOCK_OFFSET*np.asarray(normal)
def estimated_wheel_points(pose):
    side=np.array([-np.sin(pose[2]),np.cos(pose[2])])*WHEEL_HALF_TRACK
    return np.array([pose[:2]+side,pose[:2]-side])
def wheels_in_zone(pose,landmark,margin=.005):
    return np.linalg.norm(estimated_wheel_points(pose)-landmark,axis=1)<=ZONE_RADIUS-margin
