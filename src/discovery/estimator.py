"""Adapted from Assessment 3 B.3 (Joseph EKF).
Camera range/bearing, joint unknown landmark and initialization are project extensions.
Five-state joint EKF: robot x,y,yaw plus newly observed landmark lx,ly.
The origin is the first associated observation's body frame; no world prior.
"""
import numpy as np
from src.estimation.ekf import motion,jacobians,control_increment,wrap


def observation_model(x):
    dx,dy=x[3:]-x[:2];q=dx*dx+dy*dy
    if q<1e-10:raise ValueError('Robot and landmark coincide')
    r=np.sqrt(q)
    H=np.array([[-dx/r,-dy/r,0,dx/r,dy/r],[dy/q,-dx/q,-1,-dy/q,dx/q]])
    return np.array([r,wrap(np.arctan2(dy,dx)-x[2])]),H

# 
class LocalEKF:
    # Initialize robot and station estimates at the first accepted marker observation.
    def __init__(self,measurement):
        r,b=measurement.distance,measurement.bearing
        self.x=np.array([0.,0.,0.,r*np.cos(b),r*np.sin(b)])
        J=np.array([[np.cos(b),-r*np.sin(b)],[np.sin(b),r*np.cos(b)]])
        self.P=np.zeros((5,5));self.P[3:,3:]=J@measurement.covariance@J.T
        self.normal=measurement.normal.copy();self.height=measurement.height
        self.accepted=0;self.rejected=0
    # Predict the state and uncertainty from wheel encoders.
    def predict(self,reading):
        (ds,da),Q=control_increment(reading)
        Q+=np.diag([.004**2*reading.dt+(.03*abs(ds))**2,np.deg2rad(1.5)**2*reading.dt+(.08*abs(da))**2])
        F,G=jacobians(self.x[:3],ds,da);A=np.eye(5);A[:3,:3]=F
        B=np.zeros((5,2));B[:3]=G
        self.x[:3]=motion(self.x[:3],ds,da);self.P=A@self.P@A.T+B@Q@B.T
    # Correct the prediction using camera range and bearing.
    def update(self,m):
        R=m.covariance
        predicted,H=observation_model(self.x)
        y=np.array([m.distance,m.bearing])-predicted;y[1]=wrap(y[1])
        S=H@self.P@H.T+R;nis=float(y@np.linalg.solve(S,y))
        if nis>16:self.rejected+=1;return False
        K=np.linalg.solve(S,H@self.P).T
        self.x+=K@y;self.x[2]=wrap(self.x[2]);A=np.eye(5)-K@H
        self.P=A@self.P@A.T+K@R@K.T;self.P=(self.P+self.P.T)/2
        c,s=np.cos(self.x[2]),np.sin(self.x[2]);n=np.array([[c,-s],[s,c]])@m.normal
        if self.normal@n>.5:
            self.normal=.9*self.normal+.1*n;self.normal/=np.linalg.norm(self.normal)
        self.height=.9*self.height+.1*m.height
        self.accepted+=1;return True
