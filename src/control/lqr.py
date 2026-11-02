"""Adapted from Assessment 3 A.1/A.2: straight-segment discrete LQR.
Reference changes and final slowdown are docking-task adaptations.
"""
import numpy as np
from src.estimation.ekf import wrap

# 根据误差模型和 LQR 的 Q、R 计算反馈增益 K
def design(v=.08,dt=.05):
    # Exact ZOH of A=[[0,v],[0,0]], B=[[0],[1]], kappa=0 (A3 A.1).
    A=np.array([[1.,v*dt],[0.,1.]])
    B=np.array([[.5*v*dt**2],[dt]])
    Q=np.diag([1/.10**2,1/np.deg2rad(10.)**2])
    R=np.array([[1/.30**2]]) # A3 A.2 Bryson weights
    # Discrete Riccati iteration: same DARE as ct.dlqr, NumPy implementation.
    P=Q.copy()
    for _ in range(20000):
        K=np.linalg.solve(R+B.T@P@B,B.T@P@A)
        new=Q+A.T@P@A-A.T@P@B@K
        if np.max(abs(new-P))<1e-10:
            P=new;break
        P=new
    else:raise RuntimeError('LQR Riccati iteration did not converge')
    K=np.linalg.solve(R+B.T@P@B,B.T@P@A)
    return A,B,K

# 计算小车相对路径段的横向误差和航向误差，输出前进速度forward speed、转向速度turning speed
class SegmentTracker:
    def __init__(self):
        self.A,self.B,self.K=design();self.target=None;self.origin=None

    def command(self,pose,target):
        target=np.asarray(target)
        if self.target is None or np.linalg.norm(target-self.target)>1e-6:
            self.target=target.copy();self.origin=pose[:2].copy()
        delta=target-pose[:2];rho=np.linalg.norm(delta)
        line=target-self.origin
        heading=np.arctan2(line[1],line[0]) if np.linalg.norm(line)>1e-8 else pose[2]
        e_y=np.array([-np.sin(heading),np.cos(heading)])@(pose[:2]-self.origin)
        e_theta=wrap(pose[2]-heading)
        # A3: omega = v*kappa - K*[e_y,e_theta], kappa=0 for each segment.
        omega=float(np.clip(-(self.K@np.array([e_y,e_theta])).item(),-.6,.6))
        alpha=wrap(np.arctan2(delta[1],delta[0])-pose[2])
        v=min(.08,.8*rho)*max(0.,np.cos(alpha))
        if abs(alpha)>np.deg2rad(20):v=0.
        return float(v),omega
