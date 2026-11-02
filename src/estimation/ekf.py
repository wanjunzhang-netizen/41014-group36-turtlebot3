"""Shared differential-drive motion and encoder covariance primitives."""
from dataclasses import dataclass
import numpy as np

RADIUS, TRACK = .033, .16


def wrap(angle):
    return (angle + np.pi) % (2*np.pi) - np.pi


def _arc_scale(dtheta):
    # Assessment 3 B.1 exact constant-turn arc, stable at zero turn.
    a = dtheta/2
    if abs(a)<1e-5:
        return 1-a*a/6+a**4/120, -a/6+a**3/60
    return np.sin(a)/a, (a*np.cos(a)-np.sin(a))/(2*a*a)

# 预测predict下一时刻的位姿pose
def motion(pose, ds, dtheta):
    scale,_ = _arc_scale(dtheta)
    angle = pose[2]+dtheta/2
    return np.array([pose[0]+ds*scale*np.cos(angle),
                     pose[1]+ds*scale*np.sin(angle),wrap(pose[2]+dtheta)])

# 计算 EKF 传播不确定性需要的矩阵
def jacobians(pose, ds, dtheta):
    scale,derivative = _arc_scale(dtheta)
    a=pose[2]+dtheta/2;c,s=np.cos(a),np.sin(a)
    F=np.array([[1,0,-ds*scale*s],[0,1,ds*scale*c],[0,0,1.]])
    # Encoder-increment Jacobian is a project adaptation to A3 command-input model.
    G=np.array([[scale*c,ds*(derivative*c-.5*scale*s)],
                [scale*s,ds*(derivative*s+.5*scale*c)],[0,1.]])
    return F,G


@dataclass
class EncoderIncrement:
    time: float
    dt: float
    wheel_delta: np.ndarray
    wheel_variance: np.ndarray

# 把左右轮转动换算成前进距离和转角
def control_increment(reading):
    J = np.array([[RADIUS/2,RADIUS/2],[-RADIUS/TRACK,RADIUS/TRACK]])
    return J@reading.wheel_delta, J@np.diag(reading.wheel_variance)@J.T


