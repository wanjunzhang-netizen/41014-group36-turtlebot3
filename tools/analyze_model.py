"""Moving-platform controllability and unknown-landmark observation rank."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
from src.discovery.estimator import observation_model
for speed in (0.,.1):
    A=np.array([[0,0,0],[0,0,speed],[0,0,0.]])
    B=np.array([[1,0],[0,0],[0,1.]])
    print('linearized robot controllability, v=',speed,'rank=',np.linalg.matrix_rank(np.column_stack((B,A@B,A@A@B))))
x=np.array([0.,0.,.2,1.,.5]);_,H=observation_model(x)
print('instantaneous joint observation rank:',np.linalg.matrix_rank(H))
print('Common translation/rotation gauge is fixed by the first-observation local frame.')

from src.control.lqr import design
Ad,Bd,K=design()
print("A3-adapted LQR gain:",K)
print("Segment controllability rank:",np.linalg.matrix_rank(np.c_[Bd,Ad@Bd]))
print("Closed-loop eigenvalue magnitudes:",abs(np.linalg.eigvals(Ad-Bd@K)))
