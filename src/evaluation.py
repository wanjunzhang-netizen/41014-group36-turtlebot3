"""Evaluation only: truth and covariance are logged here, never sent to control."""
import csv
import numpy as np
from src.estimation.ekf import wrap

CHI2_95_2D=5.991464547

def ellipse_parameters(P):
    values,vectors=np.linalg.eigh((P[:2,:2]+P[:2,:2].T)/2)
    order=np.argsort(values)[::-1]
    values=np.maximum(values[order],0);vectors=vectors[:,order]
    return 2*np.sqrt(CHI2_95_2D*values),float(np.degrees(np.arctan2(vectors[1,0],vectors[0,0])))

