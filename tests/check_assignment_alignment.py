"""A3 formula equivalence, Jacobians and closed-loop checks."""
import unittest
import numpy as np
from src.estimation.ekf import motion,jacobians,wrap
from src.control.lqr import design,SegmentTracker

class AssignmentTests(unittest.TestCase):
    def test_arc_matches_a3(self):
        x=np.array([.3,-.2,.4]);dt=.05
        for v in (0.,.025,.08):
            for w in (0.,1e-9,.2,-.6):
                theta=x[2]+w*dt
                if abs(w)<1e-8:xy=x[:2]+v*dt*np.array([np.cos(x[2]),np.sin(x[2])])
                else:xy=x[:2]+v/w*np.array([np.sin(theta)-np.sin(x[2]),np.cos(x[2])-np.cos(theta)])
                np.testing.assert_allclose(motion(x,v*dt,w*dt),[*xy,wrap(theta)],atol=1e-10)

    def test_arc_jacobians(self):
        x=np.array([.3,-.2,.4]);eps=1e-6
        for turn in (0.,1e-7,.4,-.6):
            u=np.array([.1,turn]);F,G=jacobians(x,*u)
            Fn=np.column_stack([(motion(x+np.eye(3)[i]*eps,*u)-motion(x-np.eye(3)[i]*eps,*u))/(2*eps) for i in range(3)])
            Gn=np.column_stack([(motion(x,*(u+np.eye(2)[i]*eps))-motion(x,*(u-np.eye(2)[i]*eps)))/(2*eps) for i in range(2)])
            np.testing.assert_allclose(F,Fn,atol=1e-8);np.testing.assert_allclose(G,Gn,atol=1e-8)

    def test_lqr_stability_and_correction(self):
        A,B,K=design()
        self.assertLess(max(abs(np.linalg.eigvals(A-B@K))),1)
        self.assertEqual(np.linalg.matrix_rank(np.c_[B,A@B]),2)
        t=SegmentTracker();t.command(np.array([0.,0.,0.]),np.array([1.,0.]))
        _,w=t.command(np.array([.2,.05,.1]),np.array([1.,0.]))
        self.assertLess(w,0) # Left/CCW error must steer clockwise.
        self.assertGreaterEqual(t.command(np.array([.2,.05,.1]),np.array([1.,0.]))[0],0)

if __name__=='__main__':unittest.main()
