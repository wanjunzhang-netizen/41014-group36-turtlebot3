"""Regression: alignment should not chatter across the 5 cm staging boundary."""
import unittest
from types import SimpleNamespace
import numpy as np
from src.discovery.navigation import Navigator
from src.perception.lidar import Scan
class AlignmentTests(unittest.TestCase):
    def setUp(self):
        self.n=Navigator(0)
        self.e=SimpleNamespace(x=np.array([.551,0.,.5,1.,0.]),normal=np.array([-1.,0.]))
        self.scan=Scan(0.,np.deg2rad(np.arange(-180,180)),np.full(360,3.),6.)
    def call(self,age=0.):return self.n.command(self.e,np.zeros(3),self.scan,age)
    def test_small_boundary_changes_do_not_reverse_phase(self):
        self.assertEqual(self.call()[2],'ALIGN TO MARKER')
        for x in (.549,.551,.54,.56):
            self.e.x[0]=x
            v,w,status=self.call()
            self.assertEqual(self.n.state,'ALIGN')
            self.assertEqual(v,0.)
            self.assertLess(w,0.)
    def test_large_displacement_returns_to_approach(self):
        self.call();self.e.x[0]=.4
        self.call()
        self.assertEqual(self.n.state,'APPROACH')
    def test_missing_camera_does_not_enter_final(self):
        self.e.x[2]=0.
        self.assertEqual(self.call(2.)[2],'ALIGN: REACQUIRE CAMERA')
        self.assertEqual(self.n.state,'ALIGN')
        self.assertEqual(self.call(0.)[2],'FINAL: ENTER')
if __name__=='__main__':unittest.main()
