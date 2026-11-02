import unittest
import numpy as np
from src.control.avoidance import LocalAvoidance
from src.perception.lidar import Scan
class WallTests(unittest.TestCase):
    def scan(self,distance=1.):
        a=np.deg2rad(np.arange(-180,180,dtype=float));r=np.full(360,np.inf)
        sector=abs(a)<np.deg2rad(70);r[sector]=distance/np.cos(a[sector])
        return Scan(0.,a,r,6.)
    def test_direct_wall_rejected(self):
        s=self.scan();self.assertTrue(LocalAvoidance().heads_into_visible_wall(s,np.array([.55,0.])))
    def test_parallel_allowed(self):
        self.assertFalse(LocalAvoidance().heads_into_visible_wall(self.scan(),np.array([0.,.55])))
    def test_foreground_object_not_mistaken_for_wall(self):
        s=self.scan();s.ranges[abs(s.angles)<np.deg2rad(10)]=.4
        self.assertFalse(LocalAvoidance().heads_into_visible_wall(s,np.array([.55,0.])))
    def test_distant_wall_not_deadlock(self):
        self.assertFalse(LocalAvoidance().heads_into_visible_wall(self.scan(2.),np.array([.55,0.])))
    def test_missing_data_not_clear_wall(self):
        s=self.scan();s.ranges[175:190]=np.nan
        self.assertFalse(LocalAvoidance().heads_into_visible_wall(s,np.array([.55,0.])))
if __name__=='__main__':unittest.main()
