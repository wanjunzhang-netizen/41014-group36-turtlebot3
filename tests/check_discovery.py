"""Numerical and generation checks for the unknown-landmark mode."""
import unittest
import numpy as np
from src.discovery.estimator import LocalEKF, observation_model
from src.discovery.association import Measurement, observe
from src.discovery.scene import sample_layout
from src.estimation.ekf import EncoderIncrement
from src.perception.lidar import Scan

class DiscoveryTests(unittest.TestCase):
    def test_circular_zone(self):
        from src.discovery.docking import zone_center, wheels_in_zone, ZONE_RADIUS
        center=zone_center(np.array([.3,.1]),np.array([0.,1.]))
        np.testing.assert_allclose(center,[.3,.1])
        self.assertTrue(np.all(wheels_in_zone(np.array([.16,0.,np.pi]),np.zeros(2))))
        self.assertFalse(np.any(wheels_in_zone(np.array([.25,0.,np.pi]),np.zeros(2))))
        for seed in range(100):
            layout=sample_layout(seed);c=zone_center(layout['dock'])
            self.assertTrue(np.all(np.abs(c)+ZONE_RADIUS<1.5))
            for point,radius,_ in layout['obstacles']:
                self.assertGreater(np.linalg.norm(c-point),ZONE_RADIUS+radius)

    def test_measurement_jacobian_and_gauge(self):
        x=np.array([.2,-.1,.4,1.2,.6]);eps=1e-6
        z,H=observation_model(x)
        numerical=np.column_stack([(observation_model(x+np.eye(5)[i]*eps)[0]-
                                    observation_model(x-np.eye(5)[i]*eps)[0])/(2*eps) for i in range(5)])
        np.testing.assert_allclose(H,numerical,atol=1e-8)
        # Common translation/rotation cannot be observed from one relative landmark.
        moved=x.copy();moved[:2]+=[2.,-3.];moved[3:]+=[2.,-3.]
        np.testing.assert_allclose(observation_model(moved)[0],z)
        a=.3;R=np.array([[np.cos(a),-np.sin(a)],[np.sin(a),np.cos(a)]])
        moved[:2]=R@x[:2];moved[3:]=R@x[3:];moved[2]=x[2]+a
        np.testing.assert_allclose(observation_model(moved)[0],z)

    def test_joint_covariance_and_outliers(self):
        m=Measurement(1.,.1,.16,np.array([-1.,0.]),np.array([1.,0.,.16]),np.zeros((4,2)),.003,np.diag([.02**2,.01**2]))
        e=LocalEKF(m)
        for i in range(50):
            e.predict(EncoderIncrement(i*.05,.05,np.array([.01,.012]),np.array([1e-5,1e-5])))
        before=np.trace(e.P)
        z,_=observation_model(e.x);m.distance,m.bearing=z
        self.assertTrue(e.update(m))
        self.assertLess(np.trace(e.P),before)
        self.assertGreater(np.linalg.norm(e.P[:3,3:]),0)
        self.assertGreaterEqual(np.linalg.eigvalsh(e.P).min(),-1e-10)
        m.distance=50.;saved=e.x.copy()
        self.assertFalse(e.update(m));np.testing.assert_array_equal(saved,e.x)

    def test_random_layout_constraints(self):
        layouts=[sample_layout(seed) for seed in range(100)]
        for layout in layouts:
            self.assertEqual(len(layout['obstacles']),6)
            self.assertLess(np.max(abs(layout['start'])),1.5)
            self.assertLess(np.max(abs(layout['dock'])),1.5)
            for p,r,_ in layout['obstacles']:
                self.assertLess(np.max(abs(p))+r,1.5)
                self.assertGreater(np.linalg.norm(p-layout['start']),r+.32)
        self.assertGreater(np.linalg.norm(layouts[0]['start']-layouts[1]['start']),.1)
        np.testing.assert_array_equal(sample_layout(5)['start'],layouts[5]['start'])

    def test_fixed_demo_blocks_direct_line(self):
        layout=sample_layout(0)
        point,radius,cylinder=layout['obstacles'][0]
        np.testing.assert_allclose(point,layout['start']+.30*(layout['dock']-layout['start']))
        self.assertFalse(cylinder)
        self.assertEqual(len(layout['obstacles']),6)
        self.assertGreater(radius,.1)

    def test_no_marker_no_landmark(self):
        scan=Scan(0.,np.deg2rad(np.arange(-180,180)),np.ones(360),6.)
        measurement,_=observe(np.full((480,640,3),127,np.uint8))
        self.assertIsNone(measurement)

if __name__=='__main__':unittest.main()
