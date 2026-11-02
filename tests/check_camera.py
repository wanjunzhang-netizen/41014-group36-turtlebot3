"""Independent projection fixtures check monocular scale, direction and rejection."""
import unittest
import numpy as np
from src.discovery.association import pose_from_pixels,observe
from src.perception.camera import PROJECTIONS,T_BC
from src.discovery.scene import load
class CameraTests(unittest.TestCase):
    def test_metric_pose_from_pixels(self):
        for distance in (.20,.4,1.,2.):
            for lateral in (-.05,.05):
                points=np.array([[distance,lateral+.09,.25],[distance,lateral-.09,.25],
                                 [distance,lateral-.09,.07],[distance,lateral+.09,.07]])
                uv=np.c_[points,np.ones(4)]@PROJECTIONS[0].T
                solved=pose_from_pixels(uv[:,:2]/uv[:,2,None])
                self.assertIsNotNone(solved)
                np.testing.assert_allclose(solved[0],points,atol=1e-6)
    def test_one_physical_camera(self):
        model,_,_=load(0)
        self.assertEqual(model.ncam,1)
        self.assertEqual(model.camera(0).name,'front_camera')
    def test_blank_image_has_no_pose(self):
        self.assertIsNone(observe(np.zeros((480,640,3),np.uint8))[0])
if __name__=='__main__':unittest.main()
