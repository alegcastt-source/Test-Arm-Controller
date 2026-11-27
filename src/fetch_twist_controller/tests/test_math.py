import sys
from pathlib import Path
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from fetch_twist_controller.control_math import solve_velocity,transform_twist

class Numerics(unittest.TestCase):
    def test_well_conditioned_tracking(self):
        J=np.column_stack((np.eye(6),np.zeros(6)))
        v=np.array([.01,.02,-.03,.04,-.05,.06])
        q,lam,s=solve_velocity(J,v,np.ones(7))
        np.testing.assert_allclose(J@q,v,atol=1e-12)
        self.assertEqual(lam,0)
    def test_singular_is_finite_and_limited(self):
        J=np.diag([1,1,1,1,.001,0.]); v=np.ones(6)
        q,lam,s=solve_velocity(J,v,[.1]*6)
        self.assertTrue(np.isfinite(q).all());self.assertGreater(lam,0)
        self.assertLessEqual(abs(q).max(),.1+1e-12)
    def test_tip_twist_rotation_not_translation(self):
        R=np.array([[0,-1,0],[1,0,0],[0,0,1]])
        v=np.array([1,2,3,4,5,6.])
        got=transform_twist(v,R,[2,3,4],[.1,.5,.8],'tip')
        np.testing.assert_allclose(got,np.r_[R@v[:3],R@v[3:]],atol=1e-12)
    def test_origin_rotation_lever_arm(self):
        # Rotation about source origin x=1: tip x=2 has +y velocity.
        got=transform_twist([0,0,0,0,0,1],np.eye(3),[1,0,0],[2,0,0],'origin')
        np.testing.assert_allclose(got,[0,1,0,0,0,1],atol=1e-12)
    def test_limit_preserves_direction(self):
        q,_,_=solve_velocity(np.eye(6),np.arange(1,7.),np.ones(6)*.1)
        np.testing.assert_allclose(q,np.arange(1,7.)/60,atol=1e-12)

if __name__=='__main__': unittest.main()
