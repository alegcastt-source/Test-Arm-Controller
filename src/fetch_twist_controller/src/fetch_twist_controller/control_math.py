#!/usr/bin/env python3
"""ROS-independent numerics. Twists ordered [vx,vy,vz,wx,wy,wz]."""
import numpy as np

def skew(p):
    x,y,z=p
    return np.array([[0,-z,y],[z,0,-x],[-y,x,0]],dtype=float)

def transform_twist(twist, rotation, translation, tip_position, reference='tip'):
    """Adjoint source->base followed by shift from base origin to tip.

    reference=tip: input linear velocity is at the tip, expressed in source axes.
    reference=origin: input is the spatial twist referenced at source origin.
    The Jacobian always describes velocity of the tip, expressed in base axes.
    """
    t=np.asarray(twist,dtype=float).copy()
    R=np.asarray(rotation); p=np.asarray(translation); tip=np.asarray(tip_position)
    if reference=='tip':
        r_source=R.T @ (tip-p)
        t[:3]-=np.cross(t[3:],r_source)
    elif reference!='origin':
        raise ValueError('reference must be tip or origin')
    A=np.zeros((6,6)); A[:3,:3]=R; A[3:,3:]=R; A[:3,3:]=skew(p) @ R
    out=A @ t
    out[:3]+=np.cross(out[3:],tip)
    return out

def solve_velocity(J, twist, limits, damping=0.08, threshold=0.08):
    U,s,Vt=np.linalg.svd(J,full_matrices=False)
    lam=damping*max(0.0,1.0-(s[-1]/threshold)**2)**0.5
    denom=s*s+lam*lam
    gains=np.divide(s,denom,out=np.zeros_like(s),where=denom>1e-16)
    qdot=Vt.T @ (gains*(U.T @ twist))
    scale=max(1.0,float(np.max(np.abs(qdot)/np.asarray(limits))))
    return qdot/scale,lam,float(s[-1])
