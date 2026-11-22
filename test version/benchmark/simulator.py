"""Evaluator-owned static world. No policy reference or GT back-channel."""
import math
import numpy as np
from .geometry import target_box


def lidar(pose,boxes,cc):
    x,y,yaw=pose;c,s=math.cos(yaw),math.sin(yaw)
    origin=np.array([x+c*cc.scan_x-s*cc.scan_y,y+s*cc.scan_x+c*cc.scan_y])
    angles=np.linspace(-math.pi,math.pi,720,endpoint=False)+yaw+cc.scan_yaw
    directions=np.column_stack((np.cos(angles),np.sin(angles)))
    ranges=np.full(720,np.inf)
    for box in boxes:
        near,far=np.full(720,-np.inf),np.full(720,np.inf);valid=np.ones(720,dtype=bool)
        for axis in range(2):
            parallel=np.abs(directions[:,axis])<1e-12
            valid &= ~parallel | ((origin[axis]>=box[axis])&(origin[axis]<=box[axis+2]))
            divisor=np.where(parallel,1.,directions[:,axis])
            a,b=(box[axis]-origin[axis])/divisor,(box[axis+2]-origin[axis])/divisor
            near=np.maximum(near,np.where(parallel,-np.inf,np.minimum(a,b)))
            far=np.minimum(far,np.where(parallel,np.inf,np.maximum(a,b)))
        hit=valid&(far>=np.maximum(near,0.))
        distance=np.where(near>=0,near,far)
        readings=np.where(hit&(distance<=6.),distance,np.inf)
        readings=np.where(hit&(distance<.05),np.nan,readings)
        ranges=np.minimum(ranges,readings)
    return tuple(float(v) for v in ranges)


def world_boxes(episode):
    return [list(o['bounds_m']) for o in episode['obstacles']]+[target_box(t) for t in episode['targets']]
