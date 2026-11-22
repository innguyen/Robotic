"""Estimated camera coverage from an observed costmap only; never GT raycast."""
import math
import numpy as np


def camera_pose(pose,camera):
    x,y,yaw=pose;c,s=math.cos(yaw),math.sin(yaw)
    return (x+c*camera['x_m']-s*camera['y_m'],y+s*camera['x_m']+c*camera['y_m'],yaw+camera['yaw_rad'])


def coverage_cells(grid,pose,camera):
    x,y,yaw=camera_pose(pose,camera)
    half=math.radians(camera['fov_deg'])/2
    rays=max(2,math.ceil(2*half/max(.005,grid.resolution/(2*camera['range_m']))))
    step=grid.resolution/2
    angles=[yaw-half+2*half*i/rays for i in range(rays+1)]
    j=np.arange(math.floor(camera['range_m']/step)+1)
    # Preserve scalar operation order and math trig; only lookup/early-stop is batched.
    xs=x+np.array([math.cos(a) for a in angles])[:,None]*j*step
    ys=y+np.array([math.sin(a) for a in angles])[:,None]*j*step
    wx,wy=np.floor(xs/grid.resolution).astype(np.int64),np.floor(ys/grid.resolution).astype(np.int64)
    cols,rows=wx-grid.origin[0],wy-grid.origin[1]
    n,m=grid.raw.shape
    inside=(rows>=0)&(rows<n)&(cols>=0)&(cols<m)
    free=inside&(grid.raw[np.clip(rows,0,n-1),np.clip(cols,0,m-1)]==0)
    reached=np.logical_and.accumulate(free,axis=1)
    return frozenset(map(tuple,np.unique(np.column_stack((wx[reached],wy[reached])),axis=0).tolist()))
