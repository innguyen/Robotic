"""Estimated camera coverage from an observed costmap only; never GT raycast."""
import math


def camera_pose(pose,camera):
    x,y,yaw=pose;c,s=math.cos(yaw),math.sin(yaw)
    return (x+c*camera['x_m']-s*camera['y_m'],y+s*camera['x_m']+c*camera['y_m'],yaw+camera['yaw_rad'])


def coverage_cells(grid,pose,camera):
    x,y,yaw=camera_pose(pose,camera)
    half=math.radians(camera['fov_deg'])/2
    rays=max(2,math.ceil(2*half/max(.005,grid.resolution/(2*camera['range_m']))))
    cells=set(); step=grid.resolution/2
    for i in range(rays+1):
        angle=yaw-half+2*half*i/rays
        for j in range(math.floor(camera['range_m']/step)+1):
            px,py=x+math.cos(angle)*j*step,y+math.sin(angle)*j*step
            if not grid.inside((px,py)):break
            row,col=grid.cell((px,py))
            if grid.raw[row,col]!=0:break
            cells.add((math.floor(px/grid.resolution),math.floor(py/grid.resolution)))
    return frozenset(cells)
