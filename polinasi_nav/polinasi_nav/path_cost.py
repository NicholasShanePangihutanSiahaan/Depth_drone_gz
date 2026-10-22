"""Soft clearance preference outside the unchanged inflated collision mask.

Cost quadrature ranks paths only. Exact supercover/curve/stop checks still
authorize flight. This is a 3D engineering cost, not a Nav2/SUPER port.
"""
import numpy as np
from scipy.ndimage import distance_transform_edt


def clearance_cost_field(grid):
    preference=float(grid.c.get('planner_clearance_preference',0.))
    weight=float(grid.c.get('planner_clearance_cost_weight',0.))
    if preference<=0. or weight<=0.:return None
    if not np.isfinite(preference+weight):raise ValueError('invalid clearance preference')
    distance=distance_transform_edt(~grid.blocked)*grid.res
    return 1.+weight*np.maximum(0.,1.-distance/preference)**2


def line_travel_cost(grid,start,end,field):
    start,end=np.asarray(start),np.asarray(end)
    distance=float(np.linalg.norm(end-start))
    if field is None or distance<1e-12:return distance
    count=max(2,int(np.ceil(distance/(grid.res/2)))+1)
    ids=grid.indices(np.linspace(start,end,count))
    if not np.all(grid.inside(ids)):return float('inf')
    values=field[tuple(ids.T)]
    return float(distance*np.trapz(values,dx=1./(count-1)))
