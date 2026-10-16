"""Measured-free, under-canopy height envelopes for receding scan chunks.

No direct setpoint override: the caller must certify the resulting 3D curve.
"""
import math
import copy
import numpy as np
from .continuous import ContinuousTrajectory
from .mapping import vertical_guard,planning_map_reserve


def canopy_guard_grid(grid,guard):
    """Planner-only view: reserve the complete upward interval, voxel-rounded.

    Raw occupancy, timestamps and actual-body mask are not modified.
    """
    view=copy.copy(grid)
    view.blocked=grid.blocked.copy()
    cells=max(0,int(math.ceil(guard/grid.res-1e-9)))
    view.canopy_guard_cells=getattr(grid,'canopy_guard_cells',0)+cells
    for offset in range(1,cells+1):
        view.blocked[:,:,:-offset]|=grid.blocked[:,:,offset:]
        view.blocked[:,:,-offset:]=True
    return view


class CanopyScanTrajectory(ContinuousTrajectory):
    """The full commanded 3D curve plus a reserved vertical canopy buffer."""
    def __init__(self,trajectory,guard):
        self.__dict__.update(trajectory.__dict__)
        self.canopy_guard=vertical_guard(dict(tree_ascent_observation_guard=guard,resolution=self.c['resolution']))

    def _shifted(self,offset):
        proxy=object.__new__(ContinuousTrajectory)
        proxy.__dict__.update(self.__dict__)
        shift=np.array([0.,0.,offset])
        proxy.controls=[controls+shift for controls in self.controls]
        proxy.points=self.points+shift
        proxy.certificate=None
        return proxy

    def _offsets(self,grid):
        count=max(1,int(math.ceil(self.canopy_guard/(grid.res/2))))
        return np.linspace(0.,self.canopy_guard,count+1)[1:]

    def safe(self,grid):
        nominal=super().safe(grid)
        self.canopy_certificate=dict(nominal_safe=bool(nominal),guard_m=self.canopy_guard)
        if not nominal:return False
        for offset in self._offsets(grid):
            proxy=self._shifted(offset)
            if not proxy.safe_from(grid,0.):
                self.canopy_certificate.update(guard_safe=False,failed_offset_m=float(offset),
                    failure=proxy.execution_certificate)
                return False
        self.canopy_certificate['guard_safe']=True
        return True

    def safe_from(self,grid,elapsed):
        safe=super().safe_from(grid,elapsed)
        if safe:
            for offset in self._offsets(grid):
                proxy=self._shifted(offset)
                if not proxy.safe_from(grid,elapsed):
                    self.execution_certificate.update(remaining_curve_safe=False,
                        canopy_guard_m=self.canopy_guard,guard_failure=proxy.execution_certificate)
                    return False
        return safe


def stem_target_height(tree,config):
    return float(tree['observed_stem_top'])-config.get('scan_stem_top_offset',0.)


def canopy_profile(grid,waypoints,index,current,tree,config,initial_height):
    result=np.asarray(waypoints).copy()
    stop=min(len(result),index+config.get('survey_batch_points',6))
    guard=config.get('tree_ascent_observation_guard',.2)
    reserve=planning_map_reserve(config)
    guarded_grid=canopy_guard_grid(grid,guard)
    camera_z=config.get('camera_mount',[0.,0.,0.])[2]
    target=min(5.5,stem_target_height(tree,config)-camera_z)
    floor=max(config['takeoff_altitude'],initial_height-config.get('scan_target_max_descent',.6))
    resolution=min(.1,grid.res/2)
    entries=[]
    previous=np.asarray(current).copy()
    for i in range(index,stop):
        xy=result[i,:2]
        low=np.r_[xy,floor]
        heights=np.arange(floor,max(floor,target+reserve)+1e-8,resolution)
        connected=[]
        for height in heights:
            p=np.r_[xy,height]
            # A contiguous observed-free column from below cannot jump through
            # leaves into free air ABOVE the canopy. Unknown also blocks it.
            if not guarded_grid.line_safe(low,p):
                break
            connected.append(float(height))
        ceiling=max(connected) if connected else None
        if ceiling is not None:
            distance=float(np.linalg.norm(xy-previous[:2]))
            preferred=min(target,max(floor,ceiling-reserve))
            selected=min(preferred,previous[2]+config.get('scan_height_max_slope',.45)*distance)
            if abs(selected-previous[2])<config.get('scan_height_hysteresis',.08) and preferred>=previous[2]:
                selected=float(previous[2])
            # A cap at the endpoint alone misses a lower leaf BETWEEN anchors.
            # Prefer a slightly lower straight chord over a tiny A* detour to
            # a saturated ceiling endpoint. Real lateral blocks still use A*.
            direct=np.arange(selected,floor-1e-8,-resolution)
            selected=next((float(z) for z in direct
                if guarded_grid.line_safe(previous,np.r_[xy,z])),selected)
            result[i,2]=selected
        entries.append(dict(anchor=int(i),observed_height_cap_m=ceiling,
            selected_height_m=float(result[i,2]),column_connected=bool(connected)))
        previous=result[i].copy()
    # Start lowering before a lower roof, rather than a late vertical corner.
    for i in range(stop-2,index-1,-1):
        distance=np.linalg.norm(result[i+1,:2]-result[i,:2])
        result[i,2]=min(result[i,2],result[i+1,2]+config.get('scan_height_max_slope',.45)*distance)
    for entry in entries:entry['selected_height_m']=float(result[entry['anchor'],2])
    return result,dict(policy='receding_observed_under_canopy_profile',
        target_kind='estimated_stem_top_not_detected_flower',
        stem_target_z_m=stem_target_height(tree,config),camera_mount_z_m=float(camera_z),
        desired_vehicle_z_m=float(target),floor_m=float(floor),guard_m=float(guard),
        map_update_planning_reserve_m=reserve,
        map_version=int(grid.version),map_stamp=float(grid.last_observation_stamp)
        if math.isfinite(grid.last_observation_stamp) else None,
        planning_at_rest=True,online_height_setpoint_override=False,anchors=entries)


from .moving_trajectory import BoundaryTrajectory


class BoundaryCanopyTrajectory(CanopyScanTrajectory,BoundaryTrajectory):
    """Retain initial PVA while retiming a canopy-guarded moving join."""
