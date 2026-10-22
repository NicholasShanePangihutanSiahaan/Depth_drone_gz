"""Planning reserve for new occupied evidence adjacent to a transit corridor.

This bounded voxel reserve is not a guarantee against arbitrarily late returns.
Measured occupancy, unknown classification, actual-body masks and ages stay intact.
"""
import math
from scipy.ndimage import maximum_filter
from .canopy_scan import canopy_guard_grid, CanopyScanTrajectory
from .continuous import ContinuousTrajectory
from .moving_trajectory import BoundaryTrajectory


def transit_guard_grid(grid, vertical, lateral):
    upward=max(0,int(math.ceil(vertical/grid.res-1e-9)))
    horizontal=max(0,int(math.ceil(lateral/grid.res-1e-9)))
    extra_up=max(0,upward-getattr(grid,'canopy_guard_cells',0))
    extra_xy=max(0,horizontal-getattr(grid,'transit_lateral_guard_cells',0))
    if not extra_up and not extra_xy:return grid
    view=canopy_guard_grid(grid,extra_up*grid.res)
    if extra_xy:
        view.blocked=maximum_filter(view.blocked,size=(2*extra_xy+1,2*extra_xy+1,1),
                                    mode='constant',cval=True)
    view.transit_lateral_guard_cells=getattr(grid,'transit_lateral_guard_cells',0)+extra_xy
    view.c=dict(grid.c,
        planner_clearance_preference=grid.c.get('transit_clearance_preference',0.),
        planner_clearance_cost_weight=grid.c.get('transit_clearance_cost_weight',0.))
    return view


class TransitObservationTrajectory(CanopyScanTrajectory):
    def __init__(self,trajectory,vertical,lateral):
        super().__init__(trajectory,vertical)
        self.lateral_guard=float(lateral)

    def _view(self,grid):
        return transit_guard_grid(grid,self.canopy_guard,self.lateral_guard)

    def safe(self,grid):
        safe=ContinuousTrajectory.safe(self,self._view(grid))
        self.canopy_certificate=dict(guard_m=self.canopy_guard,
            lateral_guard_m=self.lateral_guard,guard_safe=bool(safe))
        return safe

    def safe_from(self,grid,elapsed):
        # Base suffix validation calls self.safe again. The marked view prevents
        # applying the reserve twice, including through retiming/cached geometry.
        safe=ContinuousTrajectory.safe_from(self,self._view(grid),elapsed)
        self.execution_certificate.update(canopy_guard_m=self.canopy_guard,
            lateral_guard_m=self.lateral_guard)
        return safe


class BoundaryTransitTrajectory(TransitObservationTrajectory,BoundaryTrajectory):
    """Keep moving-join PVA and boundary-specific retiming."""
