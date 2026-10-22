"""Bounded entry resampling for one measured tree; never a flight certificate."""
import math
import numpy as np
from scipy.ndimage import label
from .transit_guard import transit_guard_grid


def approach_grid(grid,config):
    return transit_guard_grid(grid,config.get('transit_observation_guard',0.),
                              config.get('transit_lateral_observation_guard',0.))


def connected_endpoint(grid,point):
    return grid.safe(point) and grid.line_safe(point,grid.centers(grid.indices(point)))


def select_approach_target(grid,current,tree,angle,config,excluded=()):
    view=approach_grid(grid,config)
    current=np.asarray(current,float)
    detail=dict(policy='bounded_same_tree_entry_resampling_not_flight_certificate',
        tree_id=tree['id'],map_version=int(grid.version),
        observation_stamp=float(grid.last_observation_stamp),
        rejected=dict(blocked_or_connector=0,disconnected=0,live_arrival_rejected=0,
                      route=0,terminal_reserve=0),candidate_count=0)
    if not connected_endpoint(view,current):
        return None,dict(detail,result='start_not_connected_safe')
    components,_=label(~view.blocked)
    component=components[tuple(view.indices(current))]
    top=float(config.get('tree_approach_altitude',config['takeoff_altitude']))
    floor=float(config.get('exploration_min_altitude',1.))
    # Include the original target height, then voxel centres BELOW the cap.
    # Snapping top upward must never silently increase its permitted height.
    levels=view.lo[2]+(np.arange(view.shape[2])+.5)*view.res
    heights=np.unique(np.r_[top,levels[(levels>=floor-1e-9)&(levels<top-1e-9)]])[::-1]
    offsets=np.r_[0.,np.array([sign*n*math.pi/12 for n in range(1,12) for sign in (1,-1)]),math.pi]
    candidates=[]
    for offset in offsets:
        direction=np.array([math.cos(angle+offset),math.sin(angle+offset)])
        for radius in config['tree_view_distances']:
            xy=np.asarray(tree['xy'])+radius*direction
            for height in heights:
                p=np.r_[xy,height]
                if any(np.linalg.norm(p-np.asarray(q))<1e-6 for q in excluded):
                    detail['rejected']['live_arrival_rejected']+=1;continue
                if not connected_endpoint(view,p):
                    detail['rejected']['blocked_or_connector']+=1;continue
                if components[tuple(view.indices(p))]!=component:
                    detail['rejected']['disconnected']+=1;continue
                score=float(np.linalg.norm(p-current)+2*(top-height)+.2*abs(offset))
                candidates.append((score,p,float(offset)))
    detail['candidate_count']=len(candidates)
    if not candidates:return None,dict(detail,result='no_safe_connected_entry')
    from .planning import plan
    from .local_braking_retime import route_terminal_reserve_check
    # Check the ACTUAL final A* edge, not a guessed radial or XY direction.
    # Full smoothed-curve/braking certification is still required downstream.
    limit=int(config.get('approach_route_candidate_limit',12))
    for trial,(_,selected,offset) in enumerate(sorted(candidates,key=lambda row:row[0])[:limit],1):
        route=plan(view,current,selected,config.get('planner_max_expansions',50000))
        if route is None:
            detail['rejected']['route']+=1;continue
        safe,check=route_terminal_reserve_check(route,view,config)
        if not safe:
            detail['rejected']['terminal_reserve']+=1
            detail['last_terminal_rejection']=check;continue
        return selected,dict(detail,result='proposed_requires_full_route_certificate',
            selected=selected.tolist(),angle_offset_rad=offset,floor_m=floor,cap_m=top,
            route_candidates_evaluated=trial,actual_arrival_terminal_reserve=True,
            radius_policy='configured_tree_view_distances_unchanged')
    return None,dict(detail,result='entry_search_budget' if len(candidates)>limit else 'no_terminal_safe_entry',
                     route_candidates_evaluated=min(limit,len(candidates)))
