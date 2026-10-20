"""Bounded history A* guidance. Not a flight/setpoint certificate."""
import time
import numpy as np
from scipy.ndimage import label
from .mapping import VoxelMap
from .planning import plan


def destination_key(goal):
    return tuple(np.round(np.asarray(goal,float),4))


def plan_history_route(history,request,now):
    started=time.perf_counter()
    c=dict(history.c)
    start,goal=np.asarray(request['start'],float),np.asarray(request['goal'],float)
    if start.shape!=(3,) or goal.shape!=(3,) or not np.all(np.isfinite(np.r_[start,goal])):
        return dict(frame='map',path=[],reason='invalid_request')
    result=dict(frame='map',goal=goal.tolist(),start=start.tolist(),stamp=float(now),
        map_stamp=float(history.last_observation_stamp),request_id=request['request_id'],
        source='sparse_history_with_FOV_prior' if c.get('fixed_fov_assumptions') else 'measured_sparse_history',path=[],partial=False,
        final_goal_reached_by_route=False)
    res=history.res
    # Search the full start-to-destination box plus lateral detour margin.
    # Bounds and a hard voxel budget prevent a full-farm dense allocation.
    margin=np.array([c.get('history_route_margin',4.)]*2+[1.5])
    lo=history.lo+np.floor((np.minimum(start,goal)-margin-history.lo)/res)*res
    hi=history.lo+np.ceil((np.maximum(start,goal)+margin-history.lo)/res)*res
    lo=np.maximum(lo,history.lo);hi=np.minimum(hi,np.asarray(c['bounds_max']))
    shape=np.ceil((hi-lo)/res).astype(int)
    result.update(search_origin=lo.tolist(),search_shape=shape.tolist())
    if np.any(shape<=0) or int(np.prod(shape))>c.get('history_route_max_voxels',800000):
        return dict(result,reason='history_search_budget')
    c.update(bounds_min=lo.tolist(),bounds_max=hi.tolist())
    grid=VoxelMap(c)
    history.restore(grid,now)
    if c.get('fixed_fov_assumptions'):history.fov_priors(grid)
    grid.rebuild(now)
    guard=float(c.get('transit_observation_guard',0.))
    if guard>0:
        from .transit_guard import transit_guard_grid
        grid=transit_guard_grid(grid,guard,c.get('transit_lateral_observation_guard',0.))
    result['observation_guard_m']=guard
    result['lateral_observation_guard_m']=c.get('transit_lateral_observation_guard',0.)
    # Transfers stay under the canopy: never climb above the higher endpoint.
    levels=grid.centers(np.column_stack([np.zeros(grid.shape[2]),np.zeros(grid.shape[2]),np.arange(grid.shape[2])]))[:,2]
    grid.blocked[:,:,levels>max(start[2],goal[2])+res/2]=True
    if not grid.safe(start):
        return dict(result,reason='history_start_not_safe')
    components,_=label(~grid.blocked)
    component=components[tuple(grid.indices(start))]
    full=(grid.safe(goal) and components[tuple(grid.indices(goal))]==component)
    endpoint=goal
    if not full:
        ids=np.argwhere(components==component)
        positions=grid.centers(ids)
        endpoint=positions[np.argmin(np.linalg.norm(positions-goal,axis=1))]
        if np.linalg.norm(start-goal)-np.linalg.norm(endpoint-goal)<res/2:
            return dict(result,reason='no_known_progress',partial=True)
    diagnostics={}
    path=plan(grid,start,endpoint,c.get('history_route_max_expansions',30000),diagnostics)
    result.update(reason=diagnostics.get('reason'),partial=not bool(full),
        final_goal_reached_by_route=bool(full and path is not None),
        endpoint=np.asarray(endpoint).tolist(),planner=diagnostics,
        wall_ms=(time.perf_counter()-started)*1000.)
    if path is not None:result['path']=np.asarray(path).tolist()
    return result


def local_route_goal(grid,current,destination,route,now,config):
    """Select a prefix of the HISTORY PATH, not a line toward the final goal."""
    current,destination=np.asarray(current,float),np.asarray(destination,float)
    if np.linalg.norm(current-destination)<.12 and grid.safe(destination):
        return destination.copy(),dict(reason='at_final_goal')
    detail=dict(reason='waiting_history_route')
    if not route or destination_key(route.get('goal',[]))!=destination_key(destination):
        return None,detail
    if not 0<=now-route.get('stamp',-np.inf)<=config.get('history_route_max_age',8.):
        return None,dict(detail,reason='stale_history_route')
    points=np.asarray(route.get('path',[]),float)
    if len(points)<2:
        return None,dict(detail,reason=route.get('reason','no_history_path'))
    best=None
    for i,(p,q) in enumerate(zip(points[:-1],points[1:])):
        delta=q-p
        u=np.clip(np.dot(current-p,delta)/max(np.dot(delta,delta),1e-12),0.,1.)
        projection=p+u*delta
        candidate=(float(np.linalg.norm(current-projection)),i,projection)
        if best is None or candidate[0]<best[0]:best=candidate
    if best[0]>.75:
        return None,dict(detail,reason='off_history_path',distance_m=best[0])
    prefix=np.vstack([best[2],points[best[1]+1:]])
    selected=None;travel=0.;eligible=[]
    def finish(detail):
        if not config.get('feedback_speed_envelope'):
            return selected,detail
        from .super_policy import feedback_brake_safe
        rejected=[]
        for endpoint,direction in reversed(eligible):
            check={}
            # Limit from the MOVING side near a stop, including unchanged
            # velocity/PID uncertainty; not a new commanded minimum speed.
            if feedback_brake_safe(grid,endpoint,direction*1e-6,np.zeros(3),config,check):
                return endpoint.copy(),dict(detail,selected=endpoint.tolist(),
                    terminal_reserve_rejections=rejected,
                    terminal_reserve_policy='near_zero_robust_stop_then_full_curve_certificate')
            rejected.append(dict(point=endpoint.tolist(),check=check))
        return None,dict(detail,selected=None,reason='observed_frontier_reached',
            terminal_reserve_rejections=rejected)

    limit=config.get('history_route_local_lookahead',2.)
    for p,q in zip(prefix[:-1],prefix[1:]):
        distance=float(np.linalg.norm(q-p))
        for length in np.linspace(0.,distance,max(2,int(np.ceil(distance/(grid.res/2)))+1))[1:]:
            if travel+length>limit+1e-8:break
            candidate=p+(q-p)*length/max(distance,1e-12)
            if not grid.safe(candidate):
                return finish(dict(reason='live_prefix_blocked',partial=route['partial'],
                    request_id=route['request_id'],selected=None if selected is None else selected.tolist()))
            if np.linalg.norm(candidate-current)>.1:
                selected=candidate.copy()
                eligible.append((selected,(q-p)/max(distance,1e-12)))
        travel+=distance
        if travel>=limit:break
    return finish(dict(reason='history_path_prefix' if selected is not None else 'observed_frontier_reached',
        partial=route['partial'],request_id=route['request_id'],
        final_destination=destination.tolist(),selected=None if selected is None else selected.tolist()))
