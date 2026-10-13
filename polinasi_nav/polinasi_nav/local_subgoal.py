"""Bounded observed-free intermediate goals; final inspection goal is unchanged."""
import math
import numpy as np


def geometric_hint(current,target,step=2.):
    current,target = np.asarray(current,float),np.asarray(target,float)
    delta = target-current
    return current+delta*min(1.,step/max(np.linalg.norm(delta),1e-8))


def inspection_hint(grid, current, target, allow_vertical=False, step=2.):
    current,target = np.asarray(current,float),np.asarray(target,float)
    delta = target-current
    nominal = geometric_hint(current,target,step)
    detail = dict(policy='nominal',nominal=nominal.tolist(),final_goal=target.tolist())
    # Unknown needs observation, not arbitrary relabelling/projection.
    if not grid.occupied_goal(nominal):
        return nominal,detail
    angle = math.atan2(delta[1],delta[0])
    heights = [nominal[2]]
    if allow_vertical:
        floor = min(current[2],target[2])
        heights += [current[2],floor,max(floor,current[2]-.4),max(floor,current[2]-.8)]
    else:
        heights = [current[2]]
    options = []
    observation_options = []
    rejected = dict(no_progress=0,occupied_endpoint=0,unknown_corridor=0,occupied_corridor=0)
    for radius in (step,step*.75,step*.5):
        for degrees in (0,15,-15,30,-30,45,-45,60,-60,90,-90,120,-120):
            a = angle+math.radians(degrees)
            for height in sorted(set(heights)):
                p = current+np.array([radius*math.cos(a),radius*math.sin(a),height-current[2]])
                move = p-current
                p = current+move*min(1.,step/max(np.linalg.norm(move),1e-8))
                progress = np.linalg.norm(delta)-np.linalg.norm(target-p)
                if progress<grid.res/2:
                    rejected['no_progress'] += 1
                    continue
                if grid.occupied_goal(p):
                    rejected['occupied_endpoint'] += 1
                    continue
                score = np.linalg.norm(target-p)+.15*np.linalg.norm(p-current)
                if grid.safe(p) and grid.line_safe(current,p):
                    options.append((score,p))
                    continue
                blockers = grid.corridor_blockers(current,p)
                if len(blockers['occupied']) or blockers['outside']:
                    rejected['occupied_corridor'] += 1
                    continue
                if len(blockers['unknown']):
                    rejected['unknown_corridor'] += 1
                    observation_options.append((score,p,blockers['unknown']))
    detail.update(policy='occupied_intermediate_no_observed_detour',candidate_count=len(options),
        observation_candidate_count=len(observation_options),candidate_rejections=rejected)
    if options:
        selected = min(options,key=lambda item:item[0])[1]
        detail.update(policy='observed_free_intermediate_detour',selected=selected.tolist(),
            original_blockage=grid.describe_blockage(grid.indices(nominal)))
        return selected,detail
    if observation_options:
        _,selected,unknown = min(observation_options,key=lambda item:item[0])
        detail.update(policy='unknown_alternative_requires_observation',selected=selected.tolist(),
            unknown_count=len(unknown),unknown_samples=grid.centers(unknown[:16]).tolist(),
            original_blockage=grid.describe_blockage(grid.indices(nominal)))
        # A hint requests observations; it is NOT a motion/setpoint authorization.
        # The shared planner and exploration gates still reject unknown space.
        return selected,detail
    return nominal,detail
