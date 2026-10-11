"""Bounded observed-free intermediate goals; final inspection goal is unchanged."""
import math
import numpy as np


def inspection_hint(grid, current, target, allow_vertical=False, step=2.):
    current,target = np.asarray(current,float),np.asarray(target,float)
    delta = target-current
    nominal = current+delta*min(1.,step/max(np.linalg.norm(delta),1e-8))
    detail = dict(policy='nominal',nominal=nominal.tolist(),final_goal=target.tolist())
    # Unknown needs observation, not arbitrary relabelling/projection.
    if not grid.occupied_goal(nominal):
        return nominal,detail
    angle = math.atan2(delta[1],delta[0])
    heights = [nominal[2]]
    if allow_vertical:
        heights += [current[2],min(current[2],target[2])]
    else:
        heights = [current[2]]
    options = []
    for radius in (step,step*.75,step*.5):
        for degrees in (0,15,-15,30,-30,45,-45,60,-60,90,-90,120,-120):
            a = angle+math.radians(degrees)
            for height in sorted(set(heights)):
                p = current+np.array([radius*math.cos(a),radius*math.sin(a),height-current[2]])
                move = p-current
                p = current+move*min(1.,step/max(np.linalg.norm(move),1e-8))
                progress = np.linalg.norm(delta)-np.linalg.norm(target-p)
                if progress<grid.res/2 or not grid.safe(p) or not grid.line_safe(current,p):
                    continue
                score = np.linalg.norm(target-p)+.15*np.linalg.norm(p-current)
                options.append((score,p))
    detail.update(policy='occupied_intermediate_no_observed_detour',candidate_count=len(options))
    if options:
        selected = min(options,key=lambda item:item[0])[1]
        detail.update(policy='observed_free_intermediate_detour',selected=selected.tolist(),
            original_blockage=grid.describe_blockage(grid.indices(nominal)))
        return selected,detail
    return nominal,detail
