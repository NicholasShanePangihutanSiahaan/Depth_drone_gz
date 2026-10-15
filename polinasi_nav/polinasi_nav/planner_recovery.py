"""Bounded candidate recovery. Safety gates and freshness are never bypassed."""
import json
import time
from pathlib import Path
import numpy as np
from .planning import brake_safe


def save_failure_snapshot(controller,detail,points=None,grid=None):
    directory=controller.c.get('planner_audit_dir')
    if not directory:return
    directory=Path(directory)
    try:
        directory.mkdir(parents=True,exist_ok=True)
        # Bound disk/CPU cost per run. Keep exact first failures for replay.
        if len(list(directory.glob('*.npz')))>=8:return
        path=directory/f'failure_{time.monotonic_ns()}.npz'
        grid=controller.grid if grid is None else grid
        metadata=dict(config=controller.c,diagnostics=detail,origin=grid.lo.tolist(),
            shape=list(grid.shape),map_version=int(grid.version),
            last_observation_stamp=float(grid.last_observation_stamp),
            state=controller.state,phase=getattr(controller,'tree_phase',None),
            command=controller.command.tolist(),command_v=controller.command_v.tolist(),
            command_a=controller.command_a.tolist(),
            provenance='exact_detached_planning_grid_not_ground_truth')
        np.savez_compressed(path,metadata=json.dumps(metadata,default=lambda x:x.tolist() if isinstance(x,np.ndarray) else x.item()),state=grid.state,seen=grid.seen,
            assumed_free=grid.assumed_free,assumed_occupied=grid.assumed_occupied,
            blocked=grid.blocked,blocked_actual=grid.blocked_actual,
            points=np.asarray(points if points is not None else getattr(controller.trajectory,'points',[])),
            controls=np.asarray(getattr(controller.trajectory,'controls',[])),
            durations=np.asarray(getattr(controller.trajectory,'durations',[])))
        detail['snapshot_path']=str(path)
    except (OSError,TypeError,ValueError) as exc:
        detail['snapshot_error']=str(exc)


def reject_candidate(controller,now,detail):
    """At-rest hold/retry with the existing timeout; moving jobs stay rejected.

    Moving clones cannot replace their committed trajectory on ready=False.
    Live sensor health, actual body checks and final PID gates remain active.
    """
    controller.route_failure_diagnostics=detail
    if not controller.c.get('feedback_speed_envelope'):
        controller.trajectory=None
        controller.fail(detail['reason'])
        return False
    from .super_policy import backup_grid
    save_failure_snapshot(controller,detail,grid=backup_grid(controller))
    controller.trajectory=None
    controller.blocked_since=now if controller.blocked_since is None else controller.blocked_since
    controller.route_blockage=dict(reason='trajectory_candidate_rejected',
        candidate_reason=detail['reason'],retry_since=float(controller.blocked_since),
        deadline=float(controller.blocked_since+controller.c['blocked_timeout']))
    if not hasattr(controller,'moving_boundary'):
        check={}
        if not brake_safe(controller.grid,controller.command,controller.command_v,
                controller.c,controller.command_a,diagnostics=check):
            detail['hold_check']=check
            controller.fail('no_safe_hold_after_planner_rejection')
        elif now-controller.blocked_since>controller.c['blocked_timeout']:
            controller.fail('blocked_route')
    return False
