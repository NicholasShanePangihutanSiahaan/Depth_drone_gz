import math
from pathlib import Path
import numpy as np
import pytest
from polinasi_nav.config import load_config
from polinasi_nav.mapping import VoxelMap, FREE
from polinasi_nav.continuous_tree import ContinuousTreeInspectionController


def controller():
    c=load_config(Path(__file__).resolve().parents[1]/'config/tree_inspection.json')
    c.update(simulation_continuous_roll=True,continuous_tree_scanning=True,
             camera_mount_yaw=math.pi/2,tree_view_count=6,tree_scan_anchors=24,
             bounds_min=[-6.,-6.,0.],bounds_max=[6.,6.,6.])
    grid=VoxelMap(c)
    grid.state[:]=FREE;grid.seen[:]=0.;grid.rebuild(0.)
    ctl=ContinuousTreeInspectionController(c,grid,asynchronous=False)
    ctl.active_tree=dict(id=7,xy=[0.,0.])
    ctl.state='SURVEY';ctl.tree_phase='SCAN'
    ctl._begin_continuous_scan(np.array([2.,0.,2.]))
    return ctl


def test_circle_is_constant_height_ccw_and_left_camera_faces_tree():
    ctl=controller()
    np.testing.assert_allclose(np.linalg.norm(ctl.waypoints[:,:2],axis=1),2.)
    np.testing.assert_allclose(ctl.waypoints[:,2],2.)
    assert ctl.waypoints[0,1]>0
    assert ctl._camera_heading([2.,0.,2.])==pytest.approx(math.pi/2)
    np.testing.assert_allclose(ctl.waypoints[-1],[2.,0.,2.],atol=1e-10)


def test_fixed_scan_unknown_requests_stationary_depth_then_restores_tree_alignment():
    ctl=controller()
    ctl.c.update(fixed_scan_depth_observation=True,lidar_fixed=True,camera_mount_yaw=0.)
    ctl.route_blockage={'unknown_count':3}
    ctl.scan_start_aligned=True
    position=np.array([2.,0.,2.])
    ctl._camera_observation_ready=lambda now,p:False
    assert not ctl._prepare_scan(1.,position)
    assert ctl.navigation_camera_active and ctl.camera_navigation_view()
    ctl._camera_observation_ready=lambda now,p:True
    assert not ctl._prepare_scan(2.,position)
    assert not ctl.navigation_camera_active and not ctl.camera_navigation_view()
    assert not ctl.scan_start_aligned
    assert ctl.navigation_camera_completed[0]==ctl.waypoint_index
    np.testing.assert_allclose(ctl.navigation_camera_completed[1],position)


def test_scan_does_not_promote_ascent_overshoot_to_new_altitude():
    ctl=controller();ctl.inspection_altitude=3.32
    ctl.command=np.array([2.,0.,3.32])
    ctl._begin_continuous_scan(np.array([2.,0.,3.44]))
    np.testing.assert_allclose(ctl.waypoints[:,2],3.32)
    assert ctl.scan_handoff['measured_start_height_m']==pytest.approx(3.44)
    assert ctl.scan_handoff['reference_height_m']==pytest.approx(3.32)
    ctl.mount_feedback=dict(feedback_fresh=True,measured={'camera_pitch':.5},targets={'camera_pitch':.5})
    ctl.body_yaw_measured=ctl._camera_heading(ctl.command)
    ctl.vehicle_speed=.02;ctl.mount_phase_started=1.
    assert not ctl._prepare_scan(2.,np.array([2.,0.,3.44]))
    assert ctl.scan_settled_since is None and not ctl.scan_start_aligned
    assert not ctl._prepare_scan(2.1,ctl.command)
    assert ctl.scan_settled_since==pytest.approx(2.1)


def test_pass_requires_new_map_but_does_not_stop_at_sector():
    ctl=controller();ctl.waypoint_index=3;ctl.batch_end_index=23
    ctl.mount_feedback=dict(feedback_fresh=True,measured={'camera_pitch':.5},targets={'camera_pitch':.5})
    p=ctl.waypoints[3];ctl.body_yaw_measured=ctl._camera_heading(p)
    ctl.command_v=np.array([.1,.2,0.])
    ctl.grid.last_observation_stamp=1.
    ctl._on_motion(1.,p)
    assert ctl.waypoint_index==3
    ctl.grid.last_observation_stamp=1.1
    ctl._on_motion(1.1,p)
    assert ctl.waypoint_index==4 and ctl.wait_for_scan is None
    assert np.linalg.norm(ctl.command_v)>0
    assert ctl.scan_sector_records[0]['inspection_valid']


def test_bad_camera_cannot_claim_sector_success():
    ctl=controller();ctl.mount_feedback=dict(feedback_fresh=False)
    ctl._record_pass(3,1.,ctl.waypoints[3])
    assert not ctl.scan_records and not ctl.scan_sector_records[0]['inspection_valid']


def test_completion_resets_for_next_tree_and_requires_all_sectors():
    ctl=controller();ctl.scan_start_aligned=True
    ctl.mount_feedback=dict(feedback_fresh=True,measured={'camera_pitch':.5},targets={'camera_pitch':.5})
    for i in (3,7,11,15,19):
        ctl.body_yaw_measured=ctl._camera_heading(ctl.waypoints[i])
        ctl._record_pass(i,1.,ctl.waypoints[i])
    ctl.waypoint_index=23;ctl.wait_for_scan=1.;ctl.grid.last_observation_stamp=2.
    ctl.body_yaw_measured=ctl._camera_heading(ctl.waypoints[23])
    ctl._prepare_trajectory(2.,ctl.waypoints[23])
    assert ctl.completed_trees==[7] and ctl.tree_phase=='SEARCH'
    assert ctl.active_tree is None and not ctl.scan_orbit_ready


def test_sensor_localisation_is_not_mislabeled_as_validated():
    ctl=controller()
    with pytest.raises(ValueError):
        ContinuousTreeInspectionController(ctl.c,ctl.grid,sensor_mode=True)


def test_rolling_approach_enters_ascent_not_scan_and_never_tilts_lidar():
    ctl=controller();ctl.c['rolling_canopy_ascent']=True;ctl.c['tree_continuous_ascent']=True
    ctl.tree_phase='APPROACH';ctl.view_target=ctl.command.copy()
    ctl.wait_for_scan=1.;ctl.grid.last_observation_stamp=2.
    ctl._prepare_trajectory(2.,ctl.command)
    assert ctl.tree_phase=='ASCEND' and not ctl.scan_start_aligned
    assert ctl.lidar_target()==0.
    ctl._mount_phase('NORMALIZE',3.)
    assert ctl.tree_phase=='SCAN' and ctl.inspection_altitude==ctl.command[2]


def test_rolling_ascent_uses_bounded_live_reference_and_keeps_xy_fixed():
    ctl=controller();ctl.c['tree_continuous_ascent']=True;ctl.c['rolling_canopy_ascent']=True
    ctl.active_tree['observed_stem_top']=4.
    ctl.tree_phase='ASCEND';ctl.command=np.array([2.,0.,2.])
    for i in range(40):
        now=1.+i*.05;ctl.grid.last_observation_stamp=now
        result=ctl._live_reference(now,.05,ctl.command,ctl.command_v)
        assert result is not None
        ctl.command,ctl.command_v,ctl.command_a,ctl.elapsed=result
        np.testing.assert_allclose(ctl.command[:2],[2.,0.])
        assert not ctl.failure and ctl.tree_phase=='ASCEND'
    assert ctl.command[2]>2.1


def blocked_scan_controller():
    from polinasi_nav.mapping import OCCUPIED
    ctl=controller()
    ctl.c.update(scan_target_repair=True,scan_target_max_outward=1.,scan_target_max_descent=.6)
    ctl._begin_continuous_scan(np.array([2.,0.,3.2]))
    ctl.waypoint_index=15
    target=ctl.waypoints[15].copy()
    leaf=target+np.array([0.,0.,.7])
    ctl.grid.state[tuple(ctl.grid.indices(leaf))]=OCCUPIED
    ctl.grid.rebuild(0.)
    ctl.command=ctl.waypoints[12].copy()
    assert ctl.grid.occupied_goal(target)
    return ctl,target


def test_occupied_scan_anchor_is_resampled_without_skipping_or_cutting_radius():
    ctl,target=blocked_scan_controller()
    assert ctl._choose_goal(ctl.command)
    assert ctl.grid.safe(ctl.goal) and ctl.waypoint_index==15
    assert np.linalg.norm(ctl.goal[:2])>=np.linalg.norm(target[:2])-1e-8
    np.testing.assert_allclose(ctl.goal[:2]/np.linalg.norm(ctl.goal[:2]),target[:2]/2.)
    assert ctl.goal[2]<=target[2]
    assert not ctl.scan_records and not ctl.completed_trees
    assert not ctl.scan_target_diagnostics['sector_skipped']


def test_lower_scan_repair_never_reclimbs_remaining_ring_or_exceeds_total_descent():
    ctl,target=blocked_scan_controller()
    ctl.c['scan_target_max_outward']=0.
    assert ctl._choose_goal(ctl.command)
    assert target[2]-.6-1e-8<=ctl.goal[2]<target[2]
    np.testing.assert_allclose(ctl.waypoints[15:,2],ctl.goal[2])
    np.testing.assert_allclose(ctl.waypoints[:15,2],target[2])
    np.testing.assert_allclose(ctl.scan_nominal_waypoints[:,2],target[2])


def test_unknown_alternatives_never_repair_a_blocked_scan_anchor():
    from polinasi_nav.mapping import UNKNOWN, OCCUPIED
    ctl,target=blocked_scan_controller()
    ctl.grid.state[:]=UNKNOWN
    ctl.grid.state[tuple(ctl.grid.indices(target))]=OCCUPIED
    ctl.grid.rebuild(0.)
    assert not ctl._choose_goal(ctl.command)
    assert not ctl.scan_target_repairs
    assert ctl.scan_target_diagnostics['result']=='no_observed_safe_candidate'
    assert ctl.waypoint_index==15


def test_async_scan_repair_waypoints_adopted_only_with_safe_plan():
    from concurrent.futures import Future
    from polinasi_nav.survey import prepare_survey_snapshot
    ctl,target=blocked_scan_controller()
    clone=ctl._planning_clone()
    clone,ready=prepare_survey_snapshot(clone,0.,ctl.command,None,None)
    assert ready and not clone.failure
    assert clone.trajectory.safe(ctl.grid)
    assert not np.allclose(clone.waypoints[15],target)
    np.testing.assert_allclose(ctl.waypoints[15],target)
    ctl.pending_plan=Future();ctl.pending_plan.set_result((clone,ready))
    ctl.scan_start_aligned=True
    # Select the async adoption path with a sentinel, no actual worker needed.
    ctl.executor=object()
    assert ctl._prepare_trajectory(0.,ctl.command)
    np.testing.assert_allclose(ctl.waypoints,clone.waypoints)
    assert ctl.scan_target_repairs
    rejected=ctl._planning_clone();rejected.waypoints[:]=0.
    accepted=ctl.waypoints.copy()
    ctl._accept_plan_auxiliary(rejected,False)
    np.testing.assert_allclose(ctl.waypoints,accepted)


def enable_feedback(ctl):
    ctl.c.update(feedback_speed_envelope=True,velocity_uncertainty_mps=.03,
                 pid_correction_acceleration=.15,scan_target_repair=True,
                 scan_target_max_outward=0.,scan_target_max_descent=.6)


def test_statically_free_anchor_with_no_arrival_reserve_is_lowered_without_skipping():
    from polinasi_nav.mapping import OCCUPIED
    ctl=controller();enable_feedback(ctl)
    ctl._begin_continuous_scan(np.array([2.,0.,3.2]));ctl.waypoint_index=3
    roof=ctl.grid.indices([0.,0.,4.2])[2]
    ctl.grid.state[:,:,roof]=OCCUPIED;ctl.grid.rebuild(0.)
    xy=ctl.waypoints[3,:2].copy()
    # Find the measured-free side of the voxel boundary where arrival
    # uncertainty crosses into the inflated roof. No artificial blocked mask.
    candidates=[np.r_[xy,z] for z in np.arange(2.6,3.8,.005)]
    target=next(p for p in candidates if ctl.grid.safe(p) and
                not ctl._scan_endpoint_safe(p,p-[0.,0.,1.])[0])
    ctl.waypoints[3]=target;ctl.scan_nominal_waypoints[3]=target
    ctl.command=target-[0.,0.,.5]
    assert not ctl.grid.occupied_goal(target)
    assert ctl._choose_goal(ctl.command)
    assert ctl.goal[2]<target[2]
    np.testing.assert_allclose(ctl.goal[:2],target[:2])
    assert ctl._scan_endpoint_safe(ctl.goal,ctl.command)[0]
    assert ctl.waypoint_index==3 and not ctl.scan_records
    assert ctl.scan_target_diagnostics['rejections']['terminal_reserve']>0


def test_scan_batch_stops_before_unstoppable_endpoint_and_keeps_next_anchor():
    ctl=controller();enable_feedback(ctl)
    g=ctl.grid
    centers=[g.centers([i,g.indices([0.,0.,2.])[1],g.indices([0.,0.,2.])[2]])
             for i in range(g.shape[0]-1,0,-1)]
    center=next(p for p in centers if g.safe(p))
    end=center+np.array([g.res*.49,0.,0.]);end[1:]=[0.,2.]
    assert g.safe(end)
    ctl.command=end-[1.,0.,0.]
    ctl.waypoints=np.array([end-[.5,0.,0.],end]);ctl.waypoint_index=0
    ctl.goal=ctl.waypoints[0].copy()
    points=ctl._plan_route(ctl.command,ctl.goal)
    assert points is not None and ctl.batch_end_index==0
    assert ctl.waypoint_index==0 and len(ctl.waypoints)==2
    np.testing.assert_allclose(ctl.goal,ctl.waypoints[0])
    assert ctl.scan_endpoint_rejections[0]['anchor']==1
    assert not ctl.scan_records and not ctl.completed_trees


def test_detour_arrival_direction_repairs_same_anchor_before_curve_construction():
    from polinasi_nav.mapping import OCCUPIED
    from polinasi_nav.canopy_scan import canopy_guard_grid
    from polinasi_nav.planning import plan
    from polinasi_nav.local_braking_retime import route_terminal_reserve_check
    ctl=controller();enable_feedback(ctl)
    ctl.c.update(bounds_min=[-12.,-4.4,-.6],bounds_max=[-5.,4.,6.],
                 drone_dimensions=[.6,.6,.7],clearance=.2,tracking_margin=.2,
                 takeoff_altitude=2.,scan_adaptive_height=True,
                 tree_ascent_observation_guard=.2,scan_target_max_outward=1.)
    ctl.grid=VoxelMap(ctl.c);g=ctl.grid
    g.state[:]=FREE;g.seen[:]=0.
    g.state[tuple(g.indices([-9.1,.3,2.5]))]=OCCUPIED;g.rebuild(0.)
    ctl.active_tree=dict(id=1,xy=[-8.5283911758325,-.3374676624235])
    ctl.command=np.array([-9.1466839259891,1.4491005542121,2.23350992018065])
    target=np.array([-9.6142586262347,1.2240617645784,2.4670198403613])
    ctl.waypoints=np.array([target]);ctl.scan_nominal_waypoints=np.array([target])
    ctl.waypoint_index=0;ctl.goal=target.copy()
    guarded=canopy_guard_grid(g,.2)
    assert guarded.safe(target)
    assert ctl._scan_endpoint_safe(target,ctl.command)[0]
    old_route=plan(guarded,ctl.command,target)
    assert old_route is not None
    assert not route_terminal_reserve_check(old_route,guarded,ctl.c)[0]
    route=ctl._plan_route(ctl.command,ctl.goal)
    assert route is not None and route_terminal_reserve_check(route,guarded,ctl.c)[0]
    assert np.linalg.norm(ctl.goal-target)>.1
    before=target[:2]-ctl.active_tree['xy'];after=ctl.goal[:2]-ctl.active_tree['xy']
    np.testing.assert_allclose(before/np.linalg.norm(before),after/np.linalg.norm(after))
    assert np.linalg.norm(after)>=np.linalg.norm(before)
    assert ctl.waypoint_index==0 and not ctl.scan_records
    assert ctl.scan_target_diagnostics['rejections']['route_terminal_reserve']>0


@pytest.mark.parametrize('phase',['APPROACH','RETURN'])
def test_transit_buffer_rechecks_new_overhead_voxel_before_body_corridor_is_blocked(phase):
    from polinasi_nav.mapping import OCCUPIED
    from polinasi_nav.canopy_scan import CanopyScanTrajectory,canopy_guard_grid
    from polinasi_nav.continuous import ContinuousTrajectory
    ctl=controller();enable_feedback(ctl)
    ctl.c['transit_observation_guard']=.2
    ctl.tree_phase='APPROACH' if phase=='APPROACH' else 'SEARCH'
    ctl.state='SURVEY' if phase=='APPROACH' else 'RETURN'
    points=np.array([[1.,0.,2.85],[2.,0.,2.85]])
    # Select adjacent observed roof levels at the actual voxel resolution.
    base=ContinuousTrajectory(points,ctl.c)
    selected=None
    for roof in range(ctl.grid.indices([0.,0.,3.])[2],ctl.grid.shape[2]-1):
        ctl.grid.state[:]=FREE;ctl.grid.state[:,:,roof]=OCCUPIED;ctl.grid.rebuild(0.)
        if not base.safe(ctl.grid) or canopy_guard_grid(ctl.grid,.2).safe(points[-1]):
            continue
        ctl.grid.state[:]=FREE;ctl.grid.state[:,:,roof+1]=OCCUPIED;ctl.grid.rebuild(0.)
        if canopy_guard_grid(ctl.grid,.2).safe(points[-1]):
            selected=roof;break
    assert selected is not None
    trajectory=ctl._trajectory_from_points(points)
    assert isinstance(trajectory,CanopyScanTrajectory) and trajectory.safe(ctl.grid)
    # Retain the old roof; a new observation extends its lower surface.
    ctl.grid.state[:,:,selected]=OCCUPIED;ctl.grid.rebuild(.1)
    assert ContinuousTrajectory(points,ctl.c).safe(ctl.grid)
    assert not trajectory.safe_from(ctl.grid,trajectory.duration/2)
    assert ctl.c['tracking_margin']==.2 and ctl.c['clearance']==.2
