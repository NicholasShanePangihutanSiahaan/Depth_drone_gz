from pathlib import Path
import numpy as np
from polinasi_nav.config import load_config
from polinasi_nav.mapping import VoxelMap,FREE,OCCUPIED,UNKNOWN
from polinasi_nav.approach_target import select_approach_target,approach_grid,connected_endpoint
from polinasi_nav.planning import plan
from polinasi_nav.tree_mission import TreeInspectionController


def fixture():
    c=load_config(Path(__file__).resolve().parents[1]/'config/tree_inspection.json')
    c.update(bounds_min=[-6.,-6.,0.],bounds_max=[6.,6.,5.],
        drone_dimensions=[.2,.2,.2],clearance=.05,tracking_margin=.05,
        tree_view_distances=[2.],exploration_min_altitude=1.,
        transit_observation_guard=.2,transit_lateral_observation_guard=.2,
        fixed_approach_target_repair=True,defer_blocked_tree_entry=True,
        history_route_enabled=False)
    g=VoxelMap(c);g.state[:]=FREE;g.seen[:]=0.;g.rebuild(0.)
    return c,g


def test_entry_resampling_keeps_tree_radius_cap_and_hard_route_checks():
    c,g=fixture();current=np.array([4.,0.,2.]);tree=dict(id=1,xy=[0.,0.])
    g.state[tuple(g.indices([2.,0.,2.]))]=OCCUPIED;g.rebuild(0.)
    before=g.state.copy();p,d=select_approach_target(g,current,tree,0.,c)
    assert p is not None and d['candidate_count']>0
    assert np.linalg.norm(p[:2])==2. and 1.<=p[2]<=2.
    assert np.linalg.norm(p-[2.,0.,2.])>.1
    view=approach_grid(g,c)
    assert connected_endpoint(view,p) and plan(view,current,p) is not None
    np.testing.assert_array_equal(g.state,before)


def test_safe_island_across_unknown_wall_is_not_an_entry_route():
    c,g=fixture();g.state[g.indices([0.,0.,2.])[0],:,:]=UNKNOWN;g.rebuild(0.)
    current=np.array([-4.,0.,2.]);tree=dict(id=1,xy=[3.,0.])
    p,d=select_approach_target(g,current,tree,np.pi,c)
    assert p is None and d['result']=='no_safe_connected_entry'
    assert d['rejected']['disconnected']>0
    assert np.any(g.state==UNKNOWN)


def blocked_controller(with_alternative):
    c,g=fixture()
    # Every radius-two entry of tree1 is enclosed by measured obstacle;
    # tree2 has a free entry on the observer's side of that obstacle.
    ids=np.indices(g.shape).transpose(1,2,3,0);xyz=g.centers(ids)
    mask=(xyz[...,0]>=-.4)&(xyz[...,0]<=4.4)&(np.abs(xyz[...,1])<=2.4)
    g.state[mask]=OCCUPIED;g.rebuild(0.)
    ctl=TreeInspectionController(c,g,asynchronous=False)
    ctl.state='SURVEY';ctl.tree_phase='APPROACH';ctl.survey_now=5.
    ctl.active_tree=dict(id=1,xy=[2.,0.],observed_stem_top=4.,last_seen=5.)
    ctl.catalog=[ctl.active_tree.copy()]
    if with_alternative:ctl.catalog.append(dict(id=2,xy=[-3.,3.],observed_stem_top=4.,last_seen=5.))
    ctl.command=np.array([-3.,0.,2.]);ctl.view_target=np.array([0.,0.,2.]);ctl.blocked_since=4.
    return ctl


def test_blocked_entry_defers_same_tree_without_counting_or_resetting_deadline():
    ctl=blocked_controller(True)
    assert ctl._choose_goal(ctl.command)
    assert ctl.active_tree['id']==2 and 1 in ctl.deferred_tree_entries
    assert not ctl.deferred_tree_entries[1]['inspected']
    assert ctl.completed_trees==[] and ctl.completed_centres==[]
    assert ctl.blocked_since==4.
    assert [t['id'] for t in ctl.candidates()]==[2]
    ctl.survey_now=36.
    # Restore fresh observations; a temporary entry deferral is not a permanent
    # declaration that the entire tree can never be reached.
    for t in ctl.catalog:t['last_seen']=36.
    assert 1 in [t['id'] for t in ctl.candidates()]


def test_no_other_tree_keeps_blocked_goal_and_does_not_invent_completion():
    ctl=blocked_controller(False)
    ready=ctl._choose_goal(ctl.command)
    if ready:
        assert ctl.grid.safe(ctl.goal)
        assert np.linalg.norm(ctl.goal-ctl.view_target)>.1  # observation prefix, not blocked arrival
    assert ctl.active_tree['id']==1 and not ctl.deferred_tree_entries
    assert ctl.completed_trees==[] and ctl.blocked_since==4.


def test_deferred_entries_do_not_allow_return_completion_below_required_demo_count():
    ctl=blocked_controller(False)
    ctl.active_tree=None;ctl.tree_phase='SEARCH';ctl.view_target=None
    ctl.deferred_tree_entries={1:dict(retry_after=100.)}
    ctl.c['tree_demo_limit']=2;ctl.search_started=0.;ctl.survey_now=20.
    assert not ctl._choose_goal(ctl.command)
    assert ctl.failure=='no_reachable_tree_entry' and not ctl.completed_trees


def test_low_safe_scan_entry_is_not_forced_back_up_to_takeoff_height():
    from polinasi_nav.canopy_scan import scan_floor,canopy_profile,canopy_guard_grid
    c,g=fixture();c.update(scan_target_max_descent=.6,tree_ascent_observation_guard=.2)
    g.state[:,:,g.indices([0.,0.,2.1])[2]:]=OCCUPIED;g.rebuild(0.)
    assert scan_floor(c,1.3)==1.3
    np.testing.assert_allclose(scan_floor(c,2.8),2.2)
    points=np.array([[2.,0.,1.3],[2.,.3,1.3]])
    result,detail=canopy_profile(g,points,0,points[0],dict(observed_stem_top=4.),c,1.3)
    assert detail['floor_m']==1.3
    assert result[:,2].max()<2.
    view=canopy_guard_grid(g,.2)
    assert all(view.safe(p) for p in result)



def test_blocked_timeout_retains_current_map_after_failure_snapshot_budget(tmp_path):
    import json
    from polinasi_nav.planner_recovery import save_failure_snapshot
    from polinasi_nav.survey import SurveyController
    ctl=blocked_controller(False);ctl.c['planner_audit_dir']=str(tmp_path)
    for _ in range(8):save_failure_snapshot(ctl,dict(reason='ordinary'))
    ctl.route_blockage=dict(reason='occupied_blocked',planner_reason='goal_not_safe')
    now=ctl.blocked_since+ctl.c['blocked_timeout']+.1
    assert not SurveyController._prepare_trajectory(ctl,now,ctl.command)
    assert ctl.failure=='blocked_route'
    with np.load(tmp_path/'failure_critical.npz') as d:
        meta=json.loads(str(d['metadata']))
        assert meta['diagnostics']['reason']=='blocked_route'
        assert meta['diagnostics']['route_blockage']['planner_reason']=='goal_not_safe'
        np.testing.assert_array_equal(d['state'],ctl.grid.state)
