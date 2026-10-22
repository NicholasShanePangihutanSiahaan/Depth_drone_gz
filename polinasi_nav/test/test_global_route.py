from pathlib import Path
import numpy as np
from polinasi_nav.config import load_config
from polinasi_nav.mapping import VoxelMap,FREE,OCCUPIED
from polinasi_nav.coverage_mapping import SparseHistory
from polinasi_nav.global_route import plan_history_route,local_route_goal


def fixture():
    c=load_config(Path(__file__).resolve().parents[1]/'config/tree_inspection.json')
    c.update(bounds_min=[-6.,-6.,0.],bounds_max=[6.,6.,6.],
        drone_dimensions=[.2,.2,.2],clearance=.05,tracking_margin=.05,
        tracking_limit=.05,history_route_margin=3.,history_route_max_voxels=200000,
        history_route_max_expansions=30000,history_route_max_age=8.,history_route_local_lookahead=2.)
    grid=VoxelMap(c);grid.state[:]=FREE;grid.seen[:]=0.;grid.rebuild(0.)
    history=SparseHistory(c)
    history.update(np.indices(grid.shape).reshape(3,-1).T,FREE,0.)
    history.last_observation_stamp=0.
    return c,grid,history


def request(start=(-3.,0.,2.),goal=(3.,0.,2.)):
    return dict(start=list(start),goal=list(goal),request_id=1)


def test_full_history_route_reaches_final_goal_and_avoids_obstacle():
    c,g,h=fixture()
    block=g.indices([-1.,0.,2.])
    h.update([block],OCCUPIED,0.)
    g.state[tuple(block)]=OCCUPIED;g.rebuild(0.)
    route=plan_history_route(h,request(),0.)
    assert route['path'] and route['final_goal_reached_by_route'] and not route['partial']
    np.testing.assert_allclose(route['path'][-1],[3.,0.,2.])
    assert len(route['path'])>2
    for p,q in zip(route['path'][:-1],route['path'][1:]):assert g.line_safe(p,q)
    selected,detail=local_route_goal(g,[-3.,0.,2.],[3.,0.,2.],route,0.,c)
    assert selected is not None and g.safe(selected)
    # The geometric 2m intermediate [-1,0,2] is occupied, but no longer used.
    assert not np.allclose(selected,[-1.,0.,2.])


def test_unknown_gap_only_returns_reachable_partial_route():
    c,g,h=fixture()
    # New measured history with only the left part observed free.
    h=SparseHistory(c)
    ids=np.indices(g.shape).reshape(3,-1).T
    h.update(ids[g.centers(ids)[:,0]<0.],FREE,0.)
    h.last_observation_stamp=0.
    route=plan_history_route(h,request(),0.)
    assert route['path'] and route['partial'] and not route['final_goal_reached_by_route']
    assert route['path'][-1][0]<0.
    np.testing.assert_allclose(route['goal'],[3.,0.,2.])


def test_expired_free_history_cannot_be_used_for_route():
    c,g,h=fixture();h.c['free_ttl']=.5
    route=plan_history_route(h,request(),1.)
    assert not route['path'] and route['reason']=='history_start_not_safe'


def test_complete_wall_is_not_mistaken_for_full_route():
    c,g,h=fixture()
    ids=np.indices(g.shape).reshape(3,-1).T
    h.update(ids[np.abs(g.centers(ids)[:,0])<.15],OCCUPIED,0.)
    route=plan_history_route(h,request(),0.)
    assert not route['final_goal_reached_by_route']
    assert route['partial'] and route['path'][-1][0]<0.


def test_live_map_rejects_blocked_path_prefix_and_stale_or_wrong_goal():
    c,g,h=fixture();route=plan_history_route(h,request(),0.)
    assert local_route_goal(g,[-3.,0.,2.],[3.,0.,2.],route,9.,c)[0] is None
    assert local_route_goal(g,[-3.,0.,2.],[3.,1.,2.],route,0.,c)[0] is None
    g.state[tuple(g.indices([-2.8,0.,2.]))]=OCCUPIED;g.rebuild(0.)
    point,detail=local_route_goal(g,[-3.,0.,2.],[3.,0.,2.],route,0.,c)
    assert point is None and detail['reason']=='live_prefix_blocked'


def test_voxel_budget_holds_instead_of_allocating_whole_farm():
    c,g,h=fixture();h.c['history_route_max_voxels']=10
    route=plan_history_route(h,request(),0.)
    assert not route['path'] and route['reason']=='history_search_budget'


def test_global_route_can_reach_beyond_local_window_but_local_goal_cannot():
    c,g,h=fixture()
    route=plan_history_route(h,request(start=(-2.,0.,2.)),0.)
    local_config=dict(c,bounds_min=[-3.,-3.,0.],bounds_max=[0.,3.,5.])
    local=VoxelMap(local_config);h.restore(local,0.);local.rebuild(0.)
    assert not local.safe([3.,0.,2.])
    point,_=local_route_goal(local,[-2.,0.,2.],[3.,0.,2.],route,0.,c)
    assert point is not None and local.safe(point) and -2.<point[0]<0.


def test_survey_return_waits_for_history_route_without_creating_fake_motion():
    from polinasi_nav.survey import SurveyController
    c,g,h=fixture()
    c.update(survey_waypoints=[[0.,0.,2.]],history_route_enabled=True)
    ctl=SurveyController(c,g,asynchronous=False)
    ctl.state='RETURN';ctl.command=np.array([3.,0.,2.]);ctl.survey_now=0.
    assert not ctl._prepare_trajectory(0.,ctl.command)
    assert ctl.trajectory is None and ctl.state=='RETURN' and not ctl.failure
    ctl.global_route=plan_history_route(h,request(start=(3.,0.,2.),goal=(0.,0.,2.)),0.)
    assert ctl._prepare_trajectory(0.,ctl.command)
    assert ctl.trajectory.safe(g) and ctl.state=='RETURN'
    np.testing.assert_allclose(ctl.route_destination(),[0.,0.,2.])


def test_history_guidance_keeps_explicit_FOV_assumptions_separate_from_measurement():
    c,g,h=fixture();c['fixed_fov_assumptions']=True
    h=SparseHistory(c)
    ids=np.indices(g.shape).reshape(3,-1).T
    keys=np.ravel_multi_index(ids.T,h.shape)
    h.assumptions={int(k):(1,0.) for k in keys};h.last_observation_stamp=0.
    route=plan_history_route(h,request(),0.)
    assert route['final_goal_reached_by_route']
    assert route['source']=='sparse_history_with_FOV_prior'
    assert not h.cells  # Not relabelled as measured free.
    h.update([g.indices([-1.,0.,2.])],OCCUPIED,0.)
    route=plan_history_route(h,request(),0.)
    assert len(route['path'])>2  # Measured obstacle defeats assumed free.


def test_history_route_uses_same_overhead_buffer_as_local_transit_curve():
    from polinasi_nav.canopy_scan import canopy_guard_grid
    c,g,h=fixture()
    leaf=g.indices([-1.,0.,2.7])
    h.update([leaf],OCCUPIED,0.);g.state[tuple(leaf)]=OCCUPIED;g.rebuild(0.)
    direct=plan_history_route(h,request(),0.)
    assert direct['path'] and len(direct['path'])==2
    guarded=canopy_guard_grid(g,.2)
    assert not guarded.line_safe(direct['path'][0],direct['path'][-1])
    h.c['transit_observation_guard']=.2
    route=plan_history_route(h,request(),0.)
    assert route['observation_guard_m']==.2
    assert route['final_goal_reached_by_route'] and len(route['path'])>2
    assert all(guarded.line_safe(p,q) for p,q in zip(route['path'],route['path'][1:]))


def test_history_goal_on_blocked_voxel_face_uses_reachable_partial_endpoint():
    # Run8 geometry: goal z=2.0 lies on a face between safe 1.9 and blocked
    # 2.1 centres. The search origin's floating representation picks the lower
    # cell for point-safe, while supercover correctly checks both neighbours.
    c=load_config(Path(__file__).resolve().parents[1]/'config/tree_inspection.json')
    c.update(bounds_min=[-40.,-40.,-.6],bounds_max=[52.,42.,12.],
        drone_dimensions=[.6,.6,.7],history_route_margin=4.,
        history_route_max_voxels=800000,transit_observation_guard=.2,
        transit_lateral_observation_guard=.2)
    local=VoxelMap(dict(c,bounds_min=[-11.4,-5.8,-.6],bounds_max=[-1.4,4.2,7.4]))
    ids=np.indices(local.shape).reshape(3,-1).T
    h=SparseHistory(c);h.update(h.indices(local.centers(ids)),FREE,0.)
    h.update([h.indices([-5.3,-.3,3.1])],OCCUPIED,0.)
    h.last_observation_stamp=0.
    start=[-6.312499999999997,-.6499999999999956,1.75]
    goal=[-6.486624939699138,-.3269617013590112,2.]
    route=plan_history_route(h,request(start,goal),0.)
    assert route['goal_connection']['point_safe']
    assert not route['goal_connection']['centre_connection_safe']
    assert route['path'] and route['partial']
    assert not route['final_goal_reached_by_route']
    assert route['planner']['reason'] in ('direct','detour')
    np.testing.assert_allclose(route['goal'],goal)
    assert not np.array_equal(route['endpoint'],goal)
    assert np.linalg.norm(np.asarray(route['endpoint'])-goal)<.2
    # Occupied evidence is preserved and no unknown cell was inserted as free.
    key=np.ravel_multi_index(h.indices([-5.3,-.3,3.1]),h.shape)
    assert h.cells[key][0]==OCCUPIED


def test_near_destination_does_not_bypass_blocked_supercover_connection():
    c,g,h=fixture()
    goal=np.array([0.,0.,np.nextafter(2.,0.)])
    current=goal-[0.,0.,.05]
    # A real measured roof inflates to the upper neighbour at 2.1 m;
    # the lower 1.9 m cell remains free. Neither point may authorize contact.
    roof=g.indices([0.,0.,2.5])[2]
    g.state[:,:,roof]=OCCUPIED;g.rebuild(0.)
    assert g.safe(goal) and not g.line_safe(current,goal)
    selected,detail=local_route_goal(g,current,goal,None,0.,c)
    assert selected is None and detail['reason']=='waiting_history_route'
