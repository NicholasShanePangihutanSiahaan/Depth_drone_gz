from pathlib import Path
import numpy as np
from polinasi_nav.config import load_config
from polinasi_nav.mapping import VoxelMap,FREE,OCCUPIED,UNKNOWN
from polinasi_nav.planning import plan
from polinasi_nav.path_cost import clearance_cost_field,line_travel_cost


def wall_grid():
    c=load_config(Path(__file__).resolve().parents[1]/'config/tree_inspection.json')
    c.update(bounds_min=[-4.,-4.,0.],bounds_max=[4.,4.,4.],
        drone_dimensions=[.2,.2,.2],clearance=.05,tracking_margin=.05,
        planner_clearance_preference=.4,planner_clearance_cost_weight=2.)
    g=VoxelMap(c);g.state[:]=FREE;g.seen[:]=0.
    g.state[:,g.indices([0.,.1,2.])[1],:]=OCCUPIED;g.rebuild(0.)
    return g


def test_cost_aware_route_and_shortcuts_prefer_room_outside_unchanged_hard_mask():
    g=wall_grid();p=np.array([-2.5,.7,2.]);q=np.array([2.5,.7,2.])
    state=g.state.copy();blocked=g.blocked.copy();actual=g.blocked_actual.copy()
    assert g.line_safe(p,q)
    field=clearance_cost_field(g);detail={}
    route=plan(g,p,q,diagnostics=detail)
    assert route is not None and len(route)>2
    assert np.asarray(route)[:,1].max()>=.9-1e-9
    assert all(g.line_safe(a,b) for a,b in zip(route[:-1],route[1:]))
    cost=sum(line_travel_cost(g,a,b,field) for a,b in zip(route[:-1],route[1:]))
    assert cost<line_travel_cost(g,p,q,field)
    assert detail['travel_cost']==cost
    np.testing.assert_array_equal(g.state,state)
    np.testing.assert_array_equal(g.blocked,blocked)
    np.testing.assert_array_equal(g.blocked_actual,actual)


def test_safe_direct_path_remains_available_if_detour_costs_more_or_budget_ends():
    g=wall_grid();p=np.array([-.1,.7,2.]);q=np.array([.1,.7,2.])
    assert len(plan(g,p,q))==2
    detail={};route=plan(g,p,q,max_expansions=0,diagnostics=detail)
    assert len(route)==2 and g.line_safe(*route)
    assert detail['clearance_search_fallback']=='search_limit'


def test_clearance_cost_does_not_open_unknown_or_raise_search_limit():
    g=wall_grid();g.state[g.indices([0.,0.,2.])[0],:,:]=UNKNOWN;g.rebuild(0.)
    p=np.array([-2.5,.7,2.]);q=np.array([2.5,.7,2.])
    assert not g.line_safe(p,q)
    detail={};assert plan(g,p,q,max_expansions=0,diagnostics=detail) is None
    assert detail['reason']=='search_limit'
    assert np.all(g.state[g.indices([0.,0.,2.])[0],:,:]==UNKNOWN)


def test_soft_cost_is_opt_in_and_transit_view_does_not_mutate_raw_config():
    from polinasi_nav.transit_guard import transit_guard_grid
    from polinasi_nav.canopy_scan import canopy_guard_grid
    g=wall_grid();g.c.pop('planner_clearance_preference');g.c.pop('planner_clearance_cost_weight')
    g.c.update(transit_clearance_preference=.4,transit_clearance_cost_weight=2.)
    assert clearance_cost_field(g) is None
    assert clearance_cost_field(canopy_guard_grid(g,.2)) is None
    view=transit_guard_grid(g,.2,.2)
    assert clearance_cost_field(view) is not None
    assert 'planner_clearance_preference' not in g.c
    assert view.c['clearance']==g.c['clearance'] and view.blocked_actual is g.blocked_actual


def test_preferred_route_and_smoothed_curve_survive_adjacent_new_occupied_evidence():
    from polinasi_nav.continuous import ContinuousTrajectory
    g=wall_grid();p=np.array([-2.5,.7,2.]);q=np.array([2.5,.7,2.])
    assert g.line_safe(p,q)
    route=plan(g,p,q)
    curve=ContinuousTrajectory(route,g.c)
    assert curve.safe(g)
    # Newly observed surface extends one voxel toward the flight corridor.
    # The old shortest chord is revoked; the preferred path retains headroom.
    g.state[tuple(g.indices([0.,.3,2.]))]=OCCUPIED;g.rebuild(.1)
    assert not g.line_safe(p,q)
    assert all(g.line_safe(a,b) for a,b in zip(route[:-1],route[1:]))
    assert curve.safe(g)
