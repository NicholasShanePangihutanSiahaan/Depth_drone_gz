import numpy as np
from polinasi_nav.config import load_config
from polinasi_nav.mapping import VoxelMap,FREE,OCCUPIED,UNKNOWN
from polinasi_nav.local_subgoal import inspection_hint


def fixture():
    c = load_config()
    c.update(bounds_min=[-5.,-5.,0.],bounds_max=[5.,5.,6.],
        drone_dimensions=[.2,.2,.2],clearance=.1,tracking_margin=.1)
    grid = VoxelMap(c)
    grid.state[:] = FREE
    grid.seen[:] = 0.
    grid.state[tuple(grid.indices([2.,0.,3.]))] = OCCUPIED
    grid.rebuild(0.)
    return grid


def test_occupied_intermediate_redirects_not_final_goal():
    grid = fixture()
    target = np.array([4.,0.,3.])
    p,detail = inspection_hint(grid,[0.,0.,3.],target)
    assert detail['policy']=='observed_free_intermediate_detour'
    assert grid.safe(p) and grid.line_safe([0.,0.,3.],p)
    assert np.linalg.norm(p-[0.,0.,3.])<=2.+1e-8
    assert p[2]==3. and p[1]!=0.
    np.testing.assert_allclose(detail['final_goal'],target)


def test_unknown_is_not_projected_into_fake_free_space():
    grid = fixture()
    grid.state[:] = UNKNOWN
    grid.rebuild(0.)
    p,detail = inspection_hint(grid,[0.,0.,3.],[4.,0.,3.])
    assert detail['policy']=='nominal' and not grid.safe(p)
    np.testing.assert_allclose(p,[2.,0.,3.])


def test_fully_blocked_still_rejects():
    grid = fixture()
    grid.state[:] = OCCUPIED
    grid.rebuild(0.)
    p,detail = inspection_hint(grid,[0.,0.,3.],[4.,0.,3.],True)
    assert detail['candidate_count']==0 and not grid.safe(p)


def test_occupied_nominal_with_unknown_alternative_requests_observation_not_motion():
    grid = fixture()
    grid.state[:] = UNKNOWN
    grid.state[tuple(grid.indices([2.,0.,3.]))] = OCCUPIED
    grid.rebuild(0.)
    p,detail = inspection_hint(grid,[0.,0.,3.],[4.,0.,3.],True)
    assert detail['policy']=='unknown_alternative_requires_observation'
    assert not grid.safe(p) and not grid.occupied_goal(p)
    assert detail['unknown_count']>0 and detail['observation_candidate_count']>0
    assert not grid.line_safe([0.,0.,3.],p)
    np.testing.assert_allclose(detail['final_goal'],[4.,0.,3.])


def test_approach_checks_partial_descent_instead_of_only_final_height():
    from types import SimpleNamespace
    # A narrow observed-free band under leaves; lower final height is unknown.
    grid = SimpleNamespace(res=.2,
        occupied_goal=lambda p:p[0]>.2 and p[2]>2.7,
        safe=lambda p:2.5<=p[2]<=2.7,
        line_safe=lambda a,b:2.5<=b[2]<=2.7,
        corridor_blockers=lambda a,b:dict(occupied=[],outside=False,unknown=np.array([[0,0,0]])),
        centers=lambda ids:np.asarray(ids,dtype=float),
        indices=lambda p:np.floor(p/.2).astype(int),
        describe_blockage=lambda ids:dict(causes=['occupied']))
    p,detail = inspection_hint(grid,[0.,0.,3.],[12.,0.,2.],True)
    assert detail['policy']=='observed_free_intermediate_detour'
    assert p[0]>.2 and 2.5<=p[2]<=2.7
    assert np.linalg.norm(p-[0.,0.,3.])<=2.+1e-8
