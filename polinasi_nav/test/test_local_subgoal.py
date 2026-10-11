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
