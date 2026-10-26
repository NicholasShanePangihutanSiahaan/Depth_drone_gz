"""Geometry reuse is exact memoisation, never an age or stability policy."""
import copy
import numpy as np
import pytest
from polinasi_nav.config import load_config
from polinasi_nav.mapping import VoxelMap,FREE,OCCUPIED,UNKNOWN


def grid():
    c=load_config();c.update(bounds_min=[0.,0.,0.],bounds_max=[4.,4.,4.],resolution=.2,
        drone_dimensions=[.2,.2,.2],clearance=.1,tracking_margin=.1,free_ttl=1.)
    g=VoxelMap(c);g.state[:]=FREE;g.seen[:]=10.;g.state[10,10,10]=OCCUPIED
    return g


def compare(g,now,cache):
    plain=copy.deepcopy(g);plain.rebuild(now)
    g.rebuild(now,geometry_cache=cache)
    for name in ('state','seen','blocked','blocked_actual','distance','free_distance'):
        np.testing.assert_array_equal(getattr(g,name),getattr(plain,name))
    assert g.version==plain.version


def test_same_geometry_reuses_computation_but_does_not_freeze_observation_time():
    cache={};g=grid();compare(g,10.,cache)
    assert not any(g.geometry_reuse.values())
    g.seen[g.state==FREE]=10.5
    compare(g,10.6,cache)
    assert all(g.geometry_reuse.values()) and g.seen[5,5,5]==10.5
    compare(g,12.,cache)
    assert g.state[5,5,5]==UNKNOWN and not g.geometry_reuse['inflation']
    assert not g.geometry_reuse['free_distance']


@pytest.mark.parametrize('change',['occupied','unknown','fov_free','fov_occupied','origin','kernel','actual_kernel','resolution'])
def test_each_changed_geometry_dependency_is_revalidated(change):
    cache={};g=grid();compare(g,10.,cache)
    if change=='occupied':g.state[5,5,5]=OCCUPIED
    elif change=='unknown':g.state[5,5,5]=UNKNOWN
    elif change=='fov_free':
        g.state[5,5,5]=UNKNOWN;g.assumed_free[5,5,5]=True
    elif change=='fov_occupied':
        g.state[5,5,5]=UNKNOWN;g.assumed_free[5,5,5]=True;g.assumed_occupied[5,5,5]=True
    elif change=='origin':g.lo+=.2
    elif change=='actual_kernel':g.actual_kernel=np.ones((5,5,5),dtype=bool)
    elif change=='resolution':g.res=.25
    else:g.kernel=np.ones((5,5,5),dtype=bool)
    compare(g,10.1,cache)
    if change in ('occupied','unknown','fov_occupied','origin','kernel','actual_kernel','resolution'):
        assert not g.geometry_reuse['inflation']
    if change=='occupied':assert not g.geometry_reuse['occupied_distance']


def test_returned_map_mutation_cannot_poison_private_geometry_cache():
    cache={};first=grid();compare(first,10.,cache)
    first.blocked[:]=False;first.blocked_actual[:]=False
    first.distance[:]=999.;first.free_distance[:]=999.
    next_grid=grid();compare(next_grid,10.1,cache)
    assert all(next_grid.geometry_reuse.values())
