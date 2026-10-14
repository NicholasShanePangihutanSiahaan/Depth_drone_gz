from pathlib import Path
import numpy as np
from polinasi_nav.config import load_config
from polinasi_nav.mapping import VoxelMap,FREE,OCCUPIED,UNKNOWN
from polinasi_nav.canopy_scan import canopy_profile,CanopyScanTrajectory,canopy_guard_grid
from polinasi_nav.continuous import ContinuousTrajectory


def fixture():
    c=load_config(Path(__file__).resolve().parents[1]/'config/tree_inspection.json')
    c.update(bounds_min=[-6.,-6.,0.],bounds_max=[6.,6.,7.],
             survey_batch_points=6,scan_height_max_slope=.45,
             scan_height_hysteresis=.08,scan_target_max_descent=.6,
             tree_ascent_observation_guard=.2,camera_mount=[0.,0.,.1])
    g=VoxelMap(c);g.state[:]=FREE;g.seen[:]=0.;g.rebuild(0.)
    angles=np.arange(1,25)*np.pi/12
    points=np.column_stack([2*np.cos(angles),2*np.sin(angles),np.full(24,2.8)])
    return c,g,points


def test_scan_rises_toward_estimated_stem_while_preserving_top_view_circle():
    c,g,points=fixture()
    result,detail=canopy_profile(g,points,0,[2.,0.,2.8],dict(observed_stem_top=4.),c,2.8)
    np.testing.assert_allclose(result[:,:2],points[:,:2])
    assert result[0,2]>2.8 and result[5,2]>result[0,2]
    assert max(result[:6,2])<=3.9+1e-8
    assert detail['target_kind']=='estimated_stem_top_not_detected_flower'
    np.testing.assert_allclose(result[6:],points[6:])


def test_height_profile_cannot_jump_through_leaves_to_free_air_above():
    c,g,points=fixture()
    roof=g.indices([0.,0.,4.2])[2]
    g.state[:,:,roof]=OCCUPIED;g.rebuild(0.)
    result,detail=canopy_profile(g,points,0,[2.,0.,2.8],dict(observed_stem_top=5.5),c,2.8)
    assert max(result[:6,2])<3.5
    for p in result[:6]:
        assert g.line_safe([p[0],p[1],detail['floor_m']],p+[0.,0.,.2])


def test_unknown_overhead_never_authorizes_upward_motion():
    c,g,points=fixture()
    unknown=g.indices([0.,0.,3.8])[2]
    g.state[:,:,unknown:]=UNKNOWN;g.rebuild(0.)
    result,detail=canopy_profile(g,points,0,[2.,0.,2.8],dict(observed_stem_top=5.5),c,2.8)
    assert max(result[:6,2])<3.4


def test_newly_observed_headroom_can_raise_profile_again_after_lower_roof():
    c,g,points=fixture()
    roof=g.indices([0.,0.,4.0])[2]
    g.state[:,:,roof]=OCCUPIED;g.rebuild(0.)
    low,_=canopy_profile(g,points,0,[2.,0.,2.8],dict(observed_stem_top=5.),c,2.8)
    # A DIFFERENT, clear map section, not permission to erase real foliage.
    clear=VoxelMap(c);clear.state[:]=FREE;clear.seen[:]=0.;clear.rebuild(0.)
    high,_=canopy_profile(clear,low,0,[2.,0.,2.8],dict(observed_stem_top=5.),c,2.8)
    assert high[5,2]>low[5,2]+.2


def test_canopy_buffer_is_checked_for_whole_curve_and_live_suffix():
    c,g,_=fixture()
    points=np.array([[1.,0.,2.8],[2.,0.,2.8]])
    base=ContinuousTrajectory(points,c)
    # A low enough roof to block only the extra reserved observation buffer.
    g.state[:,:,g.indices([0.,0.,3.95])[2]]=OCCUPIED;g.rebuild(0.)
    assert base.safe(g)
    guarded=CanopyScanTrajectory(base,.4)
    assert not guarded.safe(g)
    assert not guarded.safe_from(g,base.duration/2)


def test_three_dimensional_scan_curve_keeps_dynamic_bounds():
    c,g,points=fixture()
    result,_=canopy_profile(g,points,0,[2.,0.,2.8],dict(observed_stem_top=4.),c,2.8)
    trajectory=CanopyScanTrajectory(ContinuousTrajectory(np.vstack([[2.,0.,2.8],result[:6]]),c),.2)
    assert trajectory.safe(g)
    samples=[trajectory.derivatives(t) for t in np.linspace(0.,trajectory.duration,100)]
    assert any(np.linalg.norm(v[:2])>.01 and v[2]>.01 for _,v,_,_ in samples)
    assert max(np.linalg.norm(v) for _,v,_,_ in samples)<=c['command_speed']+1e-6
    assert max(np.linalg.norm(a) for _,_,a,_ in samples)<=c['acceleration']+1e-6
    assert max(np.linalg.norm(j) for _,_,_,j in samples)<=c['jerk']+1e-6


def test_guard_planning_mask_reserves_overhead_without_mutating_map():
    c,g,_=fixture()
    g.state[:,:,g.indices([0.,0.,3.95])[2]]=OCCUPIED;g.rebuild(0.)
    original=g.blocked.copy();raw=g.state.copy()
    guarded=canopy_guard_grid(g,.4)
    assert g.safe([1.,0.,2.8]) and not guarded.safe([1.,0.,3.0])
    assert not guarded.line_safe([1.,0.,2.8],[1.,0.,3.0])
    np.testing.assert_array_equal(g.blocked,original)
    np.testing.assert_array_equal(g.state,raw)
    assert guarded.blocked_actual is g.blocked_actual
