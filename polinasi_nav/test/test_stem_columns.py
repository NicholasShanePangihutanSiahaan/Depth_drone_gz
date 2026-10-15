import numpy as np
from polinasi_nav.tree_perception import stem_column_evidence


def column(lean=0.,low=.7,high=2.7):
    angle,z=np.meshgrid(np.linspace(-1.2,1.2,20),np.linspace(low,high,20))
    return np.column_stack([3.+.25*np.cos(angle.ravel())+lean*z.ravel(),
        2.+.25*np.sin(angle.ravel()),z.ravel()])


def test_partial_vertical_cylinder_has_consistent_independent_height_layers():
    ok,d=stem_column_evidence(column(),np.array([3.,2.]),.25,0.)
    assert ok and d['consistent_layers']>=3 and d['low_stem_supported']


def test_curved_slanted_foliage_not_confirmed_by_projected_circle_alone():
    ok,_=stem_column_evidence(column(lean=.45),np.array([3.7,2.]),.4,0.)
    assert not ok


def test_short_or_only_upper_cluster_cannot_claim_stem():
    for low,high in ((1.6,2.7),(.7,1.6)):
        ok,_=stem_column_evidence(column(low=low,high=high),np.array([3.,2.]),.25,0.)
        assert not ok
