"""Block retrieval must preserve sparse-history geometry and original timestamps."""
import numpy as np
from polinasi_nav.config import load_config
from polinasi_nav.coverage_mapping import SparseHistory,RollingMapper,navigation_window,rolling_input
from polinasi_nav.block_window_index import BlockWindowIndex
from polinasi_nav.mapping import FREE,OCCUPIED


def config():
    c=load_config();c.update(bounds_min=[-4.,-4.,0.],bounds_max=[6.,6.,5.],
        resolution=.2,navigation_window_size=[4.,4.,4.],free_ttl=2.,
        fixed_fov_assumptions=True,drone_dimensions=[.2,.2,.2],clearance=.1,tracking_margin=.1)
    return c


def equal_maps(a,b):
    for field in ('state','seen','assumed_free','assumed_occupied','blocked','blocked_actual','distance','free_distance'):
        np.testing.assert_array_equal(getattr(a,field),getattr(b,field))


def test_random_updates_expiry_priors_and_recentring_match_dictionary_history():
    c=config();plain=SparseHistory(c);indexed=SparseHistory(c)
    indexed.window_index=BlockWindowIndex(indexed.shape,side=4)
    rng=np.random.default_rng(42)
    for step in range(8):
        ids=rng.integers([0,0,0],plain.shape,size=(120,3))
        fov_ids=rng.integers([0,0,0],plain.shape,size=(80,3));codes=rng.integers(1,3,size=80)
        empty=(np.empty((0,3),dtype=int),np.empty(0,dtype=np.int8),np.empty(0),np.empty(0,bool))
        for h in (plain,indexed):
            h.update(ids,OCCUPIED if step%3==0 else FREE,float(step),prior=step==1)
            h.apply_delta((*empty,(fov_ids,codes,float(step))))
        assert indexed.cells==plain.cells and indexed.assumptions==plain.assumptions
        assert indexed.prior==plain.prior
        for centre in ([0.,0.,2.],[1.2,.6,2.]):
            for now in (float(step),float(step)+5.):
                a=navigation_window(c,centre);b=navigation_window(c,centre)
                plain.restore(a,now);plain.fov_priors(a);a.rebuild(now)
                indexed.restore(b,now);indexed.fov_priors(b);b.rebuild(now)
                equal_maps(a,b)
    assert indexed.window_index.status()['bytes']>0


def test_complete_mapper_updates_and_transport_delta_match_backend_disabled():
    c=config();indexed=RollingMapper(c);plain=RollingMapper(dict(c,mapping_block_index=False))
    assert indexed.geometry_cache is None  # Low live reuse benefit: disabled by default.
    initial=navigation_window(c,[0.,0.,2.]);reference=rolling_input(initial)
    for step,centre in enumerate(([0.,0.,2.],[.6,.4,2.],[1.2,.4,2.])):
        stamp=1.+step;now=stamp if step<2 else 20.
        points=np.array([[3.,.2,2.1],[.5,1.8,2.3],[2.2,2.2,3.1]])
        packet=dict(origin=[0.,0.,2.],rotation=np.eye(3),profile=np.full(360,np.inf),stamp=stamp)
        args=(reference,[0.,0.,2.],points,[True,True,True],stamp,None,now,centre)
        a,_,da=plain.integrate(*args,fixed_fov=packet)
        b,_,db=indexed.integrate(*args,fixed_fov=packet)
        equal_maps(a,b)
        for x,y in zip(da[:4],db[:4]):np.testing.assert_array_equal(x,y)
        for x,y in zip(da[4],db[4]):np.testing.assert_array_equal(x,y)
        assert indexed.history.cells==plain.history.cells
        assert indexed.history.assumptions==plain.history.assumptions
        reference=rolling_input(a)


def test_index_window_lookup_does_not_perform_per_voxel_dictionary_gets():
    class NoGet(dict):
        def get(self,*args):raise AssertionError('block restore must not query canonical cells per voxel')
    h=SparseHistory(config());h.window_index=BlockWindowIndex(h.shape)
    ids=h.indices([[.1,.1,2.1]]);h.update(ids,OCCUPIED,1.)
    h.cells=NoGet(h.cells);h.assumptions=NoGet(h.assumptions)
    g=navigation_window(h.c,[0,0,2]);h.restore(g,5.);h.fov_priors(g)
    assert g.state[tuple(g.indices([.1,.1,2.1]))]==OCCUPIED
    assert g.seen[tuple(g.indices([.1,.1,2.1]))]==1.
