"""Exact, scan-local nearest-neighbour reuse with a triangle-inequality proof.

At a query anchor let d1/d2 be the nearest/second distance. For a displacement
s the nearest remains unchanged if d2-d1 > 2*s. A missing bounded second
neighbour has distance >= bound, which is a valid conservative lower bound.
Ambiguous cases always run the exact KD query; distances are recomputed.
"""
import numpy as np


class ExactCorrespondenceCache:
    def __init__(self,tree,bound=.5):
        if tree.boxsize is not None:raise ValueError('periodic trees are not supported')
        self.tree,self.bound=tree,float(bound)
        self.anchor=None;self.stats=dict(queries=0,kd_points=0,reused_points=0,tie_fallback_points=0)
        self.margin=1e-10*(1.+float(np.max(np.abs(tree.data),initial=0.)))

    def query(self,points):
        points=np.asarray(points,float)
        n=len(points);reuse=np.zeros(n,dtype=bool)
        if self.anchor is not None and self.anchor.shape==points.shape:
            movement=np.linalg.norm(points-self.anchor,axis=1)
            reuse=(self.first<self.bound)&(self.second-self.first>2*movement+self.margin)
        else:
            self.anchor=np.zeros_like(points);self.first=np.full(n,np.inf)
            self.second=np.full(n,np.inf);self.ids=np.zeros(n,dtype=int)
        distances=np.full(n,np.inf);ids=np.zeros(n,dtype=int)
        if np.any(reuse):
            distances[reuse]=np.linalg.norm(points[reuse]-self.tree.data[self.ids[reuse]],axis=1)
            ids[reuse]=self.ids[reuse]
            # Keep anchors/d1/d2 unchanged: displacement is cumulative since
            # the last exact two-neighbour query, never just the last iteration.
        refresh=~reuse
        if np.any(refresh):
            d,i=self.tree.query(points[refresh],k=2,eps=0.,distance_upper_bound=self.bound)
            nearest=d[:,0].copy();index=i[:,0].copy()
            ties=np.zeros(len(nearest),dtype=bool);finite=np.isfinite(nearest)
            ties[finite]=d[finite,1]-nearest[finite]<=self.margin
            if np.any(ties):
                # k=2 may order equal-distance IDs differently than legacy k=1.
                nearest[ties],index[ties]=self.tree.query(points[refresh][ties],eps=0.,distance_upper_bound=self.bound)
            index=np.where(np.isfinite(nearest),index,0)
            distances[refresh]=nearest;ids[refresh]=index
            self.anchor[refresh]=points[refresh];self.first[refresh]=nearest
            self.second[refresh]=np.minimum(d[:,1],self.bound);self.ids[refresh]=index
            self.stats['tie_fallback_points']+=int(np.count_nonzero(ties))
        # A reused nearest may leave the radius; retain exactly the same
        # upper-bound semantics as cKDTree.query and the ICP inlier gate.
        outside=distances>=self.bound
        distances[outside]=np.inf;ids[outside]=0
        self.stats['queries']+=1;self.stats['kd_points']+=int(np.count_nonzero(refresh))
        self.stats['reused_points']+=int(np.count_nonzero(reuse))
        return distances,ids
