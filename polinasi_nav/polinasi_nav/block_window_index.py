"""Exact block-hash lookup accelerator; timestamps and unknown remain explicit."""
from itertools import product
from types import SimpleNamespace
import numpy as np
from .mapping import FREE,OCCUPIED,UNKNOWN


class BlockWindowIndex:
    def __init__(self,shape,side=16):
        self.shape=np.asarray(shape,dtype=int)
        self.side=int(side)
        if self.side<1:raise ValueError('block side must be positive')
        self.block_shape=tuple((self.shape+self.side-1)//self.side)
        self.blocks={}

    def _groups(self,ids):
        ids=np.asarray(ids,dtype=int).reshape(-1,3)
        if not len(ids):return
        if np.any(ids<0) or np.any(ids>=self.shape):raise ValueError('voxel outside history')
        keys=np.ravel_multi_index((ids//self.side).T,self.block_shape)
        unique,inverse=np.unique(keys,return_inverse=True)
        for group,key in enumerate(unique):
            selected=inverse==group
            block=self.blocks.get(int(key))
            if block is None:
                shape=(self.side,)*3
                block=SimpleNamespace(state=np.full(shape,UNKNOWN,dtype=np.int8),
                    seen=np.full(shape,-np.inf),prior=np.zeros(shape,dtype=bool),
                    fov=np.zeros(shape,dtype=np.int8))
                self.blocks[int(key)]=block
            yield block,selected,tuple((ids[selected]%self.side).T)

    def update(self,ids,states,stamps,priors=False):
        ids=np.asarray(ids,dtype=int).reshape(-1,3)
        n=len(ids)
        states=np.broadcast_to(states,(n,));stamps=np.broadcast_to(stamps,(n,))
        priors=np.broadcast_to(priors,(n,))
        for block,selected,local in self._groups(ids):
            block.state[local]=states[selected];block.seen[local]=stamps[selected]
            block.prior[local]|=priors[selected]

    def update_fov(self,ids,codes):
        codes=np.asarray(codes)
        for block,selected,local in self._groups(ids):block.fov[local]=codes[selected]

    def _overlaps(self,history,grid):
        start=history.indices(grid.centers(np.zeros(3,dtype=int)))
        stop=start+np.asarray(grid.shape)
        if np.any(start<0) or np.any(stop>self.shape):raise ValueError('window outside history')
        for coordinate in product(*(range(a,b+1) for a,b in zip(start//self.side,(stop-1)//self.side))):
            key=int(np.ravel_multi_index(coordinate,self.block_shape))
            block=self.blocks.get(key)
            if block is None:continue  # Missing block is unknown, never free.
            origin=np.asarray(coordinate)*self.side
            lo=np.maximum(start,origin);hi=np.minimum(stop,origin+self.side)
            local=tuple(slice(int(a),int(b)) for a,b in zip(lo-start,hi-start))
            source=tuple(slice(int(a),int(b)) for a,b in zip(lo-origin,hi-origin))
            yield block,source,local

    def restore(self,history,grid,now):
        for block,source,target in self._overlaps(history,grid):
            state,seen,prior=block.state[source],block.seen[source],block.prior[source]
            live=(state!=UNKNOWN)&((state==OCCUPIED)|prior|(now-seen<=history.c['free_ttl']))
            grid.state[target][live]=state[live]
            grid.seen[target][live]=np.where(prior[live],now,seen[live])

    def restore_fov(self,history,grid):
        grid.assumed_free[:]=False;grid.assumed_occupied[:]=False
        for block,source,target in self._overlaps(history,grid):
            grid.assumed_free[target]=block.fov[source]==1
            grid.assumed_occupied[target]=block.fov[source]==2

    def status(self):
        return dict(blocks=len(self.blocks),side=self.side,
            bytes=sum(a.nbytes for b in self.blocks.values() for a in (b.state,b.seen,b.prior,b.fov)),
            policy='exact_measured_state_stamp_and_existing_prior_index_no_age_relaxation')
