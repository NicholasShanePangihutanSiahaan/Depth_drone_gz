"""Conservative vertical-trunk verification near a camera candidate."""
import numpy as np


def fit_trunk(points, candidate, association_radius=0.8):
    p=np.asarray(points,dtype=float)
    p=p[np.isfinite(p).all(axis=1)]
    p=p[np.linalg.norm(p[:,:2]-np.asarray(candidate[:2]),axis=1)<association_radius]
    if len(p)<30:return None
    bins=np.floor(p[:,2]/0.25).astype(int)
    fits=[];rng=np.random.default_rng(7)
    for b in np.unique(bins):
        q=p[bins==b,:2]
        if len(q)<6:continue
        best=None
        for _ in range(60):
            a=q[rng.choice(len(q),3,replace=False)]
            mat=2*(a[1:]-a[0]);rhs=(a[1:]**2).sum(axis=1)-(a[0]**2).sum()
            if abs(np.linalg.det(mat))<1e-5:continue
            center=np.linalg.solve(mat,rhs);radius=np.linalg.norm(a[0]-center)
            if not .10<=radius<=.65 or np.linalg.norm(center-np.asarray(candidate[:2]))>.6:continue
            mask=np.abs(np.linalg.norm(q-center,axis=1)-radius)<.035
            if mask.sum()<6 or mask.mean()<.6:continue
            angle=np.sort(np.arctan2(*(q[mask]-center)[:,::-1].T))
            coverage=2*np.pi-np.diff(np.r_[angle,angle[0]+2*np.pi]).max()
            if coverage<.5:continue
            if best is None or mask.sum()>best[0]:best=(mask.sum(),center,radius)
        if best is not None:fits.append((b,*best))
    if len(fits)<4:return None
    centers=np.array([f[2] for f in fits]);center=np.median(centers,axis=0)
    good=[f for f in fits if np.linalg.norm(f[2]-center)<.12]
    if len(good)<4 or (max(f[0] for f in good)-min(f[0] for f in good))*.25<.75:return None
    radius=float(np.median([f[3] for f in good]))
    surface=p[np.abs(np.linalg.norm(p[:,:2]-center,axis=1)-radius)<.06]
    bottom,top=np.percentile(surface[:,2],[3,97]);height=top-bottom
    if height<.8:return None
    return np.array([center[0],center[1],bottom,height,radius])
