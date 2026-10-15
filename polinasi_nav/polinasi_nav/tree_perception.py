"""Bounded geometric stem candidates; no simulator tree coordinates accepted."""
import time
import numpy as np
from scipy.spatial import cKDTree


def stem_column_evidence(cluster,centre,radius,ground):
    """Reject drooping foliage: require consistent circles at several heights.

    Geometric evidence only, not a semantic palm detector or known tree layout.
    Missing/ambiguous low-stem returns leave a candidate unconfirmed.
    """
    layers=[]
    for bottom in np.arange(ground+.6,ground+2.7,.5):
        section=cluster[(cluster[:,2]>=bottom)&(cluster[:,2]<bottom+.5)]
        if len(section)<6:continue
        xy=section[:,:2]
        a=np.column_stack((2*xy,np.ones(len(xy))))
        if np.linalg.matrix_rank(a)<3:continue
        x,y,k=np.linalg.lstsq(a,np.sum(xy*xy,axis=1),rcond=None)[0]
        r=np.sqrt(max(0.,k+x*x+y*y))
        residual=float(np.mean(np.abs(np.linalg.norm(xy-[x,y],axis=1)-r)))
        layers.append(dict(z=float(bottom),centre=[float(x),float(y)],radius=float(r),residual=residual))
    consistent=[s for s in layers if np.linalg.norm(np.asarray(s['centre'])-centre)<=.2
        and abs(s['radius']-radius)<=.15 and s['residual']<=.12]
    low=any(s['z']<ground+1.2 for s in consistent)
    span=float(np.ptp(cluster[:,2])) if len(cluster) else 0.
    accepted=bool(low and span>=1.2 and len(consistent)>=3 and
        max(s['z'] for s in consistent)-min(s['z'] for s in consistent)>=1.-1e-9)
    return accepted,dict(vertical_span_m=span,consistent_layers=len(consistent),
        low_stem_supported=bool(low),layers=layers)


class StemDetector:
    def __init__(self):
        self.cells = {}
        self.tracks = []
        self.next_id = 1
        # Flat-farm prototype: retain a measured launch-ground reference.
        # Looking upward must not redefine canopy points as the ground.
        self.ground_reference = None
        self.ground_reference_stamp = None

    def update(self, points, stamp):
        started = time.perf_counter()
        points = np.asarray(points, float).reshape(-1, 3)
        points = points[np.all(np.isfinite(points), axis=1)]
        for p in points:
            key = tuple(np.floor(p/.15).astype(int))
            self.cells[key] = (p, stamp)
        self.cells = {k:v for k,v in self.cells.items() if stamp-v[1] <= 30.}
        if len(self.cells) > 40000:
            self.cells = dict(list(self.cells.items())[-40000:])
        all_points = np.asarray([v[0] for v in self.cells.values()]).reshape(-1,3)
        if len(all_points) < 30:
            self.tracks = [t for t in self.tracks if stamp-t.get('last_seen',stamp) < 60.]
            return dict(stamp=float(stamp), frame='map', source='measured_lidar_geometry',
                        tracks=self.tracks, wall_ms=(time.perf_counter()-started)*1000.,
                        retained_points=len(all_points), stem_points=0, reason='insufficient_points')
        recent_ground = float(np.percentile(all_points[:,2], 5))
        if self.ground_reference is None:
            low = all_points[np.abs(all_points[:,2]-recent_ground) <= .15]
            # A compact stem/leaf cluster alone is not ground evidence.
            if len(low) < 20 or np.max(np.ptp(low[:,:2], axis=0)) < 2.:
                return dict(stamp=float(stamp), frame='map', source='measured_lidar_geometry',
                            tracks=[], wall_ms=(time.perf_counter()-started)*1000.,
                            retained_points=len(all_points), stem_points=0,
                            reason='waiting_for_launch_ground_reference',
                            recent_ground_percentile_m=recent_ground,
                            ground_reference_m=None)
            self.ground_reference = float(np.median(low[:,2]))
            self.ground_reference_stamp = float(stamp)
        ground = self.ground_reference
        # Conservative prototype: fit trunks below the foliage layer.
        p = all_points[(all_points[:,2] > ground+.6) & (all_points[:,2] < ground+2.8)]
        p = p[::max(1,int(np.ceil(len(p)/6000)))]
        components = []
        if len(p):
            tree = cKDTree(p[:,:2])
            unseen = set(range(len(p)))
            while unseen:
                todo = [unseen.pop()]
                component = []
                while todo:
                    i = todo.pop()
                    component.append(i)
                    neighbours = set(tree.query_ball_point(p[i,:2], .4)) & unseen
                    unseen.difference_update(neighbours)
                    todo.extend(neighbours)
                if len(component) >= 12:
                    components.append(p[component])
        detections = []
        rejected_columns=0
        for cluster in components:
            if np.ptp(cluster[:,2]) < .8 or np.max(np.ptp(cluster[:,:2],axis=0)) > 1.5:
                continue
            xy = cluster[:,:2]
            a = np.column_stack((2*xy, np.ones(len(xy))))
            cx,cy,c = np.linalg.lstsq(a, np.sum(xy*xy,axis=1),rcond=None)[0]
            radius = np.sqrt(max(0.,c+cx*cx+cy*cy))
            residual = float(np.mean(np.abs(np.linalg.norm(xy-[cx,cy],axis=1)-radius)))
            if not .08 <= radius <= .7 or residual > .10:
                continue
            centre = np.array([cx,cy])
            column_ok,column_detail=stem_column_evidence(cluster,centre,radius,ground)
            if not column_ok:
                rejected_columns+=1
                continue
            related = all_points[np.linalg.norm(all_points[:,:2]-centre,axis=1) < radius+.35]
            observed_top = float(np.percentile(related[:,2],95))
            detections.append((centre,radius,observed_top,residual,len(cluster),column_detail))
        matched = set()
        for centre,radius,top,residual,count,column_detail in detections:
            candidates = [(np.linalg.norm(np.asarray(t['xy'])-centre),t)
                          for t in self.tracks if t['id'] not in matched]
            distance, track = min(candidates,key=lambda item:item[0]) if candidates else (np.inf,None)
            if distance > 1.:
                track = dict(id=self.next_id, xy=centre.tolist(), observations=0)
                self.next_id += 1
                self.tracks.append(track)
            matched.add(track['id'])
            track.update(xy=(.7*np.asarray(track['xy'])+.3*centre).tolist(), radius=float(radius),
                         observed_stem_top=top, stem_top_complete=False, last_seen=float(stamp),
                         residual_m=residual, support=count, observations=track['observations']+1,
                         column_evidence=column_detail)
            track['confirmed'] = track['observations'] >= 3
        self.tracks = [t for t in self.tracks if stamp-t.get('last_seen',stamp) < 60.]
        return dict(stamp=float(stamp), frame='map', source='measured_lidar_geometry',
                    tracks=self.tracks, wall_ms=(time.perf_counter()-started)*1000.,
                    retained_points=len(all_points), stem_points=len(p), reason='ok',
                    ground_reference_m=ground, ground_reference_stamp=self.ground_reference_stamp,
                    recent_ground_percentile_m=recent_ground,
                    ground_policy='measured_launch_reference_flat_terrain_only',
                    stem_height_band_m=[ground+.6, ground+2.8],
                    rejected_inconsistent_columns=rejected_columns,
                    detections_count=len(detections))
