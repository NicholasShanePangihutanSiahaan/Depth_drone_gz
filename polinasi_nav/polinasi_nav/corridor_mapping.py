"""Measured-space ROI only; selecting a region never declares it free."""
import numpy as np


def sample_intersections(origin, endpoints, roi, spacing):
    """Clip measured rays before sampling, then apply exact ROI membership.

    Conservative bounding boxes accelerate intersection; their corners never
    become free because contains() checks the exact spheres/capsules afterward.
    Keep the original ray sampling lattice, including disjoint ROI intervals.
    """
    origin, endpoints = np.asarray(origin), np.asarray(endpoints).reshape(-1, 3)
    boxes = []
    def box(points, radius):
        points = np.asarray(points)
        boxes.append((points.min(axis=0)-radius, points.max(axis=0)+radius))
    box([roi['centre']], roi['near_radius'])
    for route in roi['routes']:
        route = np.asarray(route)
        if len(route) < 2:
            continue
        group, distance = [route[0]], 0.
        for first, second in zip(route, route[1:]):
            group.append(second)
            distance += np.linalg.norm(second-first)
            if distance >= 2.:
                box(group, roi['radius'])
                group, distance = [second], 0.
        if len(group) > 1:
            box(group, roi['radius'])
    for centre in roi.get('expansion_centres', []):
        box([centre], roi['expansion_radius'])
    delta = endpoints-origin
    lengths = np.linalg.norm(delta, axis=1)
    intervals = [[] for _ in endpoints]
    parallel = np.abs(delta) < 1e-12
    for lo, hi in boxes:
        with np.errstate(divide='ignore', invalid='ignore'):
            a, b = (lo-origin)/delta, (hi-origin)/delta
        lower = np.where(parallel, -np.inf, np.minimum(a, b))
        upper = np.where(parallel, np.inf, np.maximum(a, b))
        enter = np.maximum(0., lower.max(axis=1))
        leave = np.minimum(1., upper.min(axis=1))
        good = (enter <= leave) & ~np.any(parallel & ((origin < lo) | (origin > hi)), axis=1)
        for index in np.flatnonzero(good):
            intervals[index].append((float(enter[index]), float(leave[index])))
    samples = []
    for index, pieces in enumerate(intervals):
        merged = []
        for start, end in sorted(pieces):
            if merged and start <= merged[-1][1]:
                merged[-1][1] = max(merged[-1][1], end)
            else:
                merged.append([start, end])
        for start, end in merged:
            length = max(lengths[index], 1e-9)
            first = int(np.ceil(start*length/spacing-1e-9))
            last = int(np.floor(end*length/spacing+1e-9))
            distance = np.arange(first, last+1)*spacing
            distance = distance[distance < length]
            samples.append(origin+(distance/length)[:, None]*delta[index])
    candidates = np.vstack(samples) if samples else np.empty((0, 3))
    result = candidates[contains(candidates, roi)]
    return result, dict(full_trace_samples=int(np.ceil(np.maximum(lengths, 1e-9)/spacing).sum()),
        generated_samples=len(candidates), retained_samples=len(result),
        rays_without_roi_intersection=sum(not p for p in intervals))


def contains(points, roi):
    points = np.asarray(points, float).reshape(-1, 3)
    delta = points-np.asarray(roi['centre'])
    keep = np.einsum('ij,ij->i', delta, delta) <= roi['near_radius']**2
    routes = roi['routes']
    for route in routes:
        for start, end in zip(route, route[1:]):
            start, end = np.asarray(start), np.asarray(end)
            radius = roi['radius']
            candidate = np.flatnonzero(~keep & np.all(
                (points >= np.minimum(start, end)-radius) &
                (points <= np.maximum(start, end)+radius), axis=1))
            if not len(candidate):
                continue
            d = end-start
            t = np.clip((points[candidate]-start)@d/max(float(d@d), 1e-12), 0., 1.)
            distance = points[candidate]-(start+t[:, None]*d)
            keep[candidate] |= np.einsum('ij,ij->i', distance, distance) <= radius**2
    for centre in roi.get('expansion_centres', []):
        delta = points-np.asarray(centre)
        keep |= np.einsum('ij,ij->i', delta, delta) <= roi['expansion_radius']**2
    return keep


def lookahead(current, path, maximum):
    result = [np.asarray(current, float)]
    remaining = float(maximum)
    for point in np.asarray(path, float).reshape(-1, 3):
        delta = point-result[-1]
        length = float(np.linalg.norm(delta))
        if length < 1e-6:
            continue
        result.append(result[-1]+delta*min(1., remaining/length))
        remaining -= length
        if remaining <= 0:
            break
    return np.asarray(result).tolist()


def simplify_roi_route(route, tolerance):
    """Bound distance of every original segment to simplified chords.

    Distance to a convex segment is convex: bounding both endpoints of each
    original edge also bounds the full edge. Enlarging capsule radius by the
    returned error therefore retains the original ROI, not only its vertices.
    This never simplifies a flight trajectory or labels unknown cells free.
    """
    points = np.asarray(route,float).reshape(-1,3)
    if len(points)<3:
        return points.tolist(),0.
    keep = {0,len(points)-1}
    pending = [(0,len(points)-1)]
    error = 0.
    while pending:
        first,last = pending.pop()
        if last-first<2:
            continue
        d = points[last]-points[first]
        middle = points[first+1:last]
        t = np.clip((middle-points[first])@d/max(float(d@d),1e-12),0.,1.)
        distances = np.linalg.norm(middle-(points[first]+t[:,None]*d),axis=1)
        index = int(np.argmax(distances))
        bound = float(distances[index])
        if bound>tolerance:
            split = first+1+index
            keep.add(split)
            pending.extend(((first,split),(split,last)))
        else:
            error = max(error,bound)
    return points[sorted(keep)].tolist(),error


class CorridorRegion:
    def __init__(self, config):
        self.c = config
        self.centres = []
        self.radius = config['mapping_corridor_radius']
        self.last_expansion_stamp = -np.inf
        self.expansions = 0

    def region(self, current, status, nominal, planned, stamp, hit_points=None):
        blockage = status.get('route_blockage', {})
        blocked = (blockage.get('occupied_count', 0) > 0 and
                   blockage.get('reason') != 'route_found')
        samples = blockage.get('occupied_samples', [])
        if blocked and samples and stamp-self.last_expansion_stamp >= self.c['mapping_corridor_expand_interval']:
            # Keep discovered detour room: shrinking it could reintroduce
            # unknown cells under an already certified manoeuvre.
            for p in samples:
                if not self.centres or min(np.linalg.norm(np.asarray(p)-q) for q in self.centres) > .5:
                    self.centres.append(np.asarray(p))
            self.centres = self.centres[-32:]
            self.radius = min(self.c['mapping_corridor_max_radius'],
                              self.radius+self.c['mapping_corridor_expand_step'])
            self.expansions += 1
            self.last_expansion_stamp = stamp
        horizon = min(self.c['lidar_range'], self.c['mapping_corridor_lookahead'])
        routes = [lookahead(current, nominal, horizon)]
        if len(planned):
            # Planned points contain the trajectory start; drop passed points.
            path = np.asarray(planned)
            nearest = int(np.argmin(np.linalg.norm(path-current, axis=1)))
            routes.append(lookahead(current, path[nearest:], horizon))
        original_segments = sum(max(0,len(r)-1) for r in routes)
        simplified = [simplify_roi_route(r,min(.05,self.c['resolution']/4)) for r in routes]
        routes = [r for r,_ in simplified]
        route_error = max((e for _,e in simplified),default=0.)
        region = dict(centre=np.asarray(current).tolist(), near_radius=self.c['mapping_near_radius'],
            radius=self.c['mapping_corridor_radius']+route_error, routes=routes,
            route_simplification=dict(original_segments=original_segments,
                simplified_segments=sum(max(0,len(r)-1) for r in routes),
                enclosure_margin_m=route_error,policy='conservative_capsule_superset_not_flight_smoothing'),
            expansion_centres=[p.tolist() for p in self.centres], expansion_radius=self.radius,
            expansions=self.expansions, lookahead_m=horizon,
            policy='route_corridor_plus_360_near_field', outside_policy='unobserved_is_unknown')
        if hit_points is not None:
            points = np.asarray(hit_points, float).reshape(-1, 3)
            points = points[np.all(np.isfinite(points), axis=1)]
            # Select relevant measured surfaces before the endpoint ROI filter.
            # A selection halo never declares space free.
            halo = dict(region)
            dimensions = np.asarray(self.c['drone_dimensions'])
            margin = (max(np.linalg.norm(dimensions[:2]/2), dimensions[2]/2)+self.c['clearance']
                      +self.c.get('tracking_margin', 0.)+self.c['resolution'])
            halo['radius'] += margin
            halo['near_radius'] += margin
            candidates = points[contains(points, halo) & ~contains(points, region)]
            # The halo must not expand along the entire flat ground merely
            # because it lies below a 2 m high route. Promote side surfaces
            # within the swept body's vertical band. Ground/foliage INSIDE the
            # base ROI remain obstacles; no surface is reclassified as free.
            vertical_margin = (dimensions[2]/2+self.c['clearance']
                               +self.c.get('tracking_margin', 0.)+self.c['resolution']+route_error)
            useful = np.zeros(len(candidates), dtype=bool)
            delta = candidates-np.asarray(current)
            useful |= ((np.linalg.norm(delta[:,:2], axis=1) <= halo['near_radius'])
                       & (np.abs(delta[:,2]) <= vertical_margin))
            for route in region['routes']:
                for start, end in zip(route, route[1:]):
                    start, end = np.asarray(start), np.asarray(end)
                    direction = end[:2]-start[:2]
                    t = np.clip((candidates[:,:2]-start[:2])@direction/max(float(direction@direction),1e-12),0.,1.)
                    z = start[2]+t*(end[2]-start[2])
                    useful |= np.abs(candidates[:,2]-z) <= vertical_margin
            candidates = candidates[useful]
            added = 0
            for p in candidates:
                if not self.centres or min(np.linalg.norm(p-q) for q in self.centres) > 1.:
                    self.centres.append(p.copy())
                    added += 1
                    if added >= 4:
                        break
            if added:
                self.centres = self.centres[-16:]
                self.radius = min(self.c['mapping_corridor_max_radius'],
                                  max(self.radius, self.c['mapping_corridor_radius']
                                      +self.c['mapping_corridor_expand_step']))
                self.expansions += 1
                region.update(expansion_centres=[p.tolist() for p in self.centres],
                              expansion_radius=self.radius, expansions=self.expansions)
            region['surface_halo_candidates'] = len(candidates)
            region['surface_expansion_centres_added'] = added
            region['surface_halo_margin_m'] = margin
        return region
