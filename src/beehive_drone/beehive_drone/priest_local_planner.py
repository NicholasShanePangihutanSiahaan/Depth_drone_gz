"""Bounded CPU implementation of projection-guided trajectory sampling.

PRIEST-style adaptation (Rastgar et al., arXiv:2309.08235): Bernstein
coefficients, batched equality-constrained projection, residual-ranked elites,
and distribution updates. Occupancy projection uses a grid distance transform
instead of the paper's ellipsoids. This is not the upstream GPU implementation.
Every returned path is independently checked against the complete inflated grid.
"""
from dataclasses import dataclass
import math
import time

import numpy as np
from scipy.ndimage import distance_transform_edt


@dataclass
class PlanResult:
    path: list
    backend: str
    elapsed_ms: float


def resample(path, count):
    p = np.asarray(path, dtype=float)
    distance = np.r_[0., np.cumsum(np.linalg.norm(np.diff(p, axis=0), axis=1))]
    if distance[-1] < 1e-8:
        return np.repeat(p[:1], count, axis=0)
    t = np.linspace(0, distance[-1], count)
    return np.column_stack([np.interp(t, distance, p[:, axis]) for axis in range(2)])


def path_valid(grid, path):
    return (len(path) >= 2 and np.all(np.isfinite(path))
            and all(grid.segment_free(a, b) for a, b in zip(path, path[1:])))


class PriestLocalPlanner:
    def __init__(self, samples=24, iterations=3, projections=6, budget_ms=60., seed=7):
        if samples < 4 or iterations < 1 or projections < 1 or budget_ms <= 0:
            raise ValueError('invalid planner budget')
        self.samples, self.iterations, self.projections = samples, iterations, projections
        self.budget_ms = budget_ms
        self.rng = np.random.default_rng(seed)
        self.degree, self.steps = 9, 40
        t = np.linspace(0., 1., self.steps)
        n = self.degree
        self.basis = np.array([math.comb(n, i)*t**i*(1-t)**(n-i)
                               for i in range(n+1)]).T
        self.fit = np.linalg.pinv(self.basis)
        self.d1 = np.diff(self.basis, axis=0)*(self.steps-1)
        self.d2 = np.diff(self.d1, axis=0)*(self.steps-1)
        self.previous = None

    def plan(self, grid, seed_path, speed=.4, acceleration=.5):
        started = time.monotonic()
        if not path_valid(grid, seed_path):
            return PlanResult([], 'invalid_seed', 0.)
        reference = resample(seed_path, self.steps)
        length = np.linalg.norm(np.diff(reference, axis=0), axis=1).sum()
        # This is a geometric local smoother. Controller speed is independently
        # bounded; duration gives the projection physically reasonable derivatives.
        duration = max(3., 1.8*length/max(speed, .05))
        v, a = self.d1/duration, self.d2/duration**2
        f = np.vstack((self.basis, v, a))
        eq = self.basis[[0, -1]]
        h = np.eye(self.degree+1)*.03 + f.T@f
        kkt = np.block([[h, eq.T], [eq, np.zeros((2, 2))]])
        inverse = np.linalg.inv(kkt)
        _, nearest = distance_transform_edt(grid.blocked, return_indices=True)
        mean = self.fit@reference
        sigma = .5
        best, backend = list(seed_path), 'astar_fallback'
        best_cost = float('inf')
        for _ in range(self.iterations):
            if (time.monotonic()-started)*1000 > self.budget_ms:
                break
            coeff = mean + self.rng.normal(0, sigma, (self.samples, self.degree+1, 2))
            coeff[0] = mean
            if self.previous is not None:
                coeff[1] = self.fit@resample(self.previous, self.steps)
            original = coeff.copy()
            for _ in range(self.projections):
                pts = self.basis@coeff
                cells = np.rint((pts-grid.origin)/grid.resolution).astype(int)
                cells = np.clip(cells, 0, grid.size-1)
                target = grid.origin + np.stack((nearest[0, cells[..., 0], cells[..., 1]],
                                                nearest[1, cells[..., 0], cells[..., 1]]), -1)*grid.resolution
                occupied = grid.blocked[cells[..., 0], cells[..., 1]]
                target = np.where(occupied[..., None], target, pts)
                vel, acc = v@coeff, a@coeff
                vel *= np.minimum(1., speed/np.maximum(np.linalg.norm(vel, axis=-1), 1e-8))[..., None]
                acc *= np.minimum(1., acceleration/np.maximum(np.linalg.norm(acc, axis=-1), 1e-8))[..., None]
                rhs = .03*original + self.basis.T@target + v.T@vel + a.T@acc
                bounds = np.broadcast_to(reference[[0, -1]], (self.samples, 2, 2))
                coeff = (inverse@np.concatenate((rhs, bounds), axis=1))[:, :self.degree+1]
            pts = self.basis@coeff
            cells = np.rint((pts-grid.origin)/grid.resolution).astype(int)
            inside = np.all((cells >= 0) & (cells < grid.size), axis=-1)
            clipped = np.clip(cells, 0, grid.size-1)
            residual = np.sum(~inside | grid.blocked[clipped[..., 0], clipped[..., 1]], axis=1)
            cost = (np.sum(np.diff(pts, axis=1)**2, axis=(1, 2))
                    + .05*np.sum((pts-reference)**2, axis=(1, 2)) + 1000*residual)
            order = np.argsort(cost)
            for index in order[:4]:
                path = list(pts[index])
                if cost[index] < best_cost and path_valid(grid, path):
                    best, best_cost, backend = path, cost[index], 'priest_projection'
            elite = coeff[order[:max(3, self.samples//4)]]
            mean = .2*mean + .8*np.mean(elite, axis=0)
            sigma = max(.08, float(np.std(elite-mean)))
        self.previous = best
        return PlanResult(best, backend, (time.monotonic()-started)*1000)
