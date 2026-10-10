"""Guarded shadow RLS for an effective PVA closed-loop model, NOT mass/PID tuning.

dv/dt = kp*(p_cmd-p) + kv*(v_cmd-v) + ka*a_cmd + bias.
Valid only in the identified envelope and for the same setpoint interface.
"""
from collections import deque
import numpy as np
import json
from pathlib import Path

INITIAL = np.array([3., 3., 1., 0.])
LOWER = np.array([.05, .1, 0., -1.])
UPPER = np.array([40., 20., 3., 1.])


def load_validated_seed(filename, config):
    """Optional simulation observer seed, never permission for flight control."""
    if not filename or not Path(filename).exists():
        return [INITIAL.copy() for _ in range(3)], [0., 0., 0.], 'nominal'
    report = json.loads(Path(filename).read_text())
    if not report.get('validation_passed') or report.get('frame') != 'map_ENU' or report.get('interface') != 'PVA_ENU':
        raise ValueError('adaptive seed must be an independently validated PVA model')
    envelope = report['identification_envelope']
    for key, value in envelope.items():
        if config.get(key) != value:
            raise ValueError(f'adaptive seed envelope mismatch: {key}')
    parameters, delays = [], []
    for axis in 'xyz':
        model = report['axes'][axis]
        theta = np.asarray(model['parameters'], float)
        delay = float(model['delay_seconds'])
        if (not model.get('validation_passed') or theta.shape != (4,) or not stable(theta)
                or not np.isfinite(delay) or not 0 <= delay <= .25):
            raise ValueError(f'invalid adaptive seed: {axis}')
        parameters.append(theta)
        delays.append(delay)
    return parameters, delays, str(filename)


def stable(theta, dt=.05):
    if not np.all(np.isfinite(theta)) or np.any(theta < LOWER) or np.any(theta > UPPER):
        return False
    kp, kv = theta[:2]
    matrix = np.array([[1., dt], [-kp*dt, 1-kv*dt]])
    return bool(np.max(np.abs(np.linalg.eigvals(matrix))) < 1.)


class GuardedRLS:
    def __init__(self, initial=None, forgetting=.995, window=120):
        self.theta = np.array(INITIAL if initial is None else initial, dtype=float)
        if self.theta.shape != (4,) or not stable(self.theta) or not .98 <= forgetting <= 1.:
            raise ValueError('invalid adaptive model seed/forgetting factor')
        self.P = np.eye(4)*20.
        self.forgetting = forgetting
        self.rows = deque(maxlen=window)
        self.accepted = self.rejected = 0
        self.reason, self.condition = 'warming_up', None

    def update(self, phi, measured_acceleration, dt, healthy=True):
        phi = np.asarray(phi, dtype=float)
        if (not healthy or phi.shape != (4,) or not np.all(np.isfinite(phi)) or
                not np.isfinite(measured_acceleration) or not .015 <= dt <= .08):
            self.reason = 'invalid_or_stale_data'
            self.rejected += 1
            return False
        if abs(measured_acceleration) > 2.:
            self.reason = 'outlier'
            self.rejected += 1
            return False
        self.rows.append(phi.copy())
        if len(self.rows) < self.rows.maxlen:
            self.reason = 'warming_up'
            return False
        rows = np.asarray(self.rows)
        scale = np.sqrt(np.mean(rows*rows, axis=0))
        if np.any(scale[:3] < 1e-4):
            self.reason = 'insufficient_excitation'
            return False  # No forgetting/covariance growth while hovering.
        singular = np.linalg.svd(rows/scale, compute_uv=False)
        self.condition = float(singular[0]/max(singular[-1], 1e-12))
        if self.condition > 500.:
            self.reason = 'correlated_inputs'
            return False
        error = float(measured_acceleration-phi@self.theta)
        if abs(error) > 1.5:
            self.reason = 'innovation_outlier'
            self.rejected += 1
            return False
        gain = self.P@phi/(self.forgetting+phi@self.P@phi)
        # Rate-limit model changes; never abrupt jumps in an online consumer.
        proposed = self.theta+np.clip(gain*error, -np.array([2., 2., .5, .2])*dt,
                                    np.array([2., 2., .5, .2])*dt)
        proposed = np.clip(proposed, LOWER, UPPER)
        covariance = (self.P-np.outer(gain, phi)@self.P)/self.forgetting
        covariance = (covariance+covariance.T)/2
        if not stable(proposed) or np.linalg.eigvalsh(covariance).min() <= 0:
            self.reason = 'unstable_candidate'
            self.rejected += 1
            return False
        eigenvalues, eigenvectors = np.linalg.eigh(covariance)
        self.P = (eigenvectors*np.clip(eigenvalues, 1e-6, 100.))@eigenvectors.T
        self.theta, self.reason = proposed, 'updated_shadow'
        self.accepted += 1
        return True

    def snapshot(self):
        return dict(parameters=self.theta.tolist(), accepted=self.accepted, rejected=self.rejected,
                    reason=self.reason, regressor_condition=self.condition,
                    covariance_diagonal=np.diag(self.P).tolist(), controls_flight=False)
