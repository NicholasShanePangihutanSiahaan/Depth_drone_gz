"""Offline effective-model fitting and independent free-running validation."""
import numpy as np
from scipy.optimize import lsq_linear
from .adaptive_model import INITIAL, LOWER, UPPER, stable


def samples(dataset, delay=0.):
    commands = dataset['commands']
    times = np.array([row['t'] for row in commands])
    result, previous = [], None
    for row in dataset['responses']:
        if not row['healthy'] or row.get('phase') in ('LAND', 'WAIT', 'WAIT_TAKEOFF'):
            previous = None
            continue
        if previous is None:
            previous = row
            continue
        dt = row['t']-previous['t']
        if dt < .045:
            continue
        index = np.searchsorted(times, previous['t']-delay, side='right')-1
        if .015 <= dt <= .08 and index >= 0 and previous['t']-delay-times[index] <= .12:
            result.append((previous, row, commands[index], dt))
        previous = row
    return result


def design(rows, axis):
    matrix, target = [], []
    for old, new, command, dt in rows:
        matrix.append([command['p'][axis]-old['p'][axis], command['v'][axis]-old['v'][axis], command['a'][axis], 1.])
        target.append((new['v'][axis]-old['v'][axis])/dt)
    return np.asarray(matrix), np.asarray(target)


def fit(dataset):
    if (not dataset.get('completed') or not dataset.get('one_final_publisher')
            or dataset.get('model_controls_flight')):
        raise ValueError('fit requires a completed, single-producer flight')
    output = {}
    for axis, name in enumerate('xyz'):
        best = None
        for delay in np.arange(0., .251, .025):
            matrix, target = design(samples(dataset, float(delay)), axis)
            if len(matrix) < 100:
                continue
            scale = np.maximum(np.sqrt(np.mean(matrix*matrix, axis=0)), 1e-8)
            singular = np.linalg.svd(matrix/scale, compute_uv=False)
            condition = float(singular[0]/max(singular[-1], 1e-12))
            solution = lsq_linear(matrix, target, bounds=(LOWER, UPPER))
            theta = solution.x
            error = float(np.sqrt(np.mean((matrix@theta-target)**2)))
            if not stable(theta):
                continue
            # A predictive model must perform beyond one differentiated sample.
            # Select delays using TRAIN rollouts only; validation is never used
            # for fitting/selection. Retain acceleration residual as a diagnostic.
            metrics = rollout_metrics(dataset, axis, theta, float(delay))
            score = float(np.hypot(metrics['position_rmse_m'], .25*metrics['velocity_rmse_mps']))
            if best is None or score < best['training_rollout_score_m']:
                best = dict(parameters=theta.tolist(), delay_seconds=float(delay),
                    training_acceleration_rmse_mps2=error, samples=len(matrix), regressor_condition=condition,
                    training_rollout=metrics, training_rollout_score_m=score,
                    identifiable=bool(condition < 500. and np.min(scale[:3]) > 1e-4),
                    at_parameter_bound=bool(np.any(np.isclose(theta, LOWER, atol=.01)) or np.any(np.isclose(theta, UPPER, atol=.01))))
        if best is None:
            raise ValueError(f'no stable fitted model for axis {name}')
        output[name] = best
    return output


def rollout_trace(dataset, axis, theta, delay, horizon=2.):
    rows = samples(dataset, delay)
    trace = []
    p = v = started = last = None
    for old, new, command, dt in rows:
        if started is None or old['t']-started >= horizon or (last is not None and old['t']-last > .1):
            p, v, started = old['p'][axis], old['v'][axis], old['t']
        kp, kv, ka, bias = theta
        acceleration = kp*(command['p'][axis]-p)+kv*(command['v'][axis]-v)+ka*command['a'][axis]+bias
        p, v = p+dt*v, v+dt*acceleration
        trace.append(dict(t=new['t'], block_start=started,
            command_position=command['p'][axis], command_velocity=command['v'][axis],
            measured_position=new['p'][axis], measured_velocity=new['v'][axis],
            predicted_position=float(p), predicted_velocity=float(v)))
        last = new['t']
    return trace


def rollout_metrics(dataset, axis, theta, delay, horizon=2.):
    trace = rollout_trace(dataset, axis, theta, delay, horizon)
    errors_p = [r['predicted_position']-r['measured_position'] for r in trace]
    errors_v = [r['predicted_velocity']-r['measured_velocity'] for r in trace]
    if not errors_p:
        raise ValueError('no healthy validation samples')
    return dict(position_rmse_m=float(np.sqrt(np.mean(np.square(errors_p)))),
        position_max_error_m=float(np.max(np.abs(errors_p))),
        velocity_rmse_mps=float(np.sqrt(np.mean(np.square(errors_v)))),
        rollout_horizon_seconds=horizon, samples=len(errors_p))


def validate(training, validation, models):
    if (not validation.get('completed') or not validation.get('one_final_publisher')
            or training['config']['identification_profile'] != 'train'
            or validation['config']['identification_profile'] != 'validation'):
        raise ValueError('requires separate completed train and validation protocols')
    for key in ('resolution', 'command_speed', 'speed', 'identification_setpoint_interface', 'identification_payload_kg'):
        if training['config'][key] != validation['config'][key]:
            raise ValueError(f'train/validation mismatch: {key}')
    for axis, name in enumerate('xyz'):
        item = models[name]
        item['validation'] = rollout_metrics(validation, axis, item['parameters'], item['delay_seconds'])
        item['baseline_validation'] = rollout_metrics(validation, axis, INITIAL, 0.)
        item['validation_passed'] = bool(item['identifiable'] and not item['at_parameter_bound']
            and item['validation']['position_max_error_m'] < .05
            and item['validation']['velocity_rmse_mps'] < .05
            and item['validation']['position_rmse_m'] <= item['baseline_validation']['position_rmse_m']+1e-4)
    return models
