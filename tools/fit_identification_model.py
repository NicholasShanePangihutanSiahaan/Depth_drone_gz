"""Fit a candidate model; never activates it or writes autopilot parameters."""
import argparse
import json
import csv
from pathlib import Path
from polinasi_nav.model_fitting import fit, validate, rollout_trace


def comparison_plots(dataset, models, directory):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    directory.mkdir(parents=True, exist_ok=True)
    figure, axes = plt.subplots(3, 2, figsize=(12, 9))
    for axis, name in enumerate('xyz'):
        item = models[name]
        trace = rollout_trace(dataset, axis, item['parameters'], item['delay_seconds'])
        with (directory/f'validation_{name}.csv').open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(trace[0]))
            writer.writeheader()
            writer.writerows(trace)
        times = [row['t']-trace[0]['t'] for row in trace]
        for column, quantity in enumerate(('position', 'velocity')):
            plot = axes[axis, column]
            for source, label in (('command', 'PVA command (delayed)'),
                                  ('measured', 'Gazebo response'), ('predicted', 'model: 2 s rollout')):
                plot.plot(times, [row[f'{source}_{quantity}'] for row in trace], label=label, linewidth=1.)
            plot.set_ylabel(f'{name}: {quantity} ({"m" if column == 0 else "m/s"})')
            plot.set_xlabel('validation elapsed simulation seconds')
            plot.grid(True, alpha=.3)
    axes[0, 0].legend(fontsize=8)
    figure.suptitle('Ground-truth Gazebo/ArduPilot: validation, not hardware or LiDAR localisation')
    figure.tight_layout()
    figure.savefig(directory/'validation_comparison.png', dpi=140)
    plt.close(figure)


def fixed_parameters(directory):
    before = json.loads((directory/'parameters_before.json').read_text())['parameters']
    after = json.loads((directory/'parameters_after.json').read_text())['parameters']
    names = [name for name in before if name.startswith(('ATC_', 'PSC_', 'WPNAV_', 'MOT_'))]
    changed = [name for name in names if before[name] != after.get(name)]
    if changed:
        raise ValueError(f'controller parameters changed: {changed}')
    if before.get('MOT_HOVER_LEARN') != 0:
        raise ValueError('hover learning was not disabled for identification')
    return {name: before[name] for name in names}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--train', required=True, type=Path)
    parser.add_argument('--validation', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--install-shadow-seed', type=Path,
                        help='Save validated coefficients for the simulation observer only')
    args = parser.parse_args()
    if args.output.exists():
        raise ValueError('refusing to overwrite model report')
    if args.install_shadow_seed and args.install_shadow_seed.exists():
        raise ValueError('refusing to overwrite an installed model seed')
    training = json.loads(args.train.read_text())
    validation = json.loads(args.validation.read_text())
    parameters = fixed_parameters(args.train.parent)
    if parameters != fixed_parameters(args.validation.parent):
        raise ValueError('train and validation autopilot parameters differ')
    models = validate(training, validation, fit(training))
    report = dict(equation='p_dot=v; v_dot=kp*(p_cmd_delayed-p)+kv*(v_cmd_delayed-v)+ka*a_cmd_delayed+bias',
        parameter_order=['kp', 'kv', 'ka', 'bias'], frame='map_ENU', interface='PVA_ENU',
        axes=models, validation_passed=all(item['validation_passed'] for item in models.values()),
        controller_parameters=parameters, training_dataset=str(args.train), validation_dataset=str(args.validation),
        activated=False, hardware_validated=False, estimated_mass_kg=None,
        identification_envelope={key: training['config'][key] for key in (
            'command_speed', 'speed', 'identification_setpoint_interface', 'identification_payload_kg')},
        note='Effective local closed-loop model; finite-difference least squares can be biased by correlated '
             'measurement noise. Not a certified physical plant model or controller stability proof.')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2)+'\n')
    comparison_plots(validation, models, args.output.parent/(args.output.stem+'_comparison'))
    if args.install_shadow_seed:
        if report['validation_passed']:
            args.install_shadow_seed.parent.mkdir(parents=True, exist_ok=True)
            args.install_shadow_seed.write_text(json.dumps(report, indent=2)+'\n')
            print(f'Validated simulation shadow seed saved: {args.install_shadow_seed}')
        else:
            print('Validation rejected; shadow seed not installed.')
    print(json.dumps(dict(validation_passed=report['validation_passed'], axes=models), indent=2))
    print(f'Candidate only, not activated: {args.output}')


if __name__ == '__main__':
    main()
