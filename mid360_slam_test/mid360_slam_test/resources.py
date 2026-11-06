"""Best-effort Linux temperature telemetry; unreadable is never zero Celsius."""
import math
from pathlib import Path


def read_temperatures(root=Path('/sys/class/thermal')):
    temperatures, errors = {}, {}
    for path in Path(root).glob('thermal_zone*/temp'):
        name = path.parent.name
        try:
            value = float(path.read_text()) / 1000.
            if not math.isfinite(value):
                raise ValueError('nonfinite temperature')
            temperatures[name] = value
        except (OSError, ValueError, TypeError, UnicodeError) as exc:
            # Some Jetson sysfs providers fail inside TextIOWrapper.read() with
            # TypeError (None returned by the underlying read), not OSError.
            errors[name] = dict(error_type=type(exc).__name__, message=str(exc))
    return temperatures, errors
