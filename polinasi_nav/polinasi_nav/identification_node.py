"""Reuse normal takeoff/alignment/landing and the sole PVA MAVROS publisher."""
from .ros_nodes import NavigationNode, run
from .identification import IdentificationController, empty_arena_fixture


class IdentificationNode(NavigationNode):
    def __init__(self):
        super().__init__('mapping', identification_empty_arena=True)
        if not self.separate_mapping or self.get_parameter('mode').value != 'ground_truth':
            raise RuntimeError('Identification requires isolated mapping and ground_truth simulation')
        self.controller.close()
        self.grid = empty_arena_fixture(self.c)
        self.controller = IdentificationController(self.c, self.grid)
        self.last_phase = None

    def extra_status(self):
        phase = self.controller.identification_phase
        if phase != self.last_phase:
            self.get_logger().info(f'IDENTIFICATION: {phase}')
            self.last_phase = phase
        return dict(mission_kind='identification', identification_profile=self.controller.profile,
                    identification_phase=phase, protocol_completed=self.controller.completed_protocol,
                    adaptive_model_mode=('active_bounded' if self.c.get('predictive_adaptation') else
                        'fixed_identified_model') if self.predictive is not None else 'shadow_only',
                    model_controls_flight=False,
                    identification_empty_arena=True, lidar_required=False,
                    map_source='known_empty_simulation_arena_not_lidar',
                    takeoff_space_observed=False, takeoff_space_known=True,
                    map_measurement_age=None, cloud_age=None)


def main(args=None):
    run(IdentificationNode, args)
