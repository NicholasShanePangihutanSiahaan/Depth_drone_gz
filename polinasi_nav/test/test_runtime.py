import pytest
from polinasi_nav.runtime import CallbackTiming, measured
from polinasi_nav.runtime import PipelineTiming, timed_job


def test_pipeline_total_is_correlated_not_sum_of_independent_maxima():
    timing = PipelineTiming()
    trace = dict(scan_stamp=42., **dict(zip(('received_ns', 'submitted_ns', 'worker_started_ns',
        'worker_finished_ns', 'integrated_ns', 'navigation_received_ns'),
        [1_000_000, 3_000_000, 4_000_000, 9_000_000, 11_000_000, 14_000_000])))
    timing.record(trace, 7, 21_000_000, .3)
    state = timing.status()
    assert state['last']['total_wall_ms'] == 20.
    assert sum(state['last']['stages_wall_ms'].values()) == 20.
    assert state['last']['measurement_to_publish_sim_seconds'] == .3
    timing.record(trace, 7, 100_000_000, 1.)
    assert timing.count == 1  # Never count repeated publication of the same map.
    timing.record({}, 8, 100_000_000, 1.)
    timing.record(trace, 8, 1, 1.)
    assert timing.count == 1  # Missing / reversed clocks do not invent a sample.


def test_worker_timing_keeps_result_and_exception():
    import time
    result = timed_job(lambda x: (x, 2.), time.monotonic_ns(), 7)
    assert result[:2] == (7, 2.)
    assert result[-1]['submitted_ns'] <= result[-1]['worker_started_ns'] <= result[-1]['worker_finished_ns']
    with pytest.raises(ZeroDivisionError):
        timed_job(lambda: 1/0, time.monotonic_ns())


def test_timing_keeps_stage_maximum_and_last_callback():
    timing = CallbackTiming()
    timing.record('control_tick', .005)
    timing.record('control_tick', .002)
    timing.record('visual_worker_local', .4)
    assert timing.max_ms['control_tick'] == 5.
    assert timing.last == dict(name='visual_worker_local', wall_ms=400.)


def test_measurement_preserves_result_and_records_on_error():
    class Owner:
        callback_timing = CallbackTiming()

        @measured('callback')
        def callback(self, fail=False):
            if fail:
                raise ValueError('original error')
            return 42
    owner = Owner()
    assert owner.callback() == 42
    with pytest.raises(ValueError, match='original error'):
        owner.callback(True)
    assert owner.callback_timing.last['name'] == 'callback'


def test_survey_planner_uses_spawned_process_not_gil_shared_thread():
    from concurrent.futures import ProcessPoolExecutor
    from polinasi_nav.config import load_config
    from polinasi_nav.mapping import VoxelMap
    from polinasi_nav.survey import SurveyController
    c = load_config()
    c['survey_waypoints'] = [[0., 0., 2.]]
    controller = SurveyController(c, VoxelMap(c))
    try:
        assert isinstance(controller.executor, ProcessPoolExecutor)
        assert controller.executor._mp_context.get_start_method() == 'spawn'
    finally:
        controller.close()
