import json
import numpy as np
import shutil
import subprocess
import pytest
from polinasi_nav.config import load_config
from polinasi_nav.mapping import VoxelMap, FREE, OCCUPIED
from polinasi_nav.map_log import MapLog


def test_point_filter_downsampling_and_storage_bound(tmp_path):
    log = MapLog(tmp_path, max_points=2)
    log.add_hits([[1, 2, 3], [1.01, 2, 3], [2, 2, 3], [3, 2, 3], [np.inf, 1, 1]])
    assert len(log.points) == 2
    assert log.truncated
    np.testing.assert_allclose(list(log.points.values())[0], [1.01, 2, 3])


def test_snapshot_contains_every_voxel_without_modifying_map(tmp_path):
    grid = VoxelMap(load_config())
    grid.state[1, 2, 3] = OCCUPIED
    grid.state[2, 3, 4] = FREE
    before = grid.state.copy()
    log = MapLog(tmp_path)
    log.add_hits([[1, 2, 3]])
    data = log.snapshot(grid, 'ground_truth', 12.)
    assert data['occupied'] == [[1, 2, 3]]
    assert data['free'] == [[2, 3, 4]]
    assert sum(len(data[k]) for k in ('occupied', 'free', 'unknown')) == grid.state.size
    assert data['map_observation_time'] is None
    assert data['frame'] == 'map'
    np.testing.assert_array_equal(before, grid.state)
    # Later point/map updates cannot change the independent export snapshot.
    log.add_hits([[1.01, 2, 3]])
    grid.state[1, 2, 3] = FREE
    assert data['points'] == [[1., 2., 3.]]
    assert data['occupied'] == [[1, 2, 3]]


def test_offline_files_are_complete_and_replace_previous_snapshot(tmp_path):
    log = MapLog(tmp_path)
    grid = VoxelMap(load_config())
    data = log.snapshot(grid, 'sensor', 5.)
    log.write(data)
    assert json.loads((tmp_path / 'map.json').read_text()) == data
    html = (tmp_path / 'map.html').read_text()
    assert '__MAP_DATA__' not in html
    assert '<canvas' in html and 'const data=' in html
    assert 'https://' not in html  # Works offline: no external library/CDN.
    data['simulation_time'] = 10.
    log.write(data)
    assert json.loads((tmp_path / 'map.json').read_text())['simulation_time'] == 10.
    assert not list(tmp_path.glob('*.tmp'))


@pytest.mark.parametrize('with_paths', [False, True])
def test_viewer_javascript_draws_and_handles_controls(tmp_path, with_paths):
    if not shutil.which('node'):
        pytest.skip('Node.js unavailable for viewer smoke test')
    log = MapLog(tmp_path)
    grid = VoxelMap(load_config())
    grid.state[0, 0, 0] = OCCUPIED
    grid.state[1, 1, 1] = FREE
    log.add_hits([[1, 2, 3]])
    data = log.snapshot(grid, 'ground_truth', 5.)
    if with_paths:
        log.add_pose([0., 0., 1.], 1.)
        log.add_pose([1., 0., 1.], 1.3)
        log.add_pose([2., 0., 1.], 3.)  # Missing data: do not connect this edge.
        data['flight_paths'] = log.flight_paths([[0, 0, 1], [1, 1, 1]], [[0, 0, 1], [0, 1, 1]])
    log.write(data)
    script = (tmp_path / 'map.html').read_text().split('<script>')[1].split('</script>')[0]
    stub = '''
    const context=new Proxy({}, {get:(o,k)=>o[k] || ((...args)=>{
      if(['moveTo','lineTo','fillRect'].includes(k) && args.some(v=>!Number.isFinite(v)))
        throw new Error('Non-finite drawing coordinate');
    }), set:(o,k,v)=>(o[k]=v,true)});
    const elements={};const listeners={};const routeColours=[];
    context.stroke=()=>routeColours.push(context.strokeStyle);
    globalThis.devicePixelRatio=1;globalThis.window={};
    globalThis.document={getElementById:id=>elements[id] ||= {
      checked:true,clientWidth:900,clientHeight:600,getContext:()=>context,
      setPointerCapture:()=>{},addEventListener:(name,fn)=>listeners[name]=fn
    }};
    '''
    interactions = '''
    canvas.onpointerdown({clientX:1,clientY:2,pointerId:1});
    canvas.onpointermove({clientX:20,clientY:30});canvas.onpointerup();
    listeners.wheel({deltaY:10,preventDefault:()=>{}});
    document.getElementById('height').value=1;
    document.getElementById('height').oninput();
    document.getElementById('points').checked=false;draw();
    document.getElementById('reset').onclick();window.onresize();
    routeColours.length=0;
    for(const id of ['nominalRoute','plannedPath','executedPath'])document.getElementById(id).checked=false;
    document.getElementById('executedPath').oninput();
    if(routeColours.some(c=>['#ffd166','#539dff','#67ef95'].includes(c)))throw new Error('Unchecked route drawn');
    '''
    if with_paths:
        interactions += '''
        routeColours.length=0;document.getElementById('executedPath').checked=true;
        document.getElementById('executedPath').oninput();
        if(routeColours.filter(c=>c==='#67ef95').length!==1)throw new Error('Executed path gap not respected');
        routeColours.length=0;document.getElementById('nominalRoute').checked=true;
        document.getElementById('nominalRoute').oninput();
        if(!routeColours.includes('#ffd166'))throw new Error('Nominal route missing');
        routeColours.length=0;document.getElementById('plannedPath').checked=true;
        document.getElementById('plannedPath').oninput();
        if(!routeColours.includes('#539dff'))throw new Error('Planned route missing');
        '''
    else:
        interactions += '''
        if(!document.getElementById('executedPath').disabled)throw new Error('Old log invented a path');
        '''
    result = subprocess.run(['node'], input=stub+script+interactions, text=True, capture_output=True)
    assert result.returncode == 0, result.stderr


def test_executed_history_is_independent_finite_bounded_and_keeps_start(tmp_path):
    log = MapLog(tmp_path)
    log.path_limit = 4
    for i in range(10):
        log.add_pose([i, 0., 2.], float(i))
    log.add_pose([np.nan, 0., 2.], 12.)
    log.add_pose([1., 0., 2.], 1.)  # Unordered input.
    log.add_pose([20., 0., 2.], 20.)  # Beyond the adaptively enlarged sample period.
    paths = log.flight_paths([[0, 0, 2], [10, 0, 2]])
    assert len(paths['executed_path']) <= 4 and paths['executed_decimated']
    assert paths['executed_path'][0] == [0., 0., 2.]
    assert paths['executed_path'][-1] == [20., 0., 2.]
    before = paths['executed_path'][:]
    log.add_pose([30., 0., 2.], 30.)
    assert paths['executed_path'] == before


def test_export_worker_saves_paths_with_original_geometry(tmp_path):
    from polinasi_nav.map_log import write_snapshot
    log = MapLog(tmp_path)
    log.add_pose([0., 0., 1.], 1.)
    log.add_pose([1., 0., 1.], 1.3)
    paths = log.flight_paths([[0, 0, 1], [1, 1, 1]])
    grid = VoxelMap(load_config())
    write_snapshot(tmp_path, grid, 'ground_truth', 5., [], False, 'mapping', {}, paths)
    data = json.loads((tmp_path/'map.json').read_text())
    assert data['flight_paths'] == paths
    assert sum(len(data[k]) for k in ('occupied', 'free', 'unknown')) == grid.state.size


def test_refresh_old_viewer_preserves_log_and_never_invents_actual_path(tmp_path):
    import importlib.util
    from pathlib import Path
    script = Path(__file__).resolve().parents[2]/'tools/refresh_map_viewer.py'
    spec = importlib.util.spec_from_file_location('refresh_map_viewer', script)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    log = MapLog(tmp_path)
    log.write(log.snapshot(VoxelMap(load_config()), 'ground_truth', 5.))
    source, original = tmp_path/'map.json', tmp_path/'map.html'
    before_json, before_html = source.read_bytes(), original.read_bytes()
    config = tmp_path/'config.json'
    config.write_text(json.dumps(dict(home=[0, 0, 2], survey_waypoints=[[1, 0, 2]])))
    output = tmp_path/'map_paths.html'
    module.refresh(source, output, config)
    html = output.read_text()
    data = json.loads(html.split('const data=')[1].split(';\n')[0])
    assert data['flight_paths']['nominal_route'] == [[0, 0, 2], [1, 0, 2]]
    assert data['flight_paths']['executed_path'] == []
    assert source.read_bytes() == before_json and original.read_bytes() == before_html
    with pytest.raises(ValueError):
        module.refresh(source, original)


def test_mission_status_is_saved_with_the_map(tmp_path):
    log = MapLog(tmp_path)
    grid = VoxelMap(load_config())
    data = log.snapshot(grid, 'ground_truth', 5.)
    data.update(mission_kind='mapping', mission_status={'mission': 'SURVEY', 'armed': True, 'survey_waypoint': 1})
    log.write(data)
    assert json.loads((tmp_path/'mission_status.json').read_text()) == data['mission_status']
