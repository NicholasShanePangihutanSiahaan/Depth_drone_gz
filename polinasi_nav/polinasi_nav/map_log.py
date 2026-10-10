"""Offline map snapshots, not a sensor replay or a navigation input."""
import json
import os
from pathlib import Path
import numpy as np


def write_snapshot(directory, grid, mode, stamp, points, truncated, mission_kind, status, flight_paths=None):
    """Picklable file-export entry point, with no ROS/control dependencies."""
    log = MapLog(directory)
    data = log.snapshot(grid, mode, stamp, points, truncated)
    data.update(mission_kind=mission_kind, mission_status=status)
    if flight_paths is not None:
        data['flight_paths'] = flight_paths
    log.write(data)


class MapLog:
    def __init__(self, directory, point_resolution=0.1, max_points=100000):
        self.directory = Path(directory)
        self.point_resolution = point_resolution
        self.max_points = max_points
        self.points = {}
        self.truncated = False
        self.executed_path, self.executed_times, self.executed_segments = [], [], []
        self.path_period, self.path_limit = .2, 20000
        self.path_decimated = False
        self.debug_stamp = -float('inf')
        self.debug_event = None

    def record_debug(self, status):
        """Small decision trace independent of large point-cloud exports."""
        stamp = status.get('simulation_time')
        if stamp is None:
            return
        event = (status.get('mission'), status.get('failure'), status.get('survey_waypoint'),
                 status.get('route_blockage', {}).get('reason'), status.get('adaptive_reason'))
        if stamp-self.debug_stamp < .5 and event == self.debug_event:
            return
        self.debug_stamp, self.debug_event = stamp, event
        self.directory.mkdir(parents=True, exist_ok=True)
        with (self.directory/'navigation_debug.jsonl').open('a', encoding='utf8') as stream:
            stream.write(json.dumps(status, allow_nan=False)+'\n')

    def add_pose(self, position, stamp):
        """Measured map-frame positions only; preserve gaps and both endpoints."""
        position = np.asarray(position, dtype=float)
        if position.shape != (3,) or not np.all(np.isfinite(position)) or not np.isfinite(stamp):
            return
        if self.executed_times and stamp-self.executed_times[-1] < self.path_period:
            return
        segment = self.executed_segments[-1] if self.executed_segments else 0
        if self.executed_times and stamp-self.executed_times[-1] > max(.5, 2.5*self.path_period):
            segment += 1  # Do not draw through a localisation/transform gap.
        self.executed_path.append(position.tolist())
        self.executed_times.append(float(stamp))
        self.executed_segments.append(segment)
        if len(self.executed_path) > self.path_limit:
            keep = list(range(0, len(self.executed_path), 2))
            if keep[-1] != len(self.executed_path)-1:
                keep.append(len(self.executed_path)-1)
            for name in ('executed_path', 'executed_times', 'executed_segments'):
                values = getattr(self, name)
                setattr(self, name, [values[i] for i in keep])
            self.path_period *= 2
            self.path_decimated = True

    def flight_paths(self, nominal=(), planned=()):
        return dict(frame='map', nominal_route=np.asarray(nominal).reshape(-1, 3).tolist(),
                    planned_path=np.asarray(planned).reshape(-1, 3).tolist(),
                    executed_path=[p[:] for p in self.executed_path],
                    executed_times=self.executed_times[:], executed_segments=self.executed_segments[:],
                    executed_decimated=self.path_decimated,
                    sample_period_seconds=self.path_period,
                    note='Nominal route is not a certified path. Planned path is the current command plan only. '
                         'Executed path uses localisation observations, not target positions or reconstructed truth.')

    def add_hits(self, points):
        """Receive only filtered finite surface hits already transformed to map."""
        points = np.asarray(points, dtype=float).reshape(-1, 3)
        points = points[np.all(np.isfinite(points), axis=1)]
        keys = np.floor(points / self.point_resolution).astype(np.int64)
        for key, point in zip(keys, points):
            key = tuple(key)
            if key in self.points or len(self.points) < self.max_points:
                self.points[key] = point.tolist()
            else:
                self.truncated = True

    def snapshot(self, grid, mode, stamp, points=None, point_limit_reached=None):
        # Fresh independent data: file writing can happen on another thread.
        return {'schema': 1, 'frame': 'map', 'units': 'metres',
                'localisation_mode': mode, 'simulation_time': float(stamp),
                'map_observation_time': (float(grid.last_observation_stamp)
                                         if np.isfinite(grid.last_observation_stamp) else None),
                'resolution': float(grid.res), 'origin': grid.lo.tolist(),
                'shape': [int(n) for n in grid.shape], 'point_resolution': self.point_resolution,
                'point_limit_reached': self.truncated if point_limit_reached is None else point_limit_reached,
                'points': list(self.points.values()) if points is None else points,
                'occupied': np.argwhere(grid.state == 1).tolist(),
                'free': np.argwhere(grid.state == 0).tolist(),
                'unknown': np.argwhere(grid.state == -1).tolist(),
                'note': 'Accumulated downsampled surface hits; current uninflated occupancy. '
                        'Free cells may include the surveyed launch-pad prior. '
                        'Unknown is NOT empty. This is not localisation validation.'}

    def write(self, data):
        self.directory.mkdir(parents=True, exist_ok=True)
        payload = json.dumps(data, separators=(',', ':'), allow_nan=False)
        outputs = [('map.json', payload), ('map.html', VIEWER.replace('__MAP_DATA__', payload))]
        if 'mission_status' in data:
            outputs.append(('mission_status.json', json.dumps(data['mission_status'], indent=2, allow_nan=False)))
        for name, content in outputs:
            target = self.directory / name
            temporary = target.with_suffix(target.suffix + '.tmp')
            temporary.write_text(content, encoding='utf-8')
            os.replace(temporary, target)


VIEWER = r'''<!doctype html>
<html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width">
<title>Polinasi — point cloud and voxel map</title>
<style>
body{margin:0;background:#101721;color:#e4edf5;font:14px system-ui}
header{padding:12px 18px;background:#192330} label{margin-right:16px;white-space:nowrap}
canvas{display:block;width:100%;height:75vh;touch-action:none;cursor:grab}
p{margin:8px 0}button{padding:5px 12px} #details{font-size:12px;color:#aebdce}
</style>
<header><strong>LiDAR point cloud + 3D voxel grid</strong>
<p>Drag to rotate · Scroll to zoom · All coordinates in metres (map frame: X/Y horizontal, Z up).</p>
<p><label><input id="points" type="checkbox" checked> Cyan: surface points</label>
<label><input id="occupied" type="checkbox" checked> Red: occupied</label>
<label><input id="free" type="checkbox"> Green: observed free</label>
<label><input id="unknown" type="checkbox"> Grey: unknown</label>
<button id="reset">Reset view</button></p>
<p><label><input id="nominalRoute" type="checkbox"> Kuning: rute rencana (bukan jaminan aman)</label>
<label><input id="plannedPath" type="checkbox"> Biru: lintasan perintah terakhir</label>
<label><input id="executedPath" type="checkbox" checked> Hijau: jalur aktual dari lokalisasi</label></p>
<p id="pathDetails"></p>
<label>Voxel height slice <input id="height" type="range" step="0.1"> <span id="heightText"></span></label>
<p id="details"></p></header><canvas id="view"></canvas>
<script>
const data=__MAP_DATA__;
const paths=data.flight_paths || {};
const pathLayers=[['nominalRoute','nominal_route','#ffd166',true],
 ['plannedPath','planned_path','#539dff',false],['executedPath','executed_path','#67ef95',false]];
for(const [id,key] of pathLayers){const box=document.getElementById(id);
 box.disabled=!(paths[key] && paths[key].length>1);if(box.disabled)box.checked=false;}
document.getElementById('pathDetails').textContent=(paths.executed_path && paths.executed_path.length>1
 ?'Jalur aktual: posisi hasil lokalisasi, bukan target. Celah data tidak dihubungkan.'
 :'Jalur aktual belum tersimpan pada log ini; jalur tidak dibuat-buat.')+
 (paths.executed_decimated?' Jalur disampel ulang untuk membatasi memori.':'');
const canvas=document.getElementById('view'),ctx=canvas.getContext('2d');
const centre=data.origin.map((v,i)=>v+data.shape[i]*data.resolution/2);
let az=-0.7,el=0.6,zoom=1,drag=null;
const height=document.getElementById('height');
height.min=data.origin[2]; height.max=data.origin[2]+data.shape[2]*data.resolution; height.value=height.max;
document.getElementById('details').textContent=`Mode: ${data.localisation_mode} · voxel edge: ${(data.resolution*100).toFixed(0)} cm · snapshot: ${data.simulation_time.toFixed(2)} s · ${data.points.length} downsampled points · ${data.occupied.length} occupied / ${data.free.length} free / ${data.unknown_count ?? data.unknown.length} unknown voxels. ${data.note} `+
 (data.point_limit_reached?'Point storage limit reached. ':'')+'For performance, each visible voxel layer displays at most 6000 cells; map.json retains all known cells (unlisted cells are unknown in format 2). Open/reload this file for the latest saved snapshot.';
const corners=[[-1,-1,-1],[1,-1,-1],[1,1,-1],[-1,1,-1],[-1,-1,1],[1,-1,1],[1,1,1],[-1,1,1]];
const edges=[[0,1],[1,2],[2,3],[3,0],[4,5],[5,6],[6,7],[7,4],[0,4],[1,5],[2,6],[3,7]];
function project(p){
 const [x,y,z]=p.map((v,i)=>v-centre[i]);
 const u=Math.cos(az)*x-Math.sin(az)*y, v=Math.sin(az)*x+Math.cos(az)*y;
 const depth=Math.cos(el)*v+Math.sin(el)*z;
 const span=Math.max(...data.shape.map(n=>n*data.resolution));
 const scale=Math.min(canvas.width,canvas.height)*0.75/span*zoom;
 return [canvas.width/2+u*scale,canvas.height/2-(Math.cos(el)*z-Math.sin(el)*v)*scale,depth];
}
function line(a,b,color){const p=project(a),q=project(b);ctx.strokeStyle=color;ctx.beginPath();ctx.moveTo(p[0],p[1]);ctx.lineTo(q[0],q[1]);ctx.stroke();}
function draw(){
 canvas.width=Math.round(canvas.clientWidth*devicePixelRatio); canvas.height=Math.round(canvas.clientHeight*devicePixelRatio);
 ctx.clearRect(0,0,canvas.width,canvas.height);ctx.lineWidth=devicePixelRatio;
 document.getElementById('heightText').textContent=Number(height.value).toFixed(1)+' m (voxels at/below)';
 const lo=data.origin, hi=lo.map((v,i)=>v+data.shape[i]*data.resolution);
 const boundary=corners.map(c=>c.map((v,i)=>v<0?lo[i]:hi[i]));
 for(const [a,b] of edges)line(boundary[a],boundary[b],'#3d5064');
 for(const [axis,color] of [[0,'#ff6d6d'],[1,'#70ef99'],[2,'#73aaff']]){
  const end=lo.slice();end[axis]+=2;line(lo,end,color);const p=project(end);ctx.fillStyle=color;ctx.font=`${14*devicePixelRatio}px system-ui`;ctx.fillText('XYZ'[axis],p[0],p[1]);
 }
 for(const [name,color] of [['unknown','#657181'],['free','#43b97b'],['occupied','#ff604c']]){
  if(!document.getElementById(name).checked)continue;
  const ids=data[name].filter(id=>lo[2]+(id[2]+0.5)*data.resolution<=Number(height.value));
  const stride=Math.max(1,Math.ceil(ids.length/6000));ctx.globalAlpha=name==='occupied'?0.8:0.25;
  for(let i=0;i<ids.length;i+=stride){
   const p=ids[i].map((v,k)=>lo[k]+(v+0.5)*data.resolution);
   const cube=corners.map(c=>c.map((v,k)=>p[k]+v*data.resolution/2));
   for(const [a,b] of edges)line(cube[a],cube[b],color);
  }
 }
 ctx.globalAlpha=1;
 if(document.getElementById('points').checked){ctx.fillStyle='#65ddff';for(const p of data.points){const q=project(p);ctx.fillRect(q[0],q[1],2*devicePixelRatio,2*devicePixelRatio);}}
 // Paths are drawn on top for readability, independently of the voxel slice.
 ctx.lineWidth=3*devicePixelRatio;
 for(const [id,key,color,dashed] of pathLayers){
  if(!document.getElementById(id).checked)continue;
  const route=paths[key] || [];ctx.setLineDash(dashed?[7*devicePixelRatio,5*devicePixelRatio]:[]);
  for(let i=1;i<route.length;i++){
   if(key==='executed_path' && paths.executed_segments && paths.executed_segments[i]!==paths.executed_segments[i-1])continue;
   line(route[i-1],route[i],color);
  }
 }
 ctx.setLineDash([]);ctx.lineWidth=devicePixelRatio;
}
canvas.onpointerdown=e=>{drag=[e.clientX,e.clientY];canvas.setPointerCapture(e.pointerId)};
canvas.onpointermove=e=>{if(!drag)return;az+=(e.clientX-drag[0])*0.008;el=Math.max(-1.5,Math.min(1.5,el+(e.clientY-drag[1])*0.008));drag=[e.clientX,e.clientY];draw()};
canvas.onpointerup=canvas.onpointercancel=()=>drag=null;
canvas.addEventListener('wheel',e=>{e.preventDefault();zoom=Math.max(.2,Math.min(12,zoom*Math.exp(-e.deltaY*.001)));draw()},{passive:false});
for(const id of ['points','occupied','free','unknown','height','nominalRoute','plannedPath','executedPath'])document.getElementById(id).oninput=draw;
document.getElementById('reset').onclick=()=>{az=-.7;el=.6;zoom=1;draw()};
window.onresize=draw;draw();
</script></html>'''
