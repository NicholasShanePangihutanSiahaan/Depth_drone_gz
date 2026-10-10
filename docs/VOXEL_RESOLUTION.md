# Smaller navigation and logged voxels

Both `config/navigation.json` and `config/palm_farm_navigation.json` now use
`"resolution": 0.2`: **20 cm × 20 cm × 20 cm** cells. Previously the base map
used 30 cm and the farm map used 40 cm. The farm generator also preserves the
new 20 cm setting when recreating assets.

The real occupancy map, inflation, route checking, RViz cube dimensions, and
saved `map.json`/`map.html` all use this same resolution. This is not just a
display change. LiDAR sensor resolution and logged point-cloud downsampling
(10 cm) are unchanged; a finer grid cannot invent unmeasured surface details.
Unknown space remains blocked and clearance/drone dimensions stay unchanged.

Restart any running simulation to load the new setting:

```bash
cd /home/abin/polinasi_lidar
bash tools/start_simulation.sh ground_truth palm_farm
```

Existing saved maps retain their original resolution; only new runs produce
20 cm maps. The offline viewer now labels the voxel edge length. Browser and
RViz displays sample very large layers to keep at most 6,000 cubes per layer;
**navigation and saved JSON retain every voxel**. A displayed gap caused by
sampling does not mean the planner considers that space free.

Mapping-only missions now use a rolling 8 m navigation cube plus a sparse
whole-farm archive at the same 20 cm resolution. Missing archive indices are
explicitly unknown, not omitted free space. Their global RViz display caps are
12,000 occupied cells / 30,000 points; browser layers remain capped at 6,000.
See [mapping mission](MAPPING_MISSION.md) for current bounds and flight evidence.

## Performance and testing

Halving each farm cell edge increases cell count from 108,750 to 862,500, roughly
eightfold. Static rebuild timing on this computer measured a median **0.177 s**
at 20 cm vs **0.016 s** at 40 cm, with the optimised inflation implementation.
Core map arrays use about 16.4 MB vs 2.1 MB, excluding temporary arrays/export
data. See [performance report](../reports/voxel_performance.json).

To reduce the added workload, cylindrical safety inflation is computed as an
XY disk followed by a vertical maximum filter. This is mathematically the same
kernel, not reduced clearance. Tests compare it cell-for-cell with the original
3D operation at 20/30/40 cm, including blocked borders. Log voxel enumeration
now runs on the export worker rather than the controller callback. Visual
messages are bounded independently of navigation data.

Tests also check actual 20 cm navigation/export spacing, invalid-resolution
rejection, matching 20 cm RViz cubes, and the original thin-corner collision
regression (which deliberately retains its historical 30 cm fixture).
These checks do not validate real-time flight deadlines. The earlier timing
hold remains unresolved; finer mapping can increase that risk.

The complete ROS suite passed **56 tests**. An unarmed ground-truth farm run
confirmed a live 20 cm grid and saved all 862,500 cells. At 30 simulation
seconds its snapshot had 5,979 LiDAR points, 2,272 occupied cells, 31,672 free
cells and 828,556 unknown cells. See the
[validation report](../reports/voxel_resolution_validation.json).

Reproduce the static performance check:

```bash
OPENBLAS_NUM_THREADS=1 .venv/bin/python tools/check_voxel_performance.py
```
