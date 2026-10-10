# Palm-oil farm scene

The new scene replaces abstract obstacle boxes **only when you choose
`palm_farm`**. Existing clear/blocked/dropout test scenes remain available.

Terminal 1:

```bash
cd /home/abin/polinasi_lidar
bash tools/start_simulation.sh ground_truth palm_farm
```

Terminal 2 (opens the existing simulation, not another server):

```bash
cd /home/abin/polinasi_lidar
bash tools/view_simulation.sh
```

Use the mouse to move around the Gazebo scene. The initial camera overlooks
the farm. The bottom controls show play/pause and simulation speed. Starting
the scene does **not** arm the drone. Mission preparation/start commands are
unchanged; see [local setup](LOCAL_SETUP.md). Do not disable safety guards.

## What changed and why

- 29 palms in staggered rows, with background size/yaw variation, instead of
  box-shaped trees. This creates a recognisable plantation and more complex
  LiDAR occlusions (surfaces hiding other surfaces).
- Tapered, rough brown trunks with retained leaf bases; 18 curved fronds per
  tree with paired, thin leaflets; coloured fruit bunches near the crown.
- Brown soil and coherent green groundcover patches, small grass blades,
  a dirt lane, clear launch pad, blue sky, directional sunlight and shadows.
  Materials explicitly specify colour and meshes include surface normals,
  avoiding the earlier ambient-only black rendering issue.
- Visual and collision meshes match, including leaflets and grass. No invisible
  box is substituted for a palm crown. The solid ground remains a flat slab.
- Farm-specific navigation configuration is selected automatically. Two target
  palms are at XY `(9, 0)` and `(18, 4.5)` m, with target height 4.8 m and nominal
  inspection radius 4.2 m. Map bounds and candidate viewing heights expand to
  cover them. These are trial mission goals, not confirmed flower positions.
- Automatic point-cloud/voxel exports still go into the launcher's printed
  `reports/.../map_3d/map.html` folder. That sensor map contains points and
  voxels, **not** the plantation's coloured mesh models.

All assets are generated locally from [generate_palm_farm.py](../tools/generate_palm_farm.py),
with no downloads or external licences. Models are packaged with the ROS package.
The reusable palm model lives under `polinasi_nav/models/palm_farm`.
The farm navigation map now uses 20 cm cells; see [smaller voxels](VOXEL_RESOLUTION.md).

Recreate assets and rebuild if changing the generator:

```bash
/usr/bin/python3 tools/generate_palm_farm.py
bash tools/build_ros.sh
```

## Limits

This is a detailed **procedural** scene, not a photogrammetry scan or a
botanically/photometrically calibrated plantation. Spacing and dimensions are
modelling choices. Soil uses coloured geometry rather than photographic
textures. Trunks and leaves are rigid; no wind, leaf translucency, moving
foliage, accurate flower recognition or sprayer physics is modelled. The camera
is still the original simplified ZED placeholder.

More detailed geometry increases graphics/CPU load. The earlier
`clock_or_scheduler_gap` mission hold has **not** been fixed by this scene change.
Farm startup/rendering/map checks are not a completed mission or validated
LiDAR–IMU localisation. Use ground truth for scene/planner development.

## Tests run

The final build and ROS regression suite passed all **46 tests**. Three farm
asset tests check model/target alignment, launch-area clearance, matching visual
and collision mesh URIs, valid triangle indices/normals, colours and ground
coverage. Gazebo SDF validation passed, with existing `gz_frame_id` extension
warnings from the drone sensors.

An unarmed ground-truth Gazebo run rendered the farm and exported a sensor map:
at 10 simulation seconds, 5,230 accumulated points and 1,051 occupied voxels,
with surface hits up to 6.79 m high. The final run showed no mesh-loading errors.
The viewer showed roughly 26% real-time speed on this laptop; 10 simulation
seconds can therefore take considerably longer than 10 wall-clock seconds.
No flight was commanded. Test processes were stopped after inspection.

[Farm preview](../reports/palm_farm_preview.png) ·
[Validation report](../reports/palm_farm_validation.json)
