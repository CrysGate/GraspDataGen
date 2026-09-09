# Web Grasp Viewer

The NiceGUI page renders USD geometry with Three.js in the browser. Asset reading
and forward kinematics run on the CPU; the web server does not initialize Isaac
Sim, CUDA, or a physics simulation. Run commands from the repository root.

`environments/web` has its own locked dependencies and virtual environment. Its
`usd-core` is deliberately isolated from Isaac Sim's USD/PhysX plugins. Do not
install the web environment's dependencies into the root simulation environment.

```bash
uv run --project environments/web --locked python environments/web/serve.py
```

Open http://127.0.0.1:8080. By default the dataset selector discovers
`outputs/**/grasps.yaml` and `grasps.yaml` beside objects listed in
`configs/objects/production.yaml`. An empty collection produces a CLI error.
Select specific files, including datasets outside `outputs`, with:

```bash
uv run --project environments/web --locked python environments/web/serve.py \
  --grasps outputs/production/piper--bottle/grasps.yaml \
           outputs/production/arx_x5--bottle/grasps.yaml
```

Use `--port 8081` if the port is occupied. `--host 0.0.0.0` enables access from
other machines. The viewer has no authentication, so bind it only on a trusted
network, or keep the default loopback address and use SSH port forwarding.
The HTTP data endpoint exposes only the datasets selected at startup.

## Assets and Coordinates

With an adjacent `manifest.json`, the viewer uses the prepared object and gripper
caches referenced by that manifest, including its calibrated TCP. Retain those
caches with the dataset. Cache fingerprints and robot/object identities are checked.
Missing or invalid caches are reported as errors.

For standalone YAML files, such as `Assets/Object/Rigid/bottle/grasps.yaml`, the
viewer resolves the object name through `--objects` and robot name through
`--grippers`. Their defaults are the existing production object list and the
YAML files in `configs/grippers`. Override them when viewing another asset set:

```bash
uv run --project environments/web --locked python environments/web/serve.py \
  --grasps Assets/Object/Rigid/bottle/grasps.yaml \
  --objects configs/objects/production.yaml \
  --grippers configs/grippers/piper.yaml
```

Standalone files depend on the current source assets and configurations. Keep
the original manifest and prepared caches for reproducible historical inspection.

All geometry is expressed in the object-root frame, in metres with Z up.
Gripper bodies use `T_object_tcp @ inverse(T_base_tcp) @ T_base_body`, with
`T_base_body` evaluated from USD fixed/prismatic joints and each candidate's
recorded closed joint positions. No opening estimate or simulated closure is used.
Malformed poses, nonfinite states, unsupported joints, and mismatched joint names
are rejected.

Visible source meshes take precedence over collision meshes. Old caches containing
only collision geometry are supported and marked `collision gripper`. Object
display colors and OmniPBR diffuse textures with vertex or triangular face-corner
UVs are transferred. Other material networks use display colors; the browser is
not an MDL renderer. This is a pose inspector, not a physical success evaluator.

## Viewing

- **Single grasp / All grasps:** inspect one pose or the entire dataset. All mode
  uses shared geometry and GPU instances for each gripper body, without sampling
  or silently limiting the number of displayed grasps. Dense mesh surfaces are
  simplified to 3,000 triangles per mesh in this mode (`--overview-faces` changes
  the budget); selected grasps always retain full geometry. Poses and closure
  transforms are identical in both representations.
- **Navigation:** previous/next, candidate ID, timeline, playback, or click a
  gripper in the distribution. IDs are zero-based; the timeline shows position
  within the collection. Selection remains opaque and amber in both modes.
- **Display:** distribution opacity, wireframe, object visibility, selected TCP
  axes, and ground grid. Approach colors blend X/Y/Z colors by the absolute
  components of the approach axis; opposite directions share a color.
- **Camera:** orbit, zoom, pan, fit, front, and top views. The camera fits the full
  distribution and adapts to the viewport aspect ratio.
- **Image:** the camera button downloads a PNG of the current 3D view.

Each browser tab owns its selection and display state. Up to four loaded datasets
are cached by the server; restart after modifying files on disk. Instance rendering
reduces geometry duplication, but very large datasets still require memory for
their transforms and can be slower with overlapping transparent surfaces.
