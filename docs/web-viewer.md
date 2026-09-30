# Web viewer

Generate a dataset, then run from the repository root:

```bash
uv run --locked python -m graspdatagen.web
```

Open http://127.0.0.1:8080. The viewer uses the CPU and discovers `grasps-<robot>.yaml` beside objects in `configs/objects/our_assets.yaml`.

To select an exported file:

```bash
uv run --locked python -m graspdatagen.web \
  --grasps Data/Assets/Object/Rigid/bubble_tea_cup/300g/grasps-piper.yaml
```

For another asset collection, pass its object manifest with `--objects` and gripper configurations with `--grippers`. These must match the exported file.

## Controls

- **Dataset:** select or open a grasp YAML file.
- **Single / All grasps:** inspect one grasp or its distribution.
- **Navigation:** step through grasps, enter an index or start playback.
- **Display:** toggle wireframe, visibility and TCP axes; adjust the camera.

Use `--port` to change the port. For remote access, use SSH forwarding or `--host 0.0.0.0` on a trusted network; the viewer has no authentication.

For region annotation, run `uv run --locked python -m graspdatagen.region_annotator_batch` after preparing assets, then open http://127.0.0.1:8765. Save annotations before setting `grasp_regions` in the run configuration.
