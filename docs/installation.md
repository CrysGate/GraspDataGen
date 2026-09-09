# Installation and CLI

## Environment

Run from the repository root on Linux x86_64 with Python 3.12, `uv`, a supported
NVIDIA GPU/driver and enough disk space for the Isaac Sim distribution and asset
cache. The runtime was verified on an RTX 5090 with driver 595.84. GPU commands must run
outside the agent sandbox. Do not install a separate `usd-core`: USD/PhysX plugins
must come from the same Isaac Sim release.

```bash
uv sync --locked
uv run --locked graspdatagen --help
```

`uv` creates the root `.venv` and installs this project, including the
`graspdatagen` console entry point and `physx.kit` runtime resource. No
`PYTHONPATH`, IsaacLab checkout or external Python environment is needed. Deactivate
an older virtual environment before syncing. The default development group includes
Ruff, mypy and YAML typing stubs; use `uv sync --locked --no-dev` for runtime only.

The locked runtime is Isaac Sim 6.0.1.0, NumPy 2.3.1, Warp 1.13.0, Torch
2.10.0/CUDA 12.8, trimesh 4.11.1, SciPy 1.17.0, Matplotlib 3.10.8
and PyYAML 6.0.3. Matplotlib produces inspection
and diagnostic images. The verified overrides retain torchvision 0.25.0 and
torchaudio 2.10.0 only as Isaac Sim transitive dependencies. The full Sim
distribution may also install Newton packages; project execution selects PhysX.
Isaac Sim also depends on CoACD transitively; GraspDataGen does not call it.
`uv.lock` preserves the already qualified dependency versions and index sources.

## Assets

Supply the following external files under `Assets/`, or adjust the project YAML
paths to an equivalent asset tree. A local symlink is supported and is ignored by
Git. Source files are read-only inputs; derived files go into `outputs/prepared/`.

```text
Assets/Robots/piper/Piper.usd
Assets/Robots/piper/piper_description/urdf/piper.urdf
Assets/Robots/x5/ARX.usd
Assets/Robots/x5/X5A.urdf
Assets/Object/Rigid/bottle/bottle.usd
Assets/Object/Rigid/bottle/bottle.json
Assets/Object/Rigid/matryoshka_dolls/00002/object.usdz
Assets/Object/Rigid/matryoshka_dolls/00002/metadata.json
```

Keep the source assets' referenced layers, meshes, materials and textures. The
production manifest also requires instances 00000 through 00024 and their metadata.
The portable `configs/robots/` snapshots define authoritative TCPs and inherit
drive settings from USD. They do not depend on the original configuration checkout.

## Commands

Accept the Isaac Sim EULA according to your installation's licensing requirements
before setting the environment variable below. These CPU thread limits are the
settings used for real qualification:

```bash
export OMNI_KIT_ACCEPT_EULA=YES
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=8

uv run --locked graspdatagen inspect --manifest configs/objects/production.yaml \
  --output outputs/inspect.json

uv run --locked graspdatagen prepare --manifest configs/objects/production.yaml \
  --gripper configs/grippers/piper.yaml \
  --output outputs/prepare-piper.json
uv run --locked graspdatagen prepare --manifest configs/objects/production.yaml \
  --gripper configs/grippers/arx_x5.yaml \
  --output outputs/prepare-arx_x5.json

uv run --locked graspdatagen generate --config configs/runs/production.yaml
uv run --locked graspdatagen generate --config configs/runs/production.yaml --resume

uv run --locked graspdatagen replay --run outputs/production/piper--bottle \
  --environments 1 --output outputs/replay/piper--bottle.json

uv run --locked graspdatagen audit --run outputs/production/piper--bottle \
  --config configs/runs/audit.yaml --output outputs/audit/piper--bottle.json
```

`prepare` is an independent inspection/prewarm command; `generate` calls the same
preparation implementation when caches are missing. To check automatic preparation
from scratch, run `generate` first with a new cache path. The default preparation
directory is `outputs/prepared`; `--cache` selects a different directory when
isolating another dataset's prepared assets. The preparation report's `gripper`
path contains `gripper.usdc` and its calibration. No separate `bots/` aliases
are generated. The run YAML uses the same directory through its `cache` field.

The production run requests 512 environments and 1,024 distinct successes per
combination. These are requested limits, not completed performance or quantity
acceptance. Adjust the run YAML for available GPU resources and the desired
object manifest; validation thresholds should retain their physical meaning.
Collision geometry follows the source asset's native PhysX settings. Preparation
retains the complete selected gripper bodies, including their visual meshes,
materials and native collision prims. Cooked hulls are cached only for sampling
and calibration; no project decomposition or visual-surface error budget is
applied. PhysX still cooks the native collision shapes used by simulation.
`prepare` stops on an invalid asset; `generate` records it and continues with
the remaining objects.

`inspect`, `prepare`, `replay` and `audit` require `--output` and use GPU 0 unless
`--device` is provided. `generate` takes device and output from its YAML. Replay
defaults to one environment and all saved grasps; `--grasp-id` selects one positive
candidate ID. Audit defaults to all cases; `--case` selects an individual physical
counterexample or sensitivity run. `--help` lists the supported arguments.

### Live GUI

Add `--gui` to `replay`, `generate` or `audit` to watch the physical validation.
Run from a graphical desktop terminal (including a remote desktop) with its valid
`DISPLAY` and X11 authorization. An SSH terminal without a display cannot open
an interactive window; do not assume another user's display is accessible.

```bash
uv run --locked graspdatagen replay --run outputs/production/piper--bottle \
  --environments 16 --gui --output outputs/gui-replay/piper--bottle.json
```

The overview camera frames all environments in the current batch. The viewport
shows the gripper's retained source visuals and the object's red collision geometry.
Standard viewport camera controls allow closer inspection; the toolbar's
Pause/Play controls pause and resume the physical execution. Stop aborts the run.
Rendering runs at
approximately 30 frames per simulated second, paced no faster than real time;
slow rendering can make playback slower. Physics timesteps and acceptance limits
are unchanged. The window closes after the requested trials finish; closing it
early aborts validation. GUI execution counts against generation's time budget.

GUI replay places each saved grasp in a separate grid cell, retaining its object
orientation, relative grasp poses, commands and disturbance conditions. This
prevents successes from different original batches overlapping on screen.
Its report records `layout: grid`; default headless replay retains the original
world positions (`layout: saved`) for reproducibility checks. World translations
can affect floating-point contact results, so use headless replay for comparison
with the original saved placement.

On a server, a virtual X display can exercise rendering for diagnostics, but it
is not itself a remotely viewable desktop. A remote desktop or X forwarding is
still required to interact with the GUI. Inactive physical desktop sessions may
fail Vulkan initialization even when X11 authentication succeeds.

Generation stops at the distinct-success target, candidate budget, elapsed budget
or sampling-round limit. A started batch always finishes the full protocol.
`insufficient_valid_grasps` and `asset_invalid` are explicit per-pair outcomes;
they must not be presented as successful target completion. An existing output
requires `--resume`; changed inputs, configuration or code require a new output.
Keep cache directories with datasets because replay resolves their saved prepared
asset paths. Old output is not silently upgraded or overwritten.

## Static Grasp Inspection

From a local or remote graphical desktop with `DISPLAY` set:

```bash
uv run --locked graspdatagen export --run outputs/production/piper--bottle
uv run --locked graspdatagen view \
  --grasps outputs/production/piper--bottle/grasps.yaml --candidate 0
```

New generation exports automatically. Re-export older YAML to include
`closed_joint_positions_m`, the measured closure position of each named joint.
The viewer reads grasp states directly from YAML and uses its adjacent
`manifest.json` to locate the prepared object, gripper and TCP definition.
Keep those prepared assets with the data. NPZ grasp shards are not needed by
the viewer. Candidate selection uses YAML's zero-based IDs, not replay IDs.

The object and gripper use their retained visual surfaces and materials.
Fixed/prismatic joint frames place each finger at its recorded state.
Physics does not advance, so the poses remain static while inspecting. Use the
previous/next buttons or numeric selector to change grasps and Frame Grasp to
reset the camera. Normal Isaac Sim viewport orbit, pan and zoom remain available.
This view is an inspection tool; use `replay` for physical revalidation.

## Checks

```bash
uv run --locked ruff check src/graspdatagen
uv run --locked mypy
uv build --no-sources
```

Static checks supplement real asset preparation, generation and fresh-process
replay. This project does not use unit tests or mock simulation for acceptance.
