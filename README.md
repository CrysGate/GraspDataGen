# GraspDataGen

Generate parallel-jaw grasp data directly from rigid USD/USDZ assets with Isaac
Sim 6.0.1, native GPU PhysX and Warp. The public command is `uv run graspdatagen`.
Python 3.12 and all runtime versions are locked in the root project.

## Run

On Linux x86_64 with a supported NVIDIA GPU and driver, install
[uv](https://docs.astral.sh/uv/getting-started/installation/) and make the external
asset tree available at `Assets/`. See [installation](docs/installation.md) for
the required paths and dependency details. Run from the repository root:

```bash
uv sync --locked
export OMNI_KIT_ACCEPT_EULA=YES
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=8
uv run --locked graspdatagen generate --config configs/runs/production.yaml
```

The production profile requests 1,024 distinct successes for each of Piper /
ARX-X5 and the bottle / 25 matryoshka instances, for 52 combinations.
It writes `outputs/production/report.json`, per-pair manifests, numeric NPZ shards,
diagnostics and worker logs. Inspect each combination's status and success count;
`complete: true` also permits an explicitly reported insufficient or invalid
combination. `all_targets_reached` checks the requested quantity.

Each generated pair gets `grasps.yaml` as soon as that combination finishes and
passes log verification: candidate ID, robot, actual grasp pose, approach axis in
object coordinates and measured closed joint positions.
Existing datasets can be exported
with `uv run --locked graspdatagen export --run outputs/production/piper--bottle`
without GPU simulation.

Inspect those static grasps in Isaac Sim from a graphical desktop:

```bash
uv run --locked graspdatagen view --grasps outputs/production/piper--bottle/grasps.yaml
```

The viewer reads poses and named joint states from YAML, with asset references
from the adjacent `manifest.json`. It does not simulate approach or holding.
Select a zero-based candidate in the window or pass `--candidate 0` at startup.

For browser inspection with NiceGUI, including all grasp poses at once:

```bash
uv run --project environments/web --locked python environments/web/serve.py
```

Open http://127.0.0.1:8080. The page offers single/all pose modes, recorded gripper
closure, transparency, pose navigation, camera controls and image export. It loads
prepared datasets or standalone YAML beside configured source objects without
starting Isaac Sim. Its isolated environment keeps USD dependencies separate from
the simulation runtime. See [the web viewer guide](docs/web-viewer.md) for data selection
and remote access.

```bash
uv run --locked graspdatagen replay --run outputs/production/piper--bottle \
  --environments 1 --output outputs/replay/piper--bottle.json
uv run --locked graspdatagen generate --config configs/runs/production.yaml --resume
```

Replay executes saved pregrasp commands and every physical trial in a new worker.
Resume requires the original configuration, implementation and inputs. For a new
task, set a new `output` in the YAML. All commands that use GPU, including inspect
and prepare, must run outside the agent sandbox.

## Project Layout

- `Assets/`: external source assets; never generated or modified by this project.
- `configs/`: robot/gripper definitions, the production object list and run settings.
- `outputs/prepared/`: generated collision geometry, physical USD assets and gripper calibration.
- `outputs/production/`: generated datasets and reports.
- `src/graspdatagen/`: application code.

The YAML `cache` field selects the prepared-asset directory. `generate` creates
missing assets automatically and reuses matching prepared assets across runs.
Object collisions retain the source asset's collision meshes and PhysX settings.
Native cooked hulls are stored for sampling; there is no project decomposition
or surface-error budget.
These assets are also required for replay, so retain them with any datasets you
keep. They can be deleted when discarding the dependent datasets; the next
generation will rebuild them. No root `cache/` or `bots/` directory is needed.

## Data and Scope

`pose_object_tcp_xyz_xyzw` is the configured TCP in the original object-root frame,
stored as `[x, y, z, qx, qy, qz, qw]` in metres with a unit quaternion. It is the
actual stable closure pose before disturbance. Each accepted grasp passes approach,
closure, gravity hold, random disturbances, continuous inversion and inverted hold
in one complete trial with the production profile. See the [data contract](docs/contracts.md).

Compact YAML exports include top-level `tcp` (parent frame, position in metres,
and xyzw orientation) and `approach_distance_m` (the configured pregrasp offset).
These are copied from the generation manifest for consumers to check TCP
compatibility and construct an approach pose.

Supported initial grippers are Piper and ARX-X5. Their TCP definitions come from
the portable robot configuration snapshots. ARX friction 0.8 is an explicit
simulation assumption. Full-arm IK, scene avoidance and hardware certification
are outside this dataset's scope.

The trial-reset defect behind the historical batch-replay failure has been fixed;
the 64-row validation dataset passed fresh-process batch replay. See the
[consistency diagnosis](docs/consistency.md) for the controlled comparison.
Full production acceptance is still open. The [remaining work](docs/refactor-plan.md)
records the outstanding validation and production requirements. Historical local
datasets and diagnostic outputs have since been cleaned up.
`configs/runs/production.yaml` enumerates
the bottle and all 25 matryoshka instances; it is a bounded production profile,
not a guarantee of 1,024 successes for every pair.

## Documentation

- [Installation and CLI](docs/installation.md)
- [Physical and coordinate contracts](docs/contracts.md)
- [Gripper adaptation](docs/grippers.md)
- [Current Status and Remaining Work](docs/refactor-plan.md)

The former `scripts/graspgen` commands, OBJ/STL workflow, IsaacLab environment,
Docker setup and sample assets have been replaced. Old source remains in Git
history. Existing v1 P2 datasets can be replayed; new writes use only v2.
Source assets, caches and generated datasets are not bundled with the project.
