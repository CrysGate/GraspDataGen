# Data and Physical Contracts

## Coordinates

All translations use metres; time, mass, force and angle use s, kg, N and rad.
`T_A_B` maps column-vector coordinates from B to A. Quaternions are unit `xyzw`
with canonical sign. Public poses are seven numeric values, not matrices.

| v2 field | Shape | Meaning |
| --- | --- | --- |
| `pose_object_tcp_xyz_xyzw` | `(N, 7)` | Actual stable closure in trial 0 |
| `pose_object_tcp_trials_xyz_xyzw` | `(N, trials, 7)` | Actual stable closure in every trial |
| `pose_object_tcp_target_xyz_xyzw` | `(N, 7)` | Sampled approach target |
| `pose_object_tcp_pregrasp_xyz_xyzw` | `(N, 7)` | Initial approach pose |
| `pose_world_object_initial_xyz_xyzw` | `(N, 7)` | Initial object placement for replay |

`object` is the original selected USD root, not a recentered mesh or centre of
mass. `tcp` is the portable robot configuration's TCP. Piper's offset is composed
through its rotated `gripper_center` frame; ARX uses its `link6` frame.

```text
T_B_tcp      = T_B_parent * T_parent_tcp
T_W_tcp      = T_W_object * T_object_tcp
T_W_B        = T_W_tcp * inverse(T_B_tcp)
T_object_tcp = inverse(T_W_object) * T_W_tcp
```

The exported actual pose uses simultaneous measured object and gripper states
before disturbance. Fixed calibration matrices remain in the definition; trial
diagnostics are separate from the public grasp-pose fields. NPZ contains numeric
arrays only and loads with `allow_pickle=False`.

## Physical Protocol

Object collision geometry is authoritative in the source USD/USDZ. Preparation
retains the original collision meshes and settings for simulation, and stores
their PhysX convex hulls for sampling. It does not decompose the visual mesh or enforce
a visual-surface approximation budget. Native `convexDecomposition` settings
still require PhysX cooking. Mass, inertia, material and grasp checks remain.

The sampler starts with an upright object and seeded yaw, a non-upward approach,
an upward-facing wrist mounting side and full-gripper clearance above the object
bottom plane. It checks actual closure in every trial against these constraints.
This is free-space local grasp validation, not a table pickup or full-arm plan.

The dynamic object's gravity is off only during reset, approach and closure.
It is enabled for gravity hold, multi-direction disturbances, continuous
180-degree inversion about world X and inverted hold. Contact must come from
both fingers against the target object. Force, slip, rotation and velocity limits
apply through the stages; a failure cannot be erased by subsequent contact.
Each saved grasp passes all stages in every configured trial. The production
profile uses one trial; historical datasets may contain three and replay retains
their recorded protocol.

Every trial starts with a fresh PhysX scene, including when a worker advances to
the next candidate batch. Resetting only tensor poses and velocities does not
clear internal contact/solver history and previously changed replay outcomes
between one and multiple environments. Explicit audit mass/friction scales are
restored when rebuilding. Replay retains saved world coordinates in headless mode;
GUI grid placement is a visualization variant, not an identical-coordinate check.

Disturbance forces use `mass_kg * acceleration_m_s2 * direction`. The saved
protocol includes units, seeds, time steps, material assumptions, drive limits,
solver settings and stage results. No physical limit is relaxed to meet a target.
Bottle uses the explicit 0.2 kg metadata override; matryoshka 00002 uses 0.05 kg
and friction 0.45. Current ARX fingertip friction 0.8 is an explicit simulation
model, not a measurement. Historical P2 used ARX friction 0.5.

## Storage and Replay

As soon as a generated pair finishes and its log has passed verification up to that
checkpoint, it gets `grasps.yaml`; later combinations do not delay its export.
This compact view numbers successful candidates from 0 in each YAML file, in
committed NPZ row order. Top-level `tcp` records `parent_frame`, `position_m`, and
`orientation_xyzw` from the generation's TCP configuration, relative to its parent
frame. Top-level `approach_distance_m` records the generation's configured
`sampling.pregrasp_distance_m`; it is the sampled pregrasp offset, not a measured
displacement from the final stable closure pose. Both values come from the saved
manifest, so exporting later does not substitute current configuration values.
These local indices are not the NPZ candidate IDs used
by replay's `--grasp-id`. Each entry also contains the robot snapshot's `name`,
the actual stable closure pose (trial 0), and `approach_axis_object`.
`closed_joint_positions_m` maps each gripper joint name to its measured position
at that same closure instant, in metres for the supported prismatic grippers.
It includes both fingers, not the commanded closing target.
The axis is a unit vector pointing along the gripper's approach direction,
rotated into the original object-root frame by the actual closure orientation.
It is not the historical approach displacement when closure moved the object.
Positions are in metres and quaternion order is xyzw. NPZ remains authoritative
for replay and diagnostics. Export existing verified data without GPU using
`uv run --locked graspdatagen export --run outputs/production/piper--bottle`.

Each pair has a manifest, checksummed NPZ shards, configuration and asset hashes,
candidate IDs, actual and commanded joints, stage metrics and failure counts.
`worker_verified` means the pair reached its final checkpoint without detected
physics or rendering errors. The run report separately verifies worker exit and GPU
release after all combinations; fresh-process replay has a separate report.
Replay starts from saved pregrasp/target poses and joint commands and reruns every
trial. It does not teleport directly to a final holding pose.

Generation uses version `graspdatagen.grasps.v2`. Existing P2 v1 evidence has a
read-only replay conversion; v1 cannot be appended to. Earlier legacy OBJ/YAML/NPZ
outputs are not accepted as new successful data.

Committed checkpoints record sampling progress and unique successes. Resume
validates identities and checksums, restores committed deduplication state, and
reruns uncommitted work. Changing code, source dependencies or configuration
creates a new task. Cache and source inputs remain distinct from output shards.
