# Repository Guidelines

## Project Structure & Module Organization

GraspDataGen is a Python workflow built around IsaacLab/PhysX. Core scripts live in `scripts/graspgen/`; the highest-priority entry points are `datagen.py` and `grasp_sim.py`. Related generation, definition, configuration, and utility logic lives beside them, including `create_gripper_lab.py`, `grasp_guess.py`, `gripper_configurations.py`, `graspgen_utils.py`, `grasp_constants.py`, `mesh_utils.py`, and `warp_*` modules. Tools are under `scripts/graspgen/tools/`. Gripper USD assets are in `bots/`, object meshes and sample JSON inputs are in `objects/`, and documentation is in `docs/`.

## Workflow Priorities

The batch path is: generate candidate grasps with `datagen.py`, then validate them with `grasp_sim.py`. In practice, treat grasp guess output as provisional until simulation has accepted it. When changing gripper behavior, first regenerate the gripper definition with `create_gripper_lab.py`, then inspect the saved `.npz` and the emitted grasp YAML before changing downstream code.

## Multi-Gripper Adaptation Goals

The core project goal is to support multiple two-finger grippers that can generate correct grasp data accepted by real simulation validation. A gripper is not adapted just because `grasp_guess.py` emits a YAML file; it must have a usable gripper USD, a verified generated definition, plausible pregrasp/grasp c-space data, and successful `grasp_sim.py` results.

Gripper sources may come from:

- `bots/`: preferred location for pure gripper USD assets used by this pipeline.
- `robot_usds/grippers/`: candidate pure or near-pure gripper assets that may need cleanup before registration.
- `robot/`: full robots or arms that may contain a two-finger end effector.

If a source USD is a full robot, arm, or scene, do not use it directly as `--gripper_file` for data generation. First extract or generate a USD that contains only the gripper, put that result under `bots/`, register it in `GRIPPER_CONFIGS`, then validate it through the normal definition, guess, and simulation path. It is acceptable to edit or regenerate USD files when the USD itself is the root cause, including mass, friction, collision bodies, joint setup, drives, or frame placement.

## Gripper Validation Checklist

When adapting or debugging a specific gripper, inspect the USD itself before trusting wrapper configuration. Read the prim hierarchy and per-prim physics settings directly, including rigid bodies, joints, drives, mass, friction/material bindings, collision APIs, collider approximations, finger collider paths, base frame placement, open-limit semantics, and joint limits.

After changing a gripper USD or configuration:

- Regenerate the gripper definition with `create_gripper_lab.py`.
- Inspect the saved `.npz` and emitted grasp YAML metadata.
- Verify the generated open state is the widest valid pregrasp opening.
- Verify the generated grasp state moves toward the closed limit.
- Run `grasp_guess.py` for a small sample and confirm it produces plausible candidate data.
- Run `grasp_sim.py --max_num_grasps 16` or another focused simulation check and confirm actual validated successes.
- Treat a final successful grasp as one that passes all required validation stages: normal grasp and hold, multiple random-direction disturbance tests, then inversion of the already-grasping gripper/object pair followed by object gravity to confirm stable holding.

If no successful grasp data can be produced after reasonable tuning, do not keep changing parameters blindly. Determine which branch is failing:

- The gripper construction is physically unsuitable or impossible for the object set.
- The gripper USD is wrong, such as bad mass, friction, collision, joint, drive, or frame settings.
- The gripper definition extraction is wrong, such as incorrect finger colliders, base frame, open axis, approach axis, open limit, or bite point.
- The grasp guess pipeline is producing invalid pregrasp/grasp c-space, bad approach poses, bad centering, or incorrect close direction.
- The simulation validation is wrong or too strict, such as incorrect contact detection, force application, gravity handling, object mass, or success criteria.

## Agent-Specific Instructions

Approach fixes cautiously. Gather enough local context, rely on verifiable information, reproduce or validate the issue when practical, and identify the root cause before editing code. Do not patch from guesses. After the root cause is clear, make the smallest effective fix; avoid broad rewrites, large helper layers, or speculative code. When a bug touches a specific gripper, inspect the USD internals and the generated gripper definition first; do not rely only on wrapper configs or previous assumptions. Read the USD prim hierarchy and per-prim settings directly, including rigid bodies, joints, drives, mass, friction/material bindings, collision APIs, collider approximations, finger collider paths, base frame placement, open-limit semantics, and joint limits. Do not assume one gripper's parameters transfer to another. In grasp generation, verify that the pregrasp/open state is the widest valid opening and that the grasp state moves toward the closed limit. When changing behavior, first trace the impact through `scripts/graspgen/datagen.py`, `scripts/graspgen/grasp_sim.py`, and files they import. Preserve their CLI compatibility and output conventions unless the request explicitly changes them.

## Generate and Validate

- `uv run python scripts/graspgen/datagen.py \
    --gripper_config robotiq_2f_85 \
    --object_scales_json objects/datagen_example.json \
    --object_root objects \
    --num_grasps 1024 \
    --max_num_envs 256`: runs the batch pipeline locally.

- `uv run scripts/graspgen/grasp_sim.py \
      --grasp_file datagen_sim_data/robotiq_2f_85/banana.0.75.yaml \
      --object_file objects/banana.obj \
      --object_scale 0.75 \
      --max_num_envs 256 \
      --max_num_grasps 1024 \
      --force_headed`: validation.

## Commit

The current history uses short, imperative summaries such as `fix gripper usd generation` and `add piper usd`. Keep commits focused and mention the affected component when useful. Include a clear description, reproduction or validation commands, and any generated asset/data changes.

## Configuration Tips

Do not commit generated datasets, local virtual environments, or private object assets.
