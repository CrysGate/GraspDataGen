# GraspDataGen TODO

This file tracks the project roadmap for Codex/agent work. Keep it current when gripper adaptation status changes, a new candidate is discovered, or a failure mode is diagnosed.

## Core Goals

- Adapt multiple two-finger grippers so they can generate correct grasp data that passes real IsaacLab/PhysX validation.
- Ensure each adapted gripper has a usable pure gripper USD in `bots/`, an entry in `GRIPPER_CONFIGS`, a regenerated gripper definition, plausible guess output, and successful simulation-validated grasp YAML.
- Upgrade `grasp_sim.py` validation from a single acceptance check into staged validation:
  1. normal grasp attempt and hold;
  2. hold under multiple random-direction disturbance forces;
  3. after the gripper has already grasped the object, invert the gripper and object together, then apply gravity to the object to test whether the grasp remains stable.
- Treat only grasps that pass all three validation stages as final successful grasp data.
- Identify strong default grasp generation and simulation settings for each gripper instead of assuming one gripper's parameters transfer to another.

## Agent Operating Notes

- Treat `grasp_guess.py` output as provisional until `grasp_sim.py` accepts it.
- IsaacLab/PhysX is expected to be available; run focused validation after behavior changes.
- If a full robot or arm USD contains the desired gripper, extract or generate a pure gripper USD and place it in `bots/` before registering it.
- Do not mark a gripper as adapted until actual simulation-validated grasp data exists.
- When repeated runs produce no successful grasps, classify the root cause before tuning further: gripper physical feasibility, USD physics setup, definition extraction, guess generation, or simulation validation.

## Active Gripper Configs

These entries are registered in `scripts/graspgen/gripper_configurations.py`. Initial status is conservative unless verified in the current checkout.

| gripper_config | Source USD | USD physics check | Definition check | Guess check | Sim validation | Current issue | Next step |
| --- | --- | --- | --- | --- | --- | --- | --- |
| `robotiq_2f_85` | `bots/robotiq_2f_85.usd` | Needs validation | Needs regenerated `.npz` inspection | Needs pregrasp/open and close-direction check | Needs simulation successes | Baseline gripper but current pass status is unverified | Run definition, guess, and small sim smoke |
| `onrobot_rg6` | `bots/onrobot_rg6.usd` | Needs validation | Needs regenerated `.npz` inspection | Needs pregrasp/open and close-direction check | Needs simulation successes | Batch example target but current pass status is unverified | Run definition, guess, and small sim smoke |
| `franka_panda` | `bots/franka_panda.usd` | Needs validation | Needs regenerated `.npz` inspection | Needs pregrasp/open and close-direction check | Needs simulation successes | Current pass status is unverified | Run definition, guess, and small sim smoke |
| `piper_v2_gripper` | `bots/piper_v2_gripper.usd` | Needs validation | Needs regenerated `.npz` inspection | Needs pregrasp/open and close-direction check | Needs simulation successes | Piper adaptation may need USD physics/collision review | Inspect USD internals, then run definition, guess, and small sim smoke |

## Pure Gripper USD Candidates

These USDs look like pure or near-pure gripper assets but are not all registered or validated.

| Candidate | Path | Status | Next step |
| --- | --- | --- | --- |
| Piper H v1 | `bots/piper_h_v1_gripper.usd` | Unregistered candidate | Inspect USD and decide whether to add `GRIPPER_CONFIGS` entry |
| Piper L v1 | `bots/piper_l_v1_gripper.usd` | Unregistered candidate | Inspect USD and decide whether to add `GRIPPER_CONFIGS` entry |
| Piper v1 | `bots/piper_v1_gripper.usd` | Unregistered candidate | Inspect USD and decide whether to add `GRIPPER_CONFIGS` entry |
| Piper X v1 | `bots/piper_x_v1_gripper.usd` | Unregistered candidate | Inspect USD and decide whether to add `GRIPPER_CONFIGS` entry |
| ChangingTek AG2F120S | `robot_usds/grippers/ChangingTek_AG2F120S/` | Candidate source | Inspect top-level USD and physics layers, then copy/clean pure gripper USD into `bots/` if suitable |
| ChangingTek AG2F90 | `robot_usds/grippers/ChangingTek_AG2F90/` | Candidate source | Inspect available pad variants and choose a default before creating a `bots/` asset |
| Galaxea G1 | `robot_usds/grippers/Galaxea_G1/` | Candidate source | Inspect left/right variants and choose/extract a pure gripper asset |
| Inspire EG2 4C2 | `robot_usds/grippers/Inspire_EG2_4C2/` | Candidate source | Inspect left/right variants and create a validated `bots/` asset |
| Jodell RG75 | `robot_usds/grippers/Jodell_RG75/` | Candidate source | Inspect pad variants and choose a default before creating a `bots/` asset |
| OmniPicker | `robot_usds/grippers/OmniPicker/` | Candidate source | Inspect whether it is a two-finger gripper and whether it fits the current pipeline |
| Robotiq 85 | `robot_usds/grippers/Robotiq_85/` | Candidate source | Compare with `bots/robotiq_2f_85.usd` before using |

## Extraction Candidates

These sources may contain two-finger grippers but are full robots, arms, scenes, or mixed assets. They must not be used directly as final `gripper_file` values for data generation.

| Source | Path | Status | Next step |
| --- | --- | --- | --- |
| Local robot assets | `robot/` | Extraction candidates | Inspect each robot for two-finger end effectors; extract only the gripper into `bots/` |
| Humanoid robot USDs | `robot_usds/humannoid/` | Extraction candidates | Search for two-finger end-effector components and avoid full-body USDs |
| Manipulator USDs | `robot_usds/manipulators/` | Extraction candidates | Search end-effector attachments and extract only gripper geometry/physics |
| Piper Isaac Sim assets | `piper_isaac_sim/` | Extraction candidates | Use as reference for Piper variants when `bots/` USDs are incomplete |

## Grasp Simulation Roadmap

| Task | Status | Notes |
| --- | --- | --- |
| Stage 1: normal grasp attempt and hold | Current behavior exists but needs per-gripper acceptance review | Confirm success criteria and contact detection are reliable for each gripper |
| Stage 2: random disturbance-force hold validation | Partially present through tug sequences | Apply multiple random-direction disturbances after grasping and report per-attempt pass/fail |
| Stage 3: inverted gripper/object gravity hold | Not implemented as a clear stage | Keep the object grasped, invert gripper and object together, then enable/apply object gravity and report stability |
| Per-stage result metadata in YAML | Needed | Store which stage each grasp passed or failed |
| Debug output for failed grasps | Needed | Record enough evidence to distinguish USD, guess, and sim validation failures |

## Failure Diagnosis Checklist

Use this checklist when a gripper cannot generate successful validated grasp data:

- Physical feasibility: can the gripper geometry, stroke, bite depth, and finger pads plausibly grasp the target object?
- USD physics: are mass, inertia, friction, collision APIs, collider approximation, rigid bodies, joint limits, and drives correct?
- Definition extraction: are `finger_colliders`, `base_frame`, `open_axis`, `approach_axis`, `open_limit`, `bite_point`, and generated c-space samples correct?
- Guess generation: is pregrasp the widest valid opening, is the grasp state closer to closed, are the object-relative transforms plausible, and are collision checks using the right bodies?
- Simulation validation: are contacts detected on the intended finger bodies, is the close direction correct, are force/gravity/object mass settings reasonable, and is the success criterion matching the intended behavior?
