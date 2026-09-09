"""Continuous free-space grasp protocol, shared unchanged by generation and replay."""

from __future__ import annotations

import time

import numpy as np
from scipy.spatial.transform import Rotation

from graspdatagen.config import ValidationProfile, digest
from graspdatagen.geometry import FloatArray, matrix_poses
from graspdatagen.records import FAILURES, METRICS, STAGES, CandidateBatch, ValidationBatch
from graspdatagen.runtime import GraspScene


def trial_conditions(
    batch: CandidateBatch, profile: ValidationProfile, seed: int
) -> tuple[np.ndarray, FloatArray]:
    seeds = np.array(
        [
            [int(digest([seed, int(i), t])[:16], 16) for t in range(profile.trials)]
            for i in batch.ids
        ],
        dtype=np.uint64,
    )
    accelerations = np.empty((len(batch), profile.trials, 6 + profile.random_directions, 3))
    for i in range(len(batch)):
        for t in range(profile.trials):
            rng = np.random.default_rng(seeds[i, t])
            random = rng.normal(size=(profile.random_directions, 3))
            random /= np.linalg.norm(random, axis=1)[:, None]
            accelerations[i, t] = np.vstack((np.eye(3), -np.eye(3), random)) * (
                profile.disturbance_acceleration_m_s2
            )
    return seeds, accelerations


def pose_error(reference: FloatArray, current: FloatArray) -> tuple[FloatArray, FloatArray]:
    delta = np.linalg.inv(reference) @ current
    translation = np.linalg.norm(delta[:, :3, 3], axis=1)
    angle = Rotation.from_matrix(delta[:, :3, :3]).magnitude()
    return translation, angle


def validate(
    scene: GraspScene,
    batch: CandidateBatch,
    initial_object: FloatArray,
    seeds: np.ndarray,
    accelerations: FloatArray,
) -> ValidationBatch:
    """Execute every trial from pregrasp; only reset writes the dynamic object pose.

    Explicit accelerations also serve the real late-disturbance counterexample:
    the audit changes its final pulse while using this same controller/checker.
    A failure is permanent within a trial, and all trials must pass to publish.
    """
    p = scene.profile
    n, trials = len(batch), p.trials
    if n != scene.count or initial_object.shape != (n, 4, 4):
        raise ValueError("Candidate and scene shapes disagree")
    if seeds.shape != (n, trials) or accelerations.shape != (n, trials, 6 + p.random_directions, 3):
        raise ValueError("Trial conditions have invalid shapes")
    if not np.isfinite(accelerations).all():
        raise ValueError("Nonfinite disturbance")
    failure = np.zeros((n, trials), dtype=np.int64)
    status = np.zeros((n, trials, len(STAGES)), dtype=np.int64)
    metrics = np.zeros((n, trials, len(STAGES), len(METRICS)))
    actual_tcp = np.tile(np.eye(4), (n, trials, 1, 1))
    actual_joints = np.zeros((n, trials, batch.pregrasp_joints.shape[1]))
    trace_lists: dict[str, list[np.ndarray]] = {
        key: [] for key in ("object", "tcp", "joints", "contact", "velocity", "stage")
    }
    T_B_tcp = scene.pair.arrays["T_B_tcp"]
    target_base = initial_object @ batch.target @ np.linalg.inv(T_B_tcp)
    pre_base = initial_object @ batch.pregrasp @ np.linalg.inv(T_B_tcp)
    limits = scene.pair.arrays["joint_limits_m"]
    calibration = scene.pair.definition["calibration"]
    active, follower = calibration["active_index"], calibration["follower_index"]
    mimic = calibration["mimic"]
    com_local = np.asarray(scene.pair.object_manifest["physical"]["com_pose_xyzw"][:3])
    zero_force = np.zeros((n, 3))
    stable_steps = round(p.stable_window_s * p.steps_per_second)
    loss_steps = round(p.contact_loss_s * p.steps_per_second)
    step_number = 0

    def run_trial(trial: int) -> None:
        scene.reset(matrix_poses(initial_object), matrix_poses(pre_base), batch.pregrasp_joints)
        if scene.runtime.gui:
            positions = np.concatenate((initial_object[:, :3, 3], pre_base[:, :3, 3]))
            scene.runtime.frame_validation(positions, scene.visual_radius)
        commands = batch.pregrasp_joints.copy()
        baseline = np.tile(np.eye(4), (n, 1, 1))
        baseline_ready = False
        missing = np.zeros(n, dtype=np.int64)
        stable = np.zeros(n, dtype=np.int64)
        previous_joints = batch.pregrasp_joints.copy()
        previous_object = initial_object.copy()
        previous_com = initial_object[:, :3, :3] @ com_local + initial_object[:, :3, 3]

        def observe(stage: int, holding: bool, settling: bool) -> dict[str, np.ndarray]:
            nonlocal step_number
            scene.runtime.step()
            step_number += 1
            state = scene.read()
            tcp = state["base"] @ T_B_tcp
            relative = np.linalg.inv(tcp) @ state["object"]
            contacts = state["contact"][:, :2].min(axis=1)
            bilateral = contacts >= p.minimum_contact_force_N
            missing[:] = np.where(bilateral, 0, missing + 1)
            solver_linear = np.linalg.norm(state["velocity"][:, :3], axis=1)
            solver_angular = np.linalg.norm(state["velocity"][:, 3:], axis=1)
            # Constraint-projected poses and solver velocity readback disagree
            # in real contact runs. Check resolved motion on every physics tick,
            # measuring translation at the COM and retaining raw velocities.
            com = state["object"][:, :3, :3] @ com_local + state["object"][:, :3, 3]
            linear = np.linalg.norm(com - previous_com, axis=1) * p.steps_per_second
            rotation = state["object"][:, :3, :3] @ previous_object[:, :3, :3].swapaxes(1, 2)
            angular = Rotation.from_matrix(rotation).magnitude() * p.steps_per_second
            previous_com[:] = com
            previous_object[:] = state["object"]
            joint_speed = np.abs(state["joints"] - previous_joints).max(axis=1) * p.steps_per_second
            previous_joints[:] = state["joints"]
            solver_joint_speed = np.abs(state["joint_velocity"]).max(axis=1)
            slow = (
                (linear < p.max_linear_speed_m_s)
                & (angular < p.max_angular_speed_rad_s)
                & (joint_speed < p.max_joint_speed_m_s)
            )
            stable[:] = np.where(bilateral & slow, stable + 1, 0)
            drift, angle = (
                pose_error(baseline, relative) if baseline_ready else (np.zeros(n), np.zeros(n))
            )
            alive = failure[:, trial] == 0
            row = metrics[:, trial, stage]
            first = row[:, -1] == 0
            row[alive, 0] = np.where(
                first[alive], contacts[alive], np.minimum(row[alive, 0], contacts[alive])
            )
            values = np.column_stack(
                (drift, angle, missing / p.steps_per_second, linear, angular, joint_speed)
            )
            row[alive, 1:7] = np.maximum(row[alive, 1:7], values[alive])
            row[alive, 7:11] = np.column_stack(
                (linear, angular, joint_speed, stable / p.steps_per_second)
            )[alive]
            row[alive, 11] = np.maximum(row[alive, 11], solver_joint_speed[alive])
            row[alive, 12] = solver_joint_speed[alive]
            row[alive, -1] += 1
            status[alive, trial, stage] = 1
            mimic_error = np.abs(
                state["joints"][:, follower]
                + mimic["gearing"] * state["joints"][:, active]
                + mimic["offset"]
            )
            limit_error = np.maximum(
                0, np.maximum(limits[:, 0] - state["joints"], state["joints"] - limits[:, 1])
            ).max(axis=1)
            palm_force = state["contact"][:, 2:].max(axis=1)
            row[alive, 13:16] = np.maximum(
                row[alive, 13:16], np.column_stack((mimic_error, limit_error, palm_force))[alive]
            )
            solver_speeds = np.column_stack((solver_linear, solver_angular))
            row[alive, 16:18] = np.maximum(row[alive, 16:18], solver_speeds[alive])
            row[alive, 18:20] = solver_speeds[alive]
            invalid_joints = np.maximum(mimic_error, limit_error) > p.joint_tolerance_m
            codes = np.zeros(n, dtype=np.int64)
            codes[invalid_joints] = FAILURES.index("joint_constraint_violation")
            palm = palm_force >= p.minimum_contact_force_N
            if stage <= 1:
                moved, rotated = pose_error(initial_object, state["object"])
                collision = (
                    (state["contact"] >= p.minimum_contact_force_N).any(axis=1)
                    | (moved > p.approach_translation_m)
                    | (rotated > p.approach_rotation_rad)
                )
                codes[collision] = 2
            elif stage == 2:
                codes[palm] = 4
            elif holding:
                slipped = (
                    (drift > p.max_translation_m)
                    | (angle > p.max_rotation_rad)
                    | (missing > loss_steps)
                    | palm
                )
                codes[slipped] = 5 if stage == 3 else (6 if stage == 4 else 7)
            if settling:
                codes[stable < stable_steps] = (
                    3 if stage == 2 else (5 if stage == 3 else (6 if stage == 4 else 7))
                )
                codes[(stage == 2) & bilateral & (stable < stable_steps)] = 4
            failed = alive & (codes != 0)
            failure[failed, trial] = codes[failed]
            status[failed, trial, stage] = -1
            if trial == 0 and (
                step_number == 1
                or step_number % max(1, p.steps_per_second // 20) == 0
                or settling
                or failed.any()
            ):
                trace_lists["object"].append(state["object"].copy())
                trace_lists["tcp"].append(tcp.copy())
                trace_lists["joints"].append(state["joints"].copy())
                trace_lists["contact"].append(state["contact"].copy())
                trace_lists["velocity"].append(state["velocity"].copy())
                trace_lists["stage"].append(np.array([stage, step_number], dtype=np.int64))
            return state

        opening_invalid = (
            (batch.opening <= batch.contact_width)
            | (commands < limits[:, 0]).any(axis=1)
            | (commands > limits[:, 1] + p.joint_tolerance_m).any(axis=1)
        )
        failure[opening_invalid, trial] = 1
        status[opening_invalid, trial, 0] = -1
        observe(0, False, False)
        for step in range(round(p.approach_s * p.steps_per_second)):
            fraction = (step + 1) / round(p.approach_s * p.steps_per_second)
            blend = fraction * fraction * (3 - 2 * fraction)
            base = pre_base.copy()
            base[:, :3, 3] += blend * (target_base[:, :3, 3] - pre_base[:, :3, 3])
            scene.move_base(matrix_poses(base))
            observe(1, False, False)
            if (failure[:, trial] != 0).all():
                break
        if (failure[:, trial] != 0).all():
            return
        stable[:] = 0
        missing[:] = 0
        for step in range(round(p.close_s * p.steps_per_second)):
            commands[:, active] += np.clip(
                batch.close_command - commands[:, active],
                -p.command_speed_m_s / p.steps_per_second,
                p.command_speed_m_s / p.steps_per_second,
            )
            scene.command(commands)
            state = observe(2, False, step == round(p.close_s * p.steps_per_second) - 1)
        baseline = np.linalg.inv(state["base"] @ T_B_tcp) @ state["object"]
        baseline_ready = True
        actual_tcp[:, trial] = np.linalg.inv(baseline)
        actual_joints[:, trial] = state["joints"]
        if (failure[:, trial] != 0).all():
            return
        scene.gravity(True)
        stable[:] = 0
        missing[:] = 0
        for step in range(round(p.hold_s * p.steps_per_second)):
            observe(3, True, step == round(p.hold_s * p.steps_per_second) - 1)
        if (failure[:, trial] != 0).all():
            return
        for direction in range(accelerations.shape[2]):
            stable[:] = 0
            for _ in range(round(p.disturbance_s * p.steps_per_second)):
                scene.force(accelerations[:, trial, direction])
                observe(4, True, False)
            scene.force(zero_force)
            for step in range(round(p.recovery_s * p.steps_per_second)):
                observe(4, True, step == round(p.recovery_s * p.steps_per_second) - 1)
            if (failure[:, trial] != 0).all():
                break
        scene.force(zero_force)
        if (failure[:, trial] != 0).all():
            return
        # World X is horizontal. Rotate the held state around each target TCP;
        # the object remains dynamic throughout the motion and final gravity hold.
        pivot = (target_base @ T_B_tcp)[:, :3, 3]
        base = target_base.copy()
        for step in range(round(p.invert_s * p.steps_per_second)):
            fraction = (step + 1) / round(p.invert_s * p.steps_per_second)
            blend = fraction**3 * (10 - 15 * fraction + 6 * fraction**2)
            rotation = Rotation.from_rotvec([np.pi * blend, 0, 0]).as_matrix()
            base[:, :3, :3] = rotation @ target_base[:, :3, :3]
            base[:, :3, 3] = pivot + (target_base[:, :3, 3] - pivot) @ rotation.T
            scene.move_base(matrix_poses(base))
            observe(5, True, False)
        scene.force(zero_force)
        stable[:] = 0
        for step in range(round(p.inverted_hold_s * p.steps_per_second)):
            observe(6, True, step == round(p.inverted_hold_s * p.steps_per_second) - 1)
        scene.force(zero_force)

    for trial in range(trials):
        trial_started = time.perf_counter()
        run_trial(trial)
        print(
            f"P2 trial {trial}: {(status[:, trial] == 1).all(axis=1).sum()}/{n} passed; "
            f"failure_codes={np.bincount(failure[:, trial], minlength=len(FAILURES)).tolist()}; "
            f"elapsed_s={time.perf_counter() - trial_started:.2f}",
            flush=True,
        )
    trace = {key: np.asarray(value) for key, value in trace_lists.items()}
    return ValidationBatch(
        batch,
        failure,
        status,
        metrics,
        actual_tcp,
        actual_joints,
        initial_object,
        seeds,
        accelerations,
        trace,
    )
