"""Bounded production with deterministic replenishment and acknowledged checkpoints."""

from __future__ import annotations

import json
import time
from collections import Counter
from dataclasses import asdict
from pathlib import Path
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation

from graspdatagen.assets import file_hash, prepare_object, source_fingerprint
from graspdatagen.config import RunConfig, digest, load_gripper, load_objects
from graspdatagen.geometry import pose_matrices
from graspdatagen.grippers import prepare_gripper
from graspdatagen.records import FAILURES, METRICS, STAGES, PreparedPair
from graspdatagen.runtime import GraspScene, PhysxRuntime, RuntimeConfig
from graspdatagen.sampling import Sampler, distinct_indices
from graspdatagen.storage import (
    FORMAT,
    commit_checkpoint,
    durable_json,
    empty_result_arrays,
    read_shards,
    result_arrays,
    write_arrays,
)
from graspdatagen.validation import trial_conditions, validate


def generate(runtime: PhysxRuntime, config: RunConfig) -> list[dict[str, Any]]:
    reports: list[dict[str, Any]] = []
    prepared_objects: dict[str, Path] = {}
    invalid_objects: dict[str, dict[str, str]] = {}
    preparation_seconds: dict[str, float] = {}
    objects = load_objects(config.manifest)
    sources = [item.source for item in objects]
    sources.extend(load_gripper(path).source for path in config.grippers)
    fingerprints: dict[str, Any] = {"assets": {}, "metadata": {}}
    for path in sources:
        try:
            fingerprints["assets"][str(path)] = source_fingerprint(path)
        except (ValueError, FileNotFoundError) as error:
            fingerprints["assets"][str(path)] = {"error": str(error)}
    for item in objects:
        try:
            fingerprints["metadata"][str(item.metadata)] = file_hash(item.metadata)
        except FileNotFoundError as error:
            fingerprints["metadata"][str(item.metadata)] = {"error": str(error)}
    input_path = config.output / "inputs.json"
    if input_path.exists():
        if json.loads(input_path.read_text()) != fingerprints:
            raise ValueError("Resume rejected: source assets or their dependency contents changed")
    else:
        durable_json(input_path, fingerprints)
    preparation_path = config.output / "preparation.json"
    if preparation_path.exists():
        previous = json.loads(preparation_path.read_text())
        invalid_objects.update(previous["invalid"])
        preparation_seconds.update(previous["elapsed_s_by_object"])
    for item in objects:
        if item.name in invalid_objects:
            continue
        prepared_at = time.monotonic()
        try:
            prepared_objects[item.name] = prepare_object(runtime, item, config.cache)
        except (ValueError, FileNotFoundError) as error:
            invalid_objects[item.name] = {"type": type(error).__name__, "message": str(error)}
        preparation_seconds.setdefault(item.name, time.monotonic() - prepared_at)
        durable_json(
            config.output / "preparation.json",
            {
                "prepared": {name: str(path) for name, path in prepared_objects.items()},
                "invalid": invalid_objects,
                "elapsed_s_by_object": preparation_seconds,
            },
        )
    for gripper_path in config.grippers:
        gripper = load_gripper(gripper_path)
        if gripper.name not in config.posture.wrist_up_axes_base:
            raise ValueError(f"Missing wrist mounting-side up axis: {gripper.name}")
        runtime.config = RuntimeConfig(config.device, gripper.calibration.steps_per_second)
        try:
            prepared = prepare_gripper(runtime, gripper, config.cache)
        except (ValueError, FileNotFoundError) as error:
            for item in objects:
                directory = config.output / (gripper.name + "--" + item.name)
                report = {
                    "directory": str(directory),
                    "status": "asset_invalid",
                    "successes": 0,
                    "error": {"type": type(error).__name__, "message": str(error)},
                }
                durable_json(directory / "failure.json", report)
                reports.append(report)
            durable_json(config.output / "progress.json", {"combinations": reports})
            continue
        for item in objects:
            directory = config.output / (gripper.name + "--" + item.name)
            directory.mkdir(parents=True, exist_ok=True)
            if item.name in invalid_objects:
                report = {
                    "directory": str(directory),
                    "status": "asset_invalid",
                    "successes": 0,
                    "error": invalid_objects[item.name],
                }
                durable_json(directory / "failure.json", report)
            else:
                pair = PreparedPair.load(prepared, prepared_objects[item.name])
                report = generate_pair(runtime, pair, config, directory)
            reports.append(report)
            durable_json(config.output / "progress.json", {"combinations": reports})
    return reports


def pair_report(directory: Path, manifest: dict[str, Any]) -> dict[str, Any]:
    return {
        "directory": str(directory),
        **{
            key: manifest[key]
            for key in ("status", "successes", "attempted", "elapsed_s", "failures", "stop_reason")
        },
    }


def generate_pair(
    runtime: PhysxRuntime, pair: PreparedPair, config: RunConfig, directory: Path
) -> dict[str, Any]:
    started = time.monotonic()
    implementation = {p.name: file_hash(p) for p in Path(__file__).parent.glob("*.py")}
    run_id = digest(
        {
            "config": config.snapshot,
            "gripper": pair.gripper.name,
            "object": pair.object.name,
            "implementation": implementation,
        }
    )
    manifest_path = directory / "manifest.json"
    if manifest_path.exists():
        manifest = json.loads(manifest_path.read_text())
        if manifest["format"] != FORMAT or manifest["run_id"] != run_id:
            raise ValueError(f"Resume configuration, implementation or asset mismatch: {directory}")
        arrays = read_shards(directory, manifest)
        if manifest["status"] != "running":
            return pair_report(directory, manifest)
        successful = pose_matrices(arrays["pose_object_tcp_xyz_xyzw"])
        joints = arrays["actual_joint_positions_m"][:, 0]
        manifest["resume_count"] += 1
    else:
        successful = np.empty((0, 4, 4))
        joints = np.empty((0, len(pair.definition["joint_names"])))
        manifest = {
            "format": FORMAT,
            "run_id": run_id,
            "status": "running",
            "stop_reason": "",
            "worker_verified": False,
            "gripper_cache": str(pair.gripper.resolve()),
            "object_cache": str(pair.object.resolve()),
            "gripper": pair.gripper_manifest,
            "object": pair.object_manifest,
            "config": config.snapshot,
            "protocol": asdict(config.validation),
            "implementation": implementation,
            "coordinates": {
                "pose_layout": ["x", "y", "z", "qx", "qy", "qz", "qw"],
                "pose_object_tcp_xyz_xyzw": "TCP in the original object root; stable close trial 0",
                "quaternion": "unit xyzw; canonical sign; active B-to-A rotation for pose_A_B",
                "object": pair.object_manifest["config"]["root_prim"],
                "tcp": pair.definition["tcp"],
                "units": "m,s,kg,N,rad",
                "initialization": (
                    "upright dynamic object with seeded yaw; gravity off reset/approach/close"
                ),
                "pickup_posture": asdict(config.posture),
                "inversion": "post-pickup holding validation rotates pi about world X",
            },
            "stage_names": STAGES,
            "metric_names": METRICS,
            "failure_names": FAILURES,
            "stage_status_codes": {"unexecuted": 0, "passed": 1, "failed": -1},
            "shards": [
                write_arrays(
                    directory / "grasps-00000.npz", empty_result_arrays(pair, config.validation)
                )
            ],
            "successes": 0,
            "attempted": 0,
            "failures": {},
            "duplicates": 0,
            "elapsed_s": 0.0,
            "resume_count": 0,
            "sampling_state": {"round": 0, "cursor": 0},
            "geometric_rejections": 0,
            "posture_rejections": 0,
            "surface_candidates_consumed": 0,
        }
        commit_checkpoint(directory, manifest)
    referenced = {shard["path"] for shard in manifest["shards"]}
    for orphan in directory.glob("grasps-*"):
        if orphan.name not in referenced:
            orphan.unlink()
    elapsed = manifest["elapsed_s"]
    deadline = started + max(0, config.time_budget_s - elapsed)
    state = manifest["sampling_state"]
    sampler = Sampler(
        pair,
        config.sampling,
        config.posture,
        config.seed,
        f"cuda:{config.device}",
        state["round"],
        state["cursor"],
    )
    scene = GraspScene(runtime, pair, config.environments, config.validation)
    failures: Counter[str] = Counter(manifest["failures"])
    active = pair.definition["calibration"]["active_index"]
    commands = pair.arrays["joint_positions_m"][:, active]
    order = np.argsort(commands)
    successful_openings = np.interp(
        joints[:, active], commands[order], pair.arrays["opening_m"][order]
    )
    prior_consumed = manifest["surface_candidates_consumed"]
    prior_rejected = manifest["geometric_rejections"]
    prior_posture = manifest["posture_rejections"]
    while (
        len(successful) < config.target_successes
        and manifest["attempted"] < config.candidate_budget
        and time.monotonic() < deadline
    ):
        batch = sampler.take(
            min(config.environments, config.candidate_budget - manifest["attempted"]), deadline
        )
        manifest.update(
            sampling_state={"round": sampler.round_index, "cursor": sampler.cursor},
            geometric_rejections=prior_rejected + sampler.rejected,
            posture_rejections=prior_posture + sampler.posture_rejected,
            surface_candidates_consumed=prior_consumed + sampler.consumed,
        )
        if not len(batch):
            manifest["stop_reason"] = (
                "time_budget" if time.monotonic() >= deadline else "surface_rounds_exhausted"
            )
            break
        if len(batch) != scene.count:
            scene = GraspScene(runtime, pair, len(batch), config.validation)
        seeds, acceleration = trial_conditions(batch, config.validation, config.seed)
        initial = np.tile(np.eye(4), (len(batch), 1, 1))
        up = np.asarray(config.posture.object_up_axis)
        alignment, _ = Rotation.align_vectors(np.array([[0.0, 0.0, 1.0]]), up[None, :])
        for i, candidate_id in enumerate(batch.ids):
            yaw = np.random.default_rng(int(candidate_id)).uniform(0, 2 * np.pi)
            initial[i, :3, :3] = (
                Rotation.from_rotvec([0, 0, yaw]).as_matrix() @ alignment.as_matrix()
            )
            initial[i, :3, 3] = scene.origins[i] + config.object_position_m
        result = validate(scene, batch, initial, seeds, acceleration)
        passed = np.flatnonzero(result.passed)
        legal = sampler.valid_closures(result.actual_tcp[passed], result.actual_joints[passed])
        failures["closure_posture_invalid"] += int((~legal).sum())
        posture_diagnostic = directory / "diagnostics" / "closure_posture_invalid.npz"
        if not legal.all() and not posture_diagnostic.exists():
            write_arrays(
                posture_diagnostic, result_arrays(result, passed[np.flatnonzero(~legal)[:1]])
            )
        passed = passed[legal]
        measured_openings = np.interp(
            result.actual_joints[:, 0, active], commands[order], pair.arrays["opening_m"][order]
        )
        combined = np.concatenate((successful, result.actual_tcp[passed, 0]))
        openings = np.concatenate((successful_openings, measured_openings[passed]))
        unique = distinct_indices(combined, openings, config.sampling)
        accepted = passed[unique[unique >= len(successful)] - len(successful)]
        manifest["duplicates"] += len(passed) - len(accepted)
        accepted = accepted[: config.target_successes - len(successful)]
        successful = np.concatenate((successful, result.actual_tcp[accepted, 0]))
        successful_openings = np.concatenate((successful_openings, measured_openings[accepted]))
        batch_number = len(manifest["shards"])
        shard = write_arrays(
            directory / f"grasps-{batch_number:05d}.npz", result_arrays(result, accepted)
        )
        manifest["shards"].append(shard)
        for i in np.flatnonzero(~result.passed):
            trial = int(np.flatnonzero(result.failure[i])[0])
            code = FAILURES[result.failure[i, trial]]
            failures[code] += 1
            diagnostic = directory / "diagnostics" / (code + ".npz")
            if not diagnostic.exists():
                write_arrays(diagnostic, result_arrays(result, np.array([i], dtype=np.int64)))
        if len(accepted) and not (directory / "inspection.png").exists():
            trace_index = int(accepted[0])
            trace = {
                key: value if key == "stage" else value[:, trace_index]
                for key, value in result.trace.items()
            }
            trace.update(result_arrays(result, np.array([trace_index], dtype=np.int64)))
            trace_path = directory / "diagnostics" / "trace.npz"
            write_arrays(trace_path, trace)
            from graspdatagen.reporting import inspect_trace

            inspect_trace(pair, trace_path, directory / "inspection.png")
        manifest.update(
            successes=len(successful),
            attempted=manifest["attempted"] + len(batch),
            failures=dict(failures),
            elapsed_s=elapsed + time.monotonic() - started,
        )
        commit_checkpoint(directory, manifest)
        print(
            f"P3 {directory.name}: {len(successful)}/{config.target_successes}; "
            f"attempted={manifest['attempted']}; round={sampler.round_index}; "
            f"elapsed={manifest['elapsed_s']:.1f}s",
            flush=True,
        )
        if len(successful) >= config.target_successes:
            manifest["stop_reason"] = "target_reached"
            break
    if len(successful) >= config.target_successes:
        manifest["stop_reason"] = "target_reached"
    elif not manifest["stop_reason"]:
        manifest["stop_reason"] = (
            "candidate_budget"
            if manifest["attempted"] >= config.candidate_budget
            else "time_budget"
        )
    manifest["status"] = (
        "target_reached"
        if len(successful) >= config.target_successes
        else "insufficient_valid_grasps"
    )
    manifest["elapsed_s"] = elapsed + time.monotonic() - started
    commit_checkpoint(directory, manifest)
    return pair_report(directory, manifest)
