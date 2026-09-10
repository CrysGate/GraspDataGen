"""Atomic numeric shards and checked manifests, with no pickle or implicit success."""

from __future__ import annotations

import json
import os
import time
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from graspdatagen.assets import file_hash
from graspdatagen.config import ValidationProfile, digest
from graspdatagen.geometry import matrix_poses, pose_matrices
from graspdatagen.records import METRICS, STAGES, CandidateBatch, PreparedPair, ValidationBatch

FORMAT = "graspdatagen.grasps.v2"
POSE_FIELDS = {
    "T_object_tcp_target": "pose_object_tcp_target_xyz_xyzw",
    "T_object_tcp_pregrasp": "pose_object_tcp_pregrasp_xyz_xyzw",
    "T_object_tcp": "pose_object_tcp_xyz_xyzw",
    "T_object_tcp_trials": "pose_object_tcp_trials_xyz_xyzw",
    "T_world_object_initial": "pose_world_object_initial_xyz_xyzw",
}


def sync_directory(directory: Path) -> None:
    descriptor = os.open(directory, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def durable_json(path: Path, data: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(".json.tmp")
    with temporary.open("w") as stream:
        json.dump(data, stream, indent=2, allow_nan=False)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    temporary.replace(path)
    sync_directory(path.parent)


def commit_checkpoint(directory: Path, manifest: dict[str, Any]) -> None:
    """The supervisor publishes only after draining the worker's physics error log.

    A killed worker can leave a transaction or orphan shard; neither advances
    the committed sampler cursor. The next worker resumes the acknowledged manifest.
    """
    manifest["checkpoint_id"] = digest({k: v for k, v in manifest.items() if k != "checkpoint_id"})
    durable_json(directory / "transaction.json", manifest)
    while True:
        if os.getppid() != int(os.environ["GRASPDATAGEN_SUPERVISOR_PID"]):
            raise RuntimeError("Supervisor exited before acknowledging this batch")
        committed = directory / "manifest.json"
        if committed.exists():
            saved = json.loads(committed.read_text())
            if saved["checkpoint_id"] == manifest["checkpoint_id"]:
                return
        time.sleep(0.1)


def acknowledge_checkpoints(root: Path) -> list[Path]:
    completed: list[Path] = []
    for transaction in root.glob("*/transaction.json"):
        manifest = json.loads(transaction.read_text())
        destination = transaction.with_name("manifest.json")
        if destination.exists():
            saved = json.loads(destination.read_text())
            if saved["checkpoint_id"] == manifest["checkpoint_id"]:
                continue
        durable_json(destination, manifest)
        if manifest["status"] != "running":
            completed.append(transaction.parent)
    return completed


def write_arrays(path: Path, arrays: dict[str, np.ndarray]) -> dict[str, Any]:
    path.parent.mkdir(parents=True, exist_ok=True)
    if any(value.dtype.hasobject or not np.isfinite(value).all() for value in arrays.values()):
        raise ValueError("A data shard must contain only finite numeric arrays")
    temporary = path.with_suffix(".tmp")
    with temporary.open("wb") as stream:
        np.savez_compressed(stream, allow_pickle=False, **arrays)
        stream.flush()
        os.fsync(stream.fileno())
    with np.load(temporary, allow_pickle=False) as saved:
        if any(not np.array_equal(saved[key], value) for key, value in arrays.items()):
            raise RuntimeError("NPZ round-trip differs")
    temporary.replace(path)
    sync_directory(path.parent)
    return {
        "path": path.name,
        "sha256": file_hash(path),
        "arrays": {
            key: {"dtype": str(value.dtype), "shape": list(value.shape)}
            for key, value in arrays.items()
        },
    }


def result_arrays(result: ValidationBatch, indices: np.ndarray) -> dict[str, np.ndarray]:
    batch = result.candidates.select(indices)
    return {
        "candidate_id": batch.ids,
        "pose_object_tcp_target_xyz_xyzw": matrix_poses(batch.target),
        "pose_object_tcp_pregrasp_xyz_xyzw": matrix_poses(batch.pregrasp),
        "pose_object_tcp_xyz_xyzw": matrix_poses(result.actual_tcp[indices, 0]),
        "pose_object_tcp_trials_xyz_xyzw": matrix_poses(result.actual_tcp[indices]),
        "pregrasp_opening_m": batch.opening,
        "contact_width_m": batch.contact_width,
        "pregrasp_joint_positions_m": batch.pregrasp_joints,
        "close_command_m": batch.close_command,
        "actual_joint_positions_m": result.actual_joints[indices],
        "pose_world_object_initial_xyz_xyzw": matrix_poses(result.initial_object[indices]),
        "trial_seed": result.seeds[indices],
        "disturbance_accelerations_world_m_s2": result.accelerations[indices],
        "trial_failure_code": result.failure[indices],
        "stage_status": result.stage_status[indices],
        "stage_metrics": result.metrics[indices],
    }


def candidates_from_arrays(arrays: dict[str, np.ndarray]) -> CandidateBatch:
    return CandidateBatch(
        arrays["candidate_id"],
        pose_matrices(arrays["pose_object_tcp_target_xyz_xyzw"]),
        pose_matrices(arrays["pose_object_tcp_pregrasp_xyz_xyzw"]),
        arrays["pregrasp_opening_m"],
        arrays["contact_width_m"],
        arrays["pregrasp_joint_positions_m"],
        arrays["close_command_m"],
    )


def empty_result_arrays(pair: PreparedPair, profile: ValidationProfile) -> dict[str, np.ndarray]:
    """A geometry-exhausted run still publishes a well-shaped zero-row shard."""
    dofs = len(pair.definition["joint_names"])
    batch = CandidateBatch(
        np.empty(0, dtype=np.int64),
        np.empty((0, 4, 4)),
        np.empty((0, 4, 4)),
        np.empty(0),
        np.empty(0),
        np.empty((0, dofs)),
        np.empty(0),
    )
    result = ValidationBatch(
        batch,
        np.empty((0, profile.trials), dtype=np.int64),
        np.empty((0, profile.trials, len(STAGES)), dtype=np.int64),
        np.empty((0, profile.trials, len(STAGES), len(METRICS))),
        np.empty((0, profile.trials, 4, 4)),
        np.empty((0, profile.trials, dofs)),
        np.empty((0, 4, 4)),
        np.empty((0, profile.trials), dtype=np.uint64),
        np.empty((0, profile.trials, 6 + profile.random_directions, 3)),
        {},
    )
    return result_arrays(result, np.empty(0, dtype=np.int64))


def load_dataset(directory: Path) -> tuple[dict[str, Any], dict[str, np.ndarray]]:
    manifest = json.loads((directory / "manifest.json").read_text())
    if not manifest["worker_verified"]:
        raise ValueError("Dataset has not passed worker/log verification")
    return manifest, read_shards(directory, manifest)


def export_grasps_yaml(directory: Path) -> Path:
    """Export verified successes; approach axes follow actual closure in object coordinates."""
    manifest, arrays = load_dataset(directory)
    robot_path = Path(manifest["gripper_cache"]) / "robot_snapshot.json"
    if file_hash(robot_path) != manifest["gripper"]["artifacts"]["robot_snapshot.json"]:
        raise ValueError("Robot snapshot differs from the recorded asset")
    robot = json.loads(robot_path.read_text())["name"]
    poses = arrays["pose_object_tcp_xyz_xyzw"]
    axis_tcp = np.asarray(manifest["coordinates"]["tcp"]["approach_axis_tcp"])
    axes = pose_matrices(poses)[:, :3, :3] @ axis_tcp
    axes /= np.linalg.norm(axes, axis=1, keepdims=True)
    data = {
        "format": "graspdatagen.grasps.compact.v1",
        "object": manifest["object"]["name"],
        "position_unit": "m",
        "pose_layout": manifest["coordinates"]["pose_layout"],
        "tcp": manifest["coordinates"]["tcp"]["configuration"],
        "approach_distance_m": manifest["config"]["sampling"]["pregrasp_distance_m"],
        "candidates": [
            {
                "candidate_id": index,
                "robot": robot,
                "pose_object_tcp_xyz_xyzw": pose.tolist(),
                "approach_axis_object": axis.tolist(),
                "closed_joint_positions_m": dict(
                    zip(
                        manifest["gripper"]["definition"]["joint_names"],
                        arrays["actual_joint_positions_m"][index, 0].tolist(),
                        strict=True,
                    )
                ),
            }
            for index, (pose, axis) in enumerate(zip(poses, axes, strict=True))
        ],
    }
    path = directory / "grasps.yaml"
    temporary = path.with_suffix(".yaml.tmp")
    with temporary.open("w") as stream:
        yaml.safe_dump(data, stream, sort_keys=False, default_flow_style=None, width=120)
        stream.flush()
        os.fsync(stream.fileno())
    temporary.replace(path)
    sync_directory(directory)
    return path


def read_shards(directory: Path, manifest: dict[str, Any]) -> dict[str, np.ndarray]:
    """Read committed data for replay or resume; v1 is read only for existing P2 evidence."""
    if manifest["format"] not in (FORMAT, "graspdatagen.grasps.v1"):
        raise ValueError("Unsupported grasp dataset format")
    parts: list[dict[str, np.ndarray]] = []
    for shard in manifest["shards"]:
        path = directory / shard["path"]
        if file_hash(path) != shard["sha256"]:
            raise ValueError(f"Corrupt shard: {path}")
        with np.load(path, allow_pickle=False) as arrays:
            if set(arrays.files) != set(shard["arrays"]):
                raise ValueError("Shard fields disagree with manifest")
            part = {key: arrays[key] for key in arrays.files}
        for key, value in part.items():
            expected = shard["arrays"][key]
            if (
                str(value.dtype) != expected["dtype"]
                or list(value.shape) != expected["shape"]
                or value.dtype.hasobject
                or not np.isfinite(value).all()
            ):
                raise ValueError(f"Invalid shard array: {key}")
        if (part["trial_failure_code"] != 0).any() or (part["stage_status"] != 1).any():
            raise ValueError("Formal dataset contains an unvalidated grasp")
        if manifest["format"] == "graspdatagen.grasps.v1":
            for old, new in POSE_FIELDS.items():
                part[new] = matrix_poses(part.pop(old))
        count = len(part["candidate_id"])
        trials = manifest["protocol"]["trials"]
        dofs = part["pregrasp_joint_positions_m"].shape[-1]
        shapes = {
            "candidate_id": (count,),
            "pregrasp_opening_m": (count,),
            "contact_width_m": (count,),
            "pregrasp_joint_positions_m": (count, dofs),
            "close_command_m": (count,),
            "actual_joint_positions_m": (count, trials, dofs),
            "trial_seed": (count, trials),
            "trial_failure_code": (count, trials),
            "stage_status": (count, trials, len(STAGES)),
            "stage_metrics": (count, trials, len(STAGES), len(METRICS)),
            "disturbance_accelerations_world_m_s2": (
                count,
                trials,
                6 + manifest["protocol"]["random_directions"],
                3,
            ),
        }
        if set(part) != set(shapes) | set(POSE_FIELDS.values()):
            raise ValueError("Formal shard fields differ from the numeric schema")
        for key, shape in shapes.items():
            dtype = (
                np.int64
                if key in ("candidate_id", "trial_failure_code", "stage_status")
                else np.uint64
                if key == "trial_seed"
                else np.float64
            )
            if part[key].shape != shape or part[key].dtype != dtype:
                raise ValueError(f"Invalid formal array: {key}")
        if (part["candidate_id"] <= 0).any():
            raise ValueError("Candidate IDs must be positive")
        for key in POSE_FIELDS.values():
            shape = (count, trials, 7) if key == "pose_object_tcp_trials_xyz_xyzw" else (count, 7)
            if part[key].shape != shape or part[key].dtype != np.float64:
                raise ValueError(f"Invalid pose field: {key}")
            pose_matrices(part[key])
        if part["stage_status"].shape != (count, trials, len(STAGES)):
            raise ValueError("Invalid stage status shape")
        parts.append(part)
    if not parts:
        raise ValueError("Dataset has no shards")
    merged = {key: np.concatenate([part[key] for part in parts]) for key in parts[0]}
    ids = merged["candidate_id"]
    if len(ids) != manifest["successes"] or len(np.unique(ids)) != len(ids):
        raise ValueError("Dataset count/identity mismatch")
    return merged
