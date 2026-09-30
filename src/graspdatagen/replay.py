"""Reload committed poses and commands and execute the generation validator."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np

from graspdatagen.assets import write_json
from graspdatagen.config import ValidationProfile
from graspdatagen.geometry import pose_matrices
from graspdatagen.records import FAILURES, PreparedPair
from graspdatagen.runtime import GraspScene, PhysxRuntime
from graspdatagen.storage import candidates_from_arrays, load_dataset, result_arrays, write_arrays
from graspdatagen.validation import validate


def replay(
    runtime: PhysxRuntime, directory: Path, output: Path, grasp_id: int, environments: int
) -> dict[str, Any]:
    """grasp_id=0 replays the full dataset; a saved positive ID selects one grasp."""
    manifest, arrays = load_dataset(directory)
    pair = PreparedPair.load(Path(manifest["gripper_cache"]), Path(manifest["object_cache"]))
    profile = ValidationProfile(**manifest["protocol"])
    if profile.steps_per_second != runtime.config.steps_per_second:
        raise ValueError("Replay timestep differs from the saved protocol")
    all_candidates = candidates_from_arrays(arrays)
    indices = np.arange(len(all_candidates), dtype=np.int64)
    if grasp_id:
        indices = np.flatnonzero(all_candidates.ids == grasp_id)
        if len(indices) != 1:
            raise ValueError(f"Unknown or nonunique grasp ID: {grasp_id}")
    if not len(indices):
        raise ValueError("No successful data to replay")
    scene = GraspScene(runtime, pair, min(environments, len(indices)), profile)
    report: dict[str, Any] = {
        "run": str(directory),
        "environments": environments,
        "requested": len(indices),
        "passed": 0,
        "results": [],
        "layout": "grid" if runtime.gui else "saved",
    }
    for start in range(0, len(indices), environments):
        selected = indices[start : start + environments]
        if len(selected) != scene.count:
            scene = GraspScene(runtime, pair, len(selected), profile)
        initial_object = pose_matrices(arrays["pose_world_object_initial_xyz_xyzw"][selected])
        if runtime.gui:
            # Saved successes can come from the same cell in different batches.
            # Separate them for viewing while preserving every relative grasp pose.
            initial_object[:, :3, 3] = scene.origins + manifest["config"]["object_position_m"]
        result = validate(
            scene,
            all_candidates.select(selected),
            initial_object,
            arrays["trial_seed"][selected],
            arrays["disturbance_accelerations_world_m_s2"][selected],
        )
        report["passed"] += int(result.passed.sum())
        for i, candidate_id in enumerate(result.candidates.ids):
            report["results"].append(
                {
                    "candidate_id": int(candidate_id),
                    "passed": bool(result.passed[i]),
                    "trials": [FAILURES[c] for c in result.failure[i]],
                    "stage_status": result.stage_status[i].tolist(),
                }
            )
        write_arrays(
            output.parent / (output.stem + f"-batch-{start:05d}.npz"),
            result_arrays(result, np.arange(len(selected), dtype=np.int64)),
        )
        write_json(output.with_suffix(".progress.json"), report)
        print(
            f"P2 replay {directory.name}: {report['passed']}/{start + len(selected)} passed",
            flush=True,
        )
    report["checks_passed"] = report["passed"] == len(indices)
    return report
