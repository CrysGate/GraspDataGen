"""Optional, offline inspection of candidates immediately before PhysX validation."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import matplotlib
import numpy as np
import trimesh

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from graspdatagen.geometry import matrix_poses
from graspdatagen.records import FAILURES, CandidateBatch, PreparedPair, ValidationBatch
from graspdatagen.storage import durable_json, write_arrays

SOURCE_NAMES = ("base", "yaw")
OUTCOME_NAMES = (*FAILURES, "closure_posture_invalid")


def write_sampling_inspection_batch(
    directory: Path,
    pair: PreparedPair,
    batch: CandidateBatch,
    result: ValidationBatch,
    successful: np.ndarray,
    accepted: np.ndarray,
    source: str,
    round_index: int,
    batch_number: int,
) -> Path:
    """Persist continuous pre-validation inputs and their eventual P2 outcomes.

    This diagnostic is enabled only by ``sampling_inspection: true``. It is kept
    below diagnostics/ so production checkpoint discovery and resume remain unchanged.
    """
    if source not in SOURCE_NAMES:
        raise ValueError(f"Unknown sampling source: {source}")
    if successful.shape != (len(batch),):
        raise ValueError("Sampling inspection success mask disagrees with candidate batch")

    accepted_mask = np.zeros(len(batch), dtype=np.bool_)
    accepted_mask[accepted] = True
    outcome = np.zeros(len(batch), dtype=np.int16)
    for index in np.flatnonzero(~result.passed):
        failed_trials = np.flatnonzero(result.failure[index])
        outcome[index] = (
            int(result.failure[index, failed_trials[0]])
            if len(failed_trials)
            else FAILURES.index("solver_invalid")
        )
    outcome[result.passed & ~successful] = len(FAILURES)

    approach_tcp = np.asarray(pair.arrays["approach_axis_tcp"], dtype=np.float64)
    approach = np.einsum("nij,j->ni", batch.target[:, :3, :3], approach_tcp)
    arrays = {
        "candidate_id": batch.ids,
        "pose_object_tcp_target_xyz_xyzw": matrix_poses(batch.target),
        "approach_axis_object": approach,
        "opening_m": batch.opening,
        "source_code": np.full(len(batch), SOURCE_NAMES.index(source), dtype=np.int8),
        "round": np.full(len(batch), round_index, dtype=np.int64),
        "batch": np.full(len(batch), batch_number, dtype=np.int64),
        "successful": successful,
        "accepted": accepted_mask,
        "outcome_code": outcome,
    }
    path = directory / "diagnostics" / "sampling" / f"batch-{batch_number:05d}.npz"
    write_arrays(path, arrays)
    return path


def _load_batches(run: Path) -> dict[str, np.ndarray]:
    paths = sorted((run / "diagnostics" / "sampling").glob("batch-*.npz"))
    if not paths:
        raise ValueError(f"No sampling inspection batches found under {run}")
    parts: dict[str, list[np.ndarray]] = {}
    expected: set[str] | None = None
    for path in paths:
        with np.load(path, allow_pickle=False) as data:
            if expected is None:
                expected = set(data.files)
            elif set(data.files) != expected:
                raise ValueError(f"Sampling inspection fields disagree: {path}")
            for name in data.files:
                parts.setdefault(name, []).append(data[name])
    return {name: np.concatenate(values) for name, values in parts.items()}


def _fit_3d(axis: Any, bounds: np.ndarray) -> None:
    center = bounds.mean(axis=0)
    extent = max(float(np.ptp(bounds, axis=0).max()) * 0.56, 1e-3)
    axis.set_xlim(center[0] - extent, center[0] + extent)
    axis.set_ylim(center[1] - extent, center[1] + extent)
    axis.set_zlim(center[2] - extent, center[2] + extent)
    axis.set_box_aspect((1, 1, 1))
    axis.set_xlabel("X (m)", fontsize=8)
    axis.set_ylabel("Y (m)", fontsize=8)
    axis.set_zlabel("Z (m)", fontsize=8)
    axis.tick_params(labelsize=7)


def _coverage_summary(arrays: dict[str, np.ndarray]) -> dict[str, Any]:
    positions = arrays["pose_object_tcp_target_xyz_xyzw"][:, :3]
    approach = arrays["approach_axis_object"]
    position_azimuth = np.arctan2(positions[:, 1], positions[:, 0])
    approach_azimuth = np.arctan2(approach[:, 1], approach[:, 0])
    approach_elevation = np.arcsin(np.clip(approach[:, 2], -1.0, 1.0))
    position_bins = np.histogram2d(positions[:, 2], position_azimuth, bins=(12, 24))[0]
    direction_bins = np.histogram2d(
        approach_elevation,
        approach_azimuth,
        bins=(12, 24),
        range=((-np.pi / 2, np.pi / 2), (-np.pi, np.pi)),
    )[0]
    return {
        "candidate_count": len(positions),
        "base_count": int((arrays["source_code"] == 0).sum()),
        "yaw_count": int((arrays["source_code"] == 1).sum()),
        "successful_count": int(arrays["successful"].sum()),
        "accepted_count": int(arrays["accepted"].sum()),
        "position_height_azimuth_bins_occupied": int((position_bins > 0).sum()),
        "position_height_azimuth_bins_total": int(position_bins.size),
        "approach_sphere_bins_occupied": int((direction_bins > 0).sum()),
        "approach_sphere_bins_total": int(direction_bins.size),
        "source_codes": dict(enumerate(SOURCE_NAMES)),
        "outcome_codes": dict(enumerate(OUTCOME_NAMES)),
    }


def inspect_sampling(run: Path, output: Path) -> Path:
    """Render pre-validation positions/directions and full-batch distribution statistics."""
    manifest_path = run / "manifest.json"
    if not manifest_path.exists():
        raise ValueError(f"Missing run manifest: {manifest_path}")
    manifest = json.loads(manifest_path.read_text())
    object_cache = Path(manifest["object_cache"])
    with np.load(object_cache / "geometry.npz", allow_pickle=False) as geometry:
        mesh = trimesh.Trimesh(
            geometry["surface_vertices_m"], geometry["surface_faces"], process=False
        )
    arrays = _load_batches(run)
    positions = arrays["pose_object_tcp_target_xyz_xyzw"][:, :3]
    approach = arrays["approach_axis_object"]
    source = arrays["source_code"]
    successful = arrays["successful"]

    figure = plt.figure(figsize=(18, 9), layout="constrained")
    mesh_points, _ = trimesh.sample.sample_surface(
        mesh, min(6000, max(1000, len(mesh.faces))), seed=0
    )
    combined = np.vstack((mesh.bounds, positions))
    bounds = np.array([combined.min(axis=0), combined.max(axis=0)])
    arrow_count = min(1000, len(positions))
    arrow_indices = np.linspace(0, len(positions) - 1, arrow_count, dtype=np.int64)
    arrow_length = max(float(np.ptp(mesh.bounds, axis=0).max()) * 0.075, 0.005)
    colors = np.array(["#2774ae", "#e07a1f"])
    views = ((90, -90, "top"), (0, -90, "front"), (0, 0, "side"))
    for column, (elevation, azimuth, title) in enumerate(views, start=1):
        axis = figure.add_subplot(2, 4, column, projection="3d")
        axis.scatter(*mesh_points.T, color="#9aa5ac", s=0.35, alpha=0.18, depthshade=False)
        for source_code, source_name in enumerate(SOURCE_NAMES):
            for passed, marker, alpha in ((False, "x", 0.28), (True, "o", 0.82)):
                mask = (source == source_code) & (successful == passed)
                if mask.any():
                    axis.scatter(
                        *positions[mask].T,
                        color=colors[source_code],
                        marker=marker,
                        s=9 if passed else 5,
                        alpha=alpha,
                        depthshade=False,
                        label=f"{source_name} / {'pass' if passed else 'fail'}",
                    )
        selected = arrow_indices
        axis.quiver(
            positions[selected, 0],
            positions[selected, 1],
            positions[selected, 2],
            approach[selected, 0],
            approach[selected, 1],
            approach[selected, 2],
            length=arrow_length,
            normalize=True,
            color=colors[source[selected]],
            alpha=0.32,
            linewidth=0.45,
        )
        _fit_3d(axis, bounds)
        axis.view_init(elev=elevation, azim=azimuth)
        axis.set_title(f"{title}: TCP and approach axes")
        if column == 1:
            axis.legend(loc="upper left", fontsize=7)

    axis = figure.add_subplot(2, 4, 5)
    axis.hist(positions[:, 2], bins=30, color="#536d7a", alpha=0.9)
    axis.set_title("TCP height")
    axis.set_xlabel("object Z (m)")
    axis.set_ylabel("candidates")

    axis = figure.add_subplot(2, 4, 6, projection="polar")
    axis.hist(np.arctan2(positions[:, 1], positions[:, 0]), bins=36, color="#536d7a")
    axis.set_title("TCP azimuth")

    axis = figure.add_subplot(2, 4, 7)
    direction_hist, x_edges, y_edges = np.histogram2d(
        np.arctan2(approach[:, 1], approach[:, 0]),
        np.arcsin(np.clip(approach[:, 2], -1.0, 1.0)),
        bins=(36, 18),
        range=((-np.pi, np.pi), (-np.pi / 2, np.pi / 2)),
    )
    image = axis.pcolormesh(x_edges, y_edges, direction_hist.T, cmap="viridis")
    figure.colorbar(image, ax=axis, label="candidates")
    axis.set_title("Approach direction")
    axis.set_xlabel("azimuth (rad)")
    axis.set_ylabel("elevation (rad)")

    axis = figure.add_subplot(2, 4, 8)
    for source_code, source_name in enumerate(SOURCE_NAMES):
        mask = source == source_code
        if mask.any():
            axis.hist(
                arrays["opening_m"][mask],
                bins=30,
                histtype="step",
                linewidth=1.7,
                color=colors[source_code],
                label=source_name,
            )
    axis.set_title("Pregrasp opening")
    axis.set_xlabel("opening (m)")
    axis.set_ylabel("candidates")
    axis.legend()

    output.parent.mkdir(parents=True, exist_ok=True)
    figure.suptitle(f"Pre-PhysX sampling inspection: {run.name}", fontsize=14)
    figure.savefig(output, dpi=170)
    plt.close(figure)
    durable_json(output.with_suffix(".json"), _coverage_summary(arrays))
    return output
