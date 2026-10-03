"""Offline grouped sampling inspection; does not modify experiment data."""
import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from graspdatagen.sampling_inspection import _load_batches

parser = argparse.ArgumentParser()
parser.add_argument("--run", type=Path, required=True)
args = parser.parse_args()
run = args.run
data = _load_batches(run)
manifest = json.loads((run / "manifest.json").read_text())

pos = data["pose_object_tcp_target_xyz_xyzw"][:, :3]
direction = data["approach_axis_object"]
source = data["source_code"]
passed = data["successful"].astype(bool)
accepted = data["accepted"].astype(bool)

if np.any(accepted & ~passed):
    raise ValueError("Accepted candidate is not successful")
if not np.isin(source, [0, 1]).all():
    raise ValueError("Unknown source code")
if not np.isfinite(pos).all() or not np.isfinite(direction).all():
    raise ValueError("Nonfinite geometry")

saved_ids = []
for shard in manifest["shards"]:
    with np.load(run / shard["path"], allow_pickle=False) as part:
        saved_ids.extend(part["candidate_id"].tolist())
if len(saved_ids) != int(accepted.sum()):
    raise ValueError("Accepted count differs from saved dataset")
if set(saved_ids) != set(data["candidate_id"][accepted].tolist()):
    raise ValueError("Accepted IDs differ from saved dataset")

height = pos[:, 2]
azimuth = np.arctan2(pos[:, 1], pos[:, 0])
direction_azimuth = np.arctan2(direction[:, 1], direction[:, 0])
norm = np.linalg.norm(direction, axis=1)
if np.any(norm <= 0):
    raise ValueError("Zero approach direction")
elevation = np.arcsin(np.clip(direction[:, 2] / norm, -1, 1))

lo, hi = float(height.min()), float(height.max())
if hi == lo:
    hi = lo + 1e-6
height_edges = np.linspace(lo, hi, 13)
angle_edges = np.linspace(-np.pi, np.pi, 25)
elevation_edges = np.linspace(-np.pi / 2, np.pi / 2, 13)

stages = [
    ("all", np.ones(len(pos), dtype=bool)),
    ("passed", passed),
    ("accepted", accepted),
]
colors = ["#2774ae", "#e07a1f"]
summary = {
    "candidate_count": len(pos),
    "successful_count": int(passed.sum()),
    "accepted_count": int(accepted.sum()),
    "accepted_ids_match_dataset": True,
    "position_height_edges_m": height_edges.tolist(),
    "azimuth_edges_rad": angle_edges.tolist(),
    "approach_elevation_edges_rad": elevation_edges.tolist(),
    "posture_constraints": manifest["config"].get("posture", {}),
    "coverage_note": (
        "TCP target positions, not surface contacts. "
        "Direction occupancy uses global angular bins, not feasible-domain coverage. "
        "Elevation bins are not equal solid angle."
    ),
    "groups": {},
}

# Rows: base/yaw. Columns: all/passed/accepted.
# Counts and acceptance ratio share exactly the same position bins.
fig, axes = plt.subplots(2, 3, figsize=(15, 8), layout="constrained")
count_grids = {}
for code, name in enumerate(("base", "yaw")):
    summary["groups"][name] = {}
    for stage, stage_mask in stages:
        mask = (source == code) & stage_mask
        grid = np.histogram2d(
            height[mask], azimuth[mask], bins=(height_edges, angle_edges)
        )[0]
        count_grids[name, stage] = grid
        directions = np.histogram2d(
            elevation[mask], direction_azimuth[mask],
            bins=(elevation_edges, angle_edges),
        )[0]
        n = int(mask.sum())
        summary["groups"][name][stage] = {
            "count": n,
            "position_bins_occupied": int((grid > 0).sum()),
            "direction_bins_occupied": int((directions > 0).sum()),
            "position_counts": grid.astype(int).tolist(),
            "direction_counts": directions.astype(int).tolist(),
            "position_fraction": (grid / max(n, 1)).tolist(),
        }

vmax = max(1, max(g.max() for g in count_grids.values()))
for row, name in enumerate(("base", "yaw")):
    for col, (stage, _) in enumerate(stages):
        ax = axes[row, col]
        image = ax.pcolormesh(
            angle_edges, height_edges, count_grids[name, stage],
            vmin=0, vmax=vmax, cmap="viridis",
        )
        n = summary["groups"][name][stage]["count"]
        ax.set_title(f"{name} / {stage}: n={n}")
        ax.set_xlabel("TCP azimuth (rad)")
        ax.set_ylabel("TCP target Z (m)")
fig.colorbar(image, ax=axes.ravel().tolist(), label="count per bin")
fig.suptitle("TCP height × azimuth — shared bins and count scale")
fig.savefig(run / "sampling-groups-counts.png", dpi=170)
plt.close(fig)


# Group-normalized TCP position distributions with a shared fraction scale.
fraction_grids = {}
for name in ("base", "yaw"):
    for stage, _ in stages:
        n = summary["groups"][name][stage]["count"]
        fraction_grids[name, stage] = count_grids[name, stage] / max(n, 1)

fraction_max = max(
    1e-12, max(float(grid.max()) for grid in fraction_grids.values())
)
fig, axes = plt.subplots(2, 3, figsize=(15, 8), layout="constrained")
for row, name in enumerate(("base", "yaw")):
    for col, (stage, _) in enumerate(stages):
        ax = axes[row, col]
        n = summary["groups"][name][stage]["count"]
        image = ax.pcolormesh(
            angle_edges, height_edges, fraction_grids[name, stage],
            vmin=0, vmax=fraction_max, cmap="viridis",
        )
        ax.set_title(f"{name} / {stage}: n={n}")
        ax.set_xlabel("TCP azimuth (rad)")
        ax.set_ylabel("TCP target Z (m)")
        if n == 0:
            ax.text(
                .5, .5, "No candidates", transform=ax.transAxes,
                ha="center", va="center", color="white",
            )
fig.colorbar(
    image, ax=axes.ravel().tolist(),
    label="fraction within source / stage",
)
fig.suptitle("TCP position distributions — shared bins and fraction scale")
fig.savefig(run / "sampling-groups-position-fractions.png", dpi=170)
plt.close(fig)

# Compare group-normalized height, azimuth and direction distributions.
fig, axes = plt.subplots(3, 3, figsize=(15, 11), layout="constrained")
for row, (stage, stage_mask) in enumerate(stages):
    for code, name in enumerate(("base", "yaw")):
        mask = (source == code) & stage_mask
        n = int(mask.sum())
        weights = np.full(n, 1 / max(n, 1))
        axes[row, 0].hist(
            height[mask], bins=height_edges, weights=weights,
            histtype="step", color=colors[code], label=f"{name}: n={n}",
        )
        axes[row, 1].hist(
            azimuth[mask], bins=angle_edges, weights=weights,
            histtype="step", color=colors[code], label=f"{name}: n={n}",
        )
    mask = stage_mask
    hist = np.histogram2d(
        elevation[mask], direction_azimuth[mask],
        bins=(elevation_edges, angle_edges),
    )[0]
    image = axes[row, 2].pcolormesh(
        angle_edges, elevation_edges, hist / max(int(mask.sum()), 1),
        cmap="viridis",
    )
    fig.colorbar(image, ax=axes[row, 2], label="fraction within stage")
    axes[row, 0].set_title(f"{stage}: TCP height")
    axes[row, 1].set_title(f"{stage}: TCP azimuth")
    axes[row, 2].set_title(f"{stage}: approach direction (base + yaw)")
    axes[row, 0].set_xlabel("object Z (m)")
    axes[row, 1].set_xlabel("azimuth (rad)")
    axes[row, 2].set_xlabel("azimuth (rad)")
    axes[row, 2].set_ylabel("elevation (rad)")
    for col in (0, 1):
        axes[row, col].set_ylabel("fraction within source/stage")
        axes[row, col].legend()
fig.savefig(run / "sampling-groups-distributions.png", dpi=170)
plt.close(fig)

# Separate validation pass rate from retention after deduplication.
fig, axes = plt.subplots(2, 2, figsize=(12, 9), layout="constrained")
cmap = plt.get_cmap("viridis").copy()
cmap.set_bad("#dddddd")
for row, name in enumerate(("base", "yaw")):
    for col, (numerator, denominator, title) in enumerate((
        ("passed", "all", "passed / all"),
        ("accepted", "passed", "accepted / passed"),
    )):
        den = count_grids[name, denominator]
        num = count_grids[name, numerator]
        ratio = np.divide(num, den, out=np.zeros_like(den), where=den > 0)
        ratio = np.ma.masked_where(den == 0, ratio)
        ax = axes[row, col]
        image = ax.pcolormesh(
            angle_edges, height_edges, ratio, cmap=cmap, vmin=0, vmax=1,
        )
        ax.set_title(f"{name}: {title}")
        ax.set_xlabel("TCP azimuth (rad)")
        ax.set_ylabel("TCP target Z (m)")
fig.colorbar(image, ax=axes.ravel().tolist(), label="fraction")
fig.suptitle("Grey = no denominator samples; yaw expands successful base grasps")
fig.savefig(run / "sampling-groups-acceptance.png", dpi=170)
plt.close(fig)

# Separate approach-direction distributions by source and stage.
fig, axes = plt.subplots(2, 3, figsize=(15, 8), layout="constrained")
direction_grids = {}
for name in ("base", "yaw"):
    for stage, _ in stages:
        info = summary["groups"][name][stage]
        grid = np.asarray(info["direction_counts"], dtype=float)
        direction_grids[name, stage] = grid / max(info["count"], 1)

vmax = max(1e-12, max(grid.max() for grid in direction_grids.values()))
for row, name in enumerate(("base", "yaw")):
    for col, (stage, _) in enumerate(stages):
        ax = axes[row, col]
        image = ax.pcolormesh(
            angle_edges, elevation_edges,
            direction_grids[name, stage],
            cmap="viridis", vmin=0, vmax=vmax,
        )
        n = summary["groups"][name][stage]["count"]
        ax.set_title(f"{name} / {stage}: n={n}")
        ax.set_xlabel("Approach azimuth (rad)")
        ax.set_ylabel("Approach elevation (rad)")
fig.colorbar(image, ax=axes.ravel().tolist(), label="fraction within source/stage")
fig.suptitle("Approach directions — shared scale; angular bins are not equal solid angle")
fig.savefig(run / "sampling-groups-directions.png", dpi=170)
plt.close(fig)

summary["rate_definitions"] = {
    "validation_pass_rate": "passed / all",
    "retention_after_validation": "accepted / passed",
    "grey_bins": "denominator is zero",
}
summary["source_note"] = (
    "Yaw candidates expand successful base grasps; "
    "base and yaw pass rates are conditional on different input populations."
)
for name in ("base", "yaw"):
    group = summary["groups"][name]
    total = group["all"]["count"]
    passed_count = group["passed"]["count"]
    accepted_count = group["accepted"]["count"]
    group["validation_pass_rate"] = passed_count / total if total else None
    group["retention_after_validation"] = (
        accepted_count / passed_count if passed_count else None
    )

# True 2D projections of TCP target positions.
fig, axes = plt.subplots(1, 3, figsize=(14, 5), layout="constrained")
for ax, (i, j, title) in zip(
    axes, ((0, 1, "top"), (0, 2, "front"), (1, 2, "side")), strict=True
):
    for code, name in enumerate(("base", "yaw")):
        mask = (source == code) & ~accepted
        ax.scatter(pos[mask, i], pos[mask, j], s=2, alpha=.12, color=colors[code])
        mask = (source == code) & accepted
        ax.scatter(
            pos[mask, i], pos[mask, j], s=7, alpha=.7,
            color=colors[code], label=f"{name} accepted",
        )
    ax.set_title(f"{title}: TCP targets")
    ax.set_xlabel(f"{'XYZ'[i]} (m)")
    ax.set_ylabel(f"{'XYZ'[j]} (m)")
    ax.set_aspect("equal", adjustable="box")
    ax.legend(fontsize=8)
fig.suptitle(
    "TCP target positions: faint = not retained; solid = retained; "
    "blue = base; orange = yaw"
)
fig.savefig(run / "sampling-groups-projections.png", dpi=170)
plt.close(fig)

(run / "sampling-groups.json").write_text(
    json.dumps(summary, indent=2, allow_nan=False) + "\n"
)
print("total:", len(pos), "passed:", int(passed.sum()), "accepted:", int(accepted.sum()))
for name, groups in summary["groups"].items():
    print(name, {stage: groups[stage]["count"] for stage, _ in stages})
print("Saved sampling-groups-*.png and sampling-groups.json under:", run)
