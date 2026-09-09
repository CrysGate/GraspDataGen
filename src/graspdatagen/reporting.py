"""Inspect real trajectory snapshots using the same prepared collision geometry."""

from __future__ import annotations

from pathlib import Path

import matplotlib
import numpy as np
import trimesh

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from graspdatagen.inspection import fit_axes, frame_plot, mesh_plot
from graspdatagen.records import STAGES, PreparedPair
from graspdatagen.sampling import gripper_meshes


def inspect_trace(pair: PreparedPair, trace_path: Path, output: Path) -> None:
    with np.load(pair.object / "geometry.npz", allow_pickle=False) as geometry:
        object_mesh = trimesh.util.concatenate(
            [
                trimesh.Trimesh(
                    geometry[key], geometry[key.replace("vertices_m", "faces")], process=False
                )
                for key in geometry.files
                if key.startswith("hull_") and key.endswith("vertices_m")
            ]
        )
    with np.load(trace_path, allow_pickle=False) as trace:
        figure = plt.figure(figsize=(18, 10), layout="constrained")
        stages = trace["stage"][:, 0]
        for i, stage in enumerate(np.unique(stages)):
            frame = int(np.flatnonzero(stages == stage)[-1])
            axis = figure.add_subplot(2, 4, i + 1, projection="3d")
            object_world = object_mesh.copy()
            object_world.apply_transform(trace["object"][frame])
            mesh_plot(axis, object_world, "#cf8763")
            base = trace["tcp"][frame] @ np.linalg.inv(pair.arrays["T_B_tcp"])
            meshes = gripper_meshes(pair, trace["joints"][frame])
            for mesh in meshes:
                mesh.apply_transform(base)
                mesh_plot(axis, mesh, "#86aeb7")
            frame_plot(axis, trace["tcp"][frame], 0.025)
            fit_axes(axis, np.vstack([object_world.bounds, *(m.bounds for m in meshes)]))
            axis.set_title(STAGES[stage], fontsize=11)
        figure.savefig(output, dpi=130)
        plt.close(figure)
