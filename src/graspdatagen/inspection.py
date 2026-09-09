"""Reproducible 3D inspection artifacts from the actual prepared geometry/states."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import matplotlib
import numpy as np
import trimesh

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from mpl_toolkits.mplot3d.axes3d import Axes3D

from graspdatagen.geometry import author_mesh, transform_points


def mesh_plot(axis: Axes3D, mesh: trimesh.Trimesh, color: str) -> None:
    collection = Poly3DCollection(
        mesh.triangles, facecolor=color, edgecolor="#364148", linewidth=0.15, alpha=0.85
    )
    axis.add_collection3d(collection)


def frame_plot(axis: Axes3D, transform: np.ndarray, size: float) -> None:
    for i, color in enumerate(("#d13e38", "#259659", "#386acf")):
        points = np.array([transform[:3, 3], transform[:3, 3] + transform[:3, i] * size])
        axis.plot(*points.T, color=color, linewidth=2.5)


def fit_axes(axis: Axes3D, bounds: np.ndarray) -> None:
    center = bounds.mean(axis=0)
    extent = np.ptp(bounds, axis=0).max() * 0.56
    axis.set_xlim(center[0] - extent, center[0] + extent)
    axis.set_ylim(center[1] - extent, center[1] + extent)
    axis.set_zlim(center[2] - extent, center[2] + extent)
    axis.set_box_aspect((1, 1, 1))
    axis.set_xlabel("X (m)")
    axis.set_ylabel("Y (m)")
    axis.set_zlabel("Z (m)")
    axis.tick_params(labelsize=7)
    axis.view_init(elev=25, azim=-55)


def inspect_gripper_3d(directory: Path, definition: dict[str, Any]) -> None:
    from pxr import Gf, Usd, UsdGeom

    from graspdatagen.grippers import pose_matrix

    data = np.load(directory / "definition.npz", allow_pickle=False)
    geometry = np.load(directory / "collision_geometry.npz", allow_pickle=False)
    extraction = definition["extraction"]
    measured = definition["calibration"]
    commands = np.asarray(measured["commands_m"])
    poses = np.asarray(measured["link_poses_xyzw"])
    indices = [
        int(np.argmin(abs(commands - definition["closed_command_m"]))),
        len(commands) // 2,
        int(np.argmin(abs(commands - definition["open_command_m"]))),
    ]
    stage = Usd.Stage.CreateNew(str(directory / "inspection.usda"))
    stage.SetDefaultPrim(UsdGeom.Xform.Define(stage, "/Inspection").GetPrim())
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    figure = plt.figure(figsize=(15, 5), layout="constrained")
    all_bounds = []
    axes = []
    spacing = 0.30
    for column, (name, sample) in enumerate(
        zip(("closed", "middle", "open"), indices, strict=True)
    ):
        axis = figure.add_subplot(1, 3, column + 1, projection="3d")
        axes.append(axis)
        base_index = measured["link_names"].index(extraction["base_body"])
        inverse_base = np.linalg.inv(pose_matrix(poses[sample, base_index]))
        T_B_links = {
            name: inverse_base @ pose_matrix(poses[sample, i])
            for i, name in enumerate(measured["link_names"])
        }
        for i, source_path in enumerate(extraction["source_collider_order"]):
            body = source_path.split("/")[2]
            transform = T_B_links[body] @ np.linalg.inv(
                np.asarray(extraction["source_body_transforms_B"][body])
            )
            points = transform_points(geometry[f"mesh_{i:03d}_vertices_B_m"], transform)
            mesh = trimesh.Trimesh(points, geometry[f"mesh_{i:03d}_faces"], process=False)
            all_bounds.append(mesh.bounds)
            color = "#9aa5ac" if body not in definition["finger_body_names"] else "#c7dce5"
            mesh_plot(axis, mesh, color)
            placed = mesh.copy()
            placed.apply_translation([column * spacing, 0, 0])
            author_mesh(stage, f"/Inspection/{name}/collision_{i:03d}", placed)
        for side, body in enumerate(definition["finger_body_names"]):
            vertices = transform_points(
                data[f"contact_patch_{side}_vertices_body_m"], T_B_links[body]
            )
            patch = trimesh.Trimesh(vertices, data[f"contact_patch_{side}_faces"], process=False)
            mesh_plot(axis, patch, "#1dc6a0")
            patch.apply_translation([column * spacing, 0, 0])
            prim = author_mesh(stage, f"/Inspection/{name}/contact_{side}", patch)
            UsdGeom.Mesh(prim).CreateDisplayColorAttr([(0.1, 0.8, 0.6)])
        tcp = data["T_B_tcp"]
        frame_plot(axis, tcp, 0.025)
        for i, axis_color in enumerate(((0.85, 0.2, 0.2), (0.2, 0.75, 0.3), (0.2, 0.4, 0.9))):
            points = np.array([tcp[:3, 3], tcp[:3, 3] + tcp[:3, i] * 0.025])
            points[:, 0] += column * spacing
            line = UsdGeom.BasisCurves.Define(stage, f"/Inspection/{name}/tcp_axis_{i}")
            line.CreateTypeAttr("linear")
            line.CreateCurveVertexCountsAttr([2])
            line.CreatePointsAttr([Gf.Vec3f(*point) for point in points])
            line.CreateWidthsAttr([0.001])
            line.CreateDisplayColorAttr([axis_color])
        aperture = np.interp(commands[sample], commands, np.asarray(measured["surface_gaps_m"]))
        axis.set_title(
            f"{definition['name']} / {name}\n"
            f"q = {commands[sample] * 1000:.2f} mm, gap = {aperture * 1000:.2f} mm",
            fontsize=11,
        )
    bounds = np.array(
        [np.min(np.vstack(all_bounds), axis=0), np.max(np.vstack(all_bounds), axis=0)]
    )
    for axis in axes:
        fit_axes(axis, bounds)
    figure.savefig(directory / "inspection.png", dpi=160)
    plt.close(figure)
    stage.GetRootLayer().Save()
    geometry.close()
    data.close()


def inspect_object_3d(directory: Path, name: str) -> None:
    with np.load(directory / "geometry.npz", allow_pickle=False) as data:
        source = trimesh.Trimesh(data["surface_vertices_m"], data["surface_faces"], process=False)
        hulls = [
            trimesh.Trimesh(data[key], data[key.replace("vertices_m", "faces")], process=False)
            for key in data.files
            if key.startswith("hull_") and key.endswith("vertices_m")
        ]
    figure = plt.figure(figsize=(10, 5), layout="constrained")
    for column in range(2):
        axis = figure.add_subplot(1, 2, column + 1, projection="3d")
        if column == 0:
            points, ids = trimesh.sample.sample_surface(source, 16000, seed=0)
            light = np.clip(source.face_normals[ids] @ np.array([0.2, -0.5, 0.84]), 0.0, 1.0)
            colors = np.column_stack((0.2 + light * 0.25, 0.45 + light * 0.35, 0.6 + light * 0.3))
            axis.scatter(*points.T, c=colors, s=0.6, depthshade=False)
        else:
            for i, hull in enumerate(hulls):
                mesh_plot(axis, hull, matplotlib.colors.to_hex(plt.get_cmap("tab20")(i % 20)))
        fit_axes(axis, source.bounds)
        axis.set_title(
            f"{name}: " + ("source surface" if column == 0 else "prepared convex collisions")
        )
    figure.savefig(directory / "inspection.png", dpi=160)
    plt.close(figure)
