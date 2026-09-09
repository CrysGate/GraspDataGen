"""Extract explicit rigid-body boundaries and calibrate from contact surfaces."""

from __future__ import annotations

import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

import numpy as np
import trimesh
from scipy.spatial.transform import Rotation

from graspdatagen.assets import (
    bind_material,
    cache_identity,
    check_cache,
    check_references,
    file_hash,
    localize_assets,
    open_stage,
    seal_cache,
    source_fingerprint,
    write_json,
)
from graspdatagen.config import GripperConfig, digest
from graspdatagen.geometry import (
    FloatArray,
    mesh_in_frame,
    relative_transform,
    set_transform,
    transform_points,
)
from graspdatagen.runtime import PhysxRuntime


def pose_matrix(pose: FloatArray) -> FloatArray:
    result = np.eye(4)
    result[:3, :3] = Rotation.from_quat(pose[3:]).as_matrix()
    result[:3, 3] = pose[:3]
    return result


def tcp_definition(config: GripperConfig) -> dict[str, Any]:
    from pxr import UsdGeom

    stage = open_stage(config.source, config.root_prim)
    base = stage.GetPrimAtPath(config.root_prim + "/" + config.base_body)
    parent = stage.GetPrimAtPath(config.root_prim + "/" + config.tcp_parent_prim)
    if not parent or not parent.GetPath().HasPrefix(base.GetPath()):
        raise ValueError("TCP parent must be retained in the fixed base frame chain")
    if UsdGeom.GetStageMetersPerUnit(stage) != 1.0:
        raise ValueError("Robot adapters require metre-authored joints and inertias")
    tcp = config.robot_snapshot["kinematics"]["tcp"]
    T_P_tcp = pose_matrix(np.array(tcp["position_m"] + tcp["orientation_xyzw"]))
    T_B_P = relative_transform(parent, base)
    T_B_tcp = T_B_P @ T_P_tcp
    chain = []
    prim = parent
    while prim != base:
        chain.append(
            {
                "prim": str(prim.GetPath()),
                "parent": str(prim.GetParent().GetPath()),
                "T_parent_child": relative_transform(prim, prim.GetParent()).tolist(),
            }
        )
        prim = prim.GetParent()
    chain.reverse()
    ee = stage.GetPrimAtPath(
        config.root_prim + "/" + config.robot_snapshot["kinematics"]["ee_body"]
    )
    return {
        "configuration": tcp,
        "configuration_sha256": digest(tcp),
        "robot_snapshot_sha256": file_hash(config.robot_config),
        "parent_chain": chain,
        "T_B_P": T_B_P.tolist(),
        "T_P_tcp": T_P_tcp.tolist(),
        "T_B_tcp": T_B_tcp.tolist(),
        "T_ee_B": relative_transform(base, ee).tolist(),
        "approach_axis_tcp": (T_B_tcp[:3, :3].T @ config.approach_axis_base).tolist(),
        "opening_axis_tcp": (T_B_tcp[:3, :3].T @ config.opening_axis_base).tolist(),
    }


def extract_gripper(
    runtime: PhysxRuntime, config: GripperConfig, directory: Path
) -> dict[str, Any]:
    from pxr import PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics

    source = open_stage(config.source, config.root_prim)
    source_root = source.GetDefaultPrim()
    base = source.GetPrimAtPath(config.root_prim + "/" + config.base_body)
    stage = Usd.Stage.CreateNew(str(directory / "gripper.usdc"))
    root = UsdGeom.Xform.Define(stage, "/Gripper").GetPrim()
    stage.SetDefaultPrim(root)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    mapping: dict[str, str] = {}
    source_looks = source.GetPrimAtPath(config.root_prim + "/Looks")
    if source_looks:
        Sdf.CopySpec(
            source.GetRootLayer(), source_looks.GetPath(), stage.GetRootLayer(), "/Gripper/Looks"
        )
    for relative in (*config.retained_bodies, *config.retained_joints):
        src = source.GetPrimAtPath(config.root_prim + "/" + relative)
        if not src:
            raise ValueError(f"Missing extraction boundary: {relative}")
        target = Sdf.Path("/Gripper/" + relative)
        stage.DefinePrim(target.GetParentPath(), "Scope")
        Sdf.CopySpec(source.GetRootLayer(), src.GetPath(), stage.GetRootLayer(), target)
        for child in Usd.PrimRange(src):
            mapping[str(child.GetPath())] = str(
                child.GetPath().ReplacePrefix(source_root.GetPath(), root.GetPath())
            )
        if relative in config.retained_bodies:
            if not src.HasAPI(UsdPhysics.RigidBodyAPI):
                raise ValueError(f"Retained body is not rigid: {relative}")
            set_transform(stage.GetPrimAtPath(target), relative_transform(src, base))
    for prim in stage.Traverse():
        for relationship in list(prim.GetRelationships()):
            relationship.SetTargets(
                [
                    target.ReplacePrefix(source_root.GetPath(), root.GetPath())
                    for target in relationship.GetTargets()
                ]
            )
        for attribute in prim.GetAttributes():
            connections = attribute.GetConnections()
            if connections:
                attribute.SetConnections(
                    [
                        target.ReplacePrefix(source_root.GetPath(), root.GetPath())
                        for target in connections
                    ]
                )
    # Keep the source visual and native collision prims in one portable USD. The
    # cooked hull arrays are a numeric cache for calibration and CPU-side sampling.
    runtime.new_scene()
    runtime.reference(config.source, "/World/source")
    collider_paths = [
        str(p.GetPath())
        for p in stage.Traverse()
        if p.HasAPI(UsdPhysics.CollisionAPI)
        and UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get()
    ]
    source_geometry: dict[str, np.ndarray] = {}
    collision_geometry: dict[str, np.ndarray] = {}
    retained_colliders: dict[str, list[str]] = {}
    for index, path in enumerate(collider_paths):
        relative = path[len("/Gripper/") :]
        original = source.GetPrimAtPath(config.root_prim + "/" + relative)
        mesh = mesh_in_frame(original, base)
        source_geometry[f"mesh_{index:03d}_vertices_B_m"] = np.asarray(mesh.vertices)
        source_geometry[f"mesh_{index:03d}_faces"] = np.asarray(mesh.faces, dtype=np.int64)
        runtime_path = "/World/source/" + relative
        hulls = runtime.cook_convexes(runtime_path)
        source_to_base = relative_transform(original, base)
        retained_colliders[relative] = [path]
        merged = trimesh.util.concatenate(hulls)
        merged.apply_transform(source_to_base)
        collision_geometry[f"mesh_{index:03d}_vertices_B_m"] = np.asarray(merged.vertices)
        collision_geometry[f"mesh_{index:03d}_faces"] = np.asarray(merged.faces, dtype=np.int64)
    for prim in reversed(list(stage.Traverse())):
        if prim.IsA(UsdGeom.Camera):
            stage.RemovePrim(prim.GetPath())
    all_colliders = [stage.GetPrimAtPath(p) for paths in retained_colliders.values() for p in paths]
    bind_material(stage, all_colliders, config.material)
    joint = UsdPhysics.FixedJoint.Define(stage, "/Gripper/root_joint")
    joint.CreateBody1Rel().SetTargets(["/Gripper/" + config.base_body])
    UsdPhysics.ArticulationRootAPI.Apply(joint.GetPrim())
    api = PhysxSchema.PhysxArticulationAPI.Apply(joint.GetPrim())
    api.CreateEnabledSelfCollisionsAttr(False)
    localize_assets(stage, directory)
    stage.GetRootLayer().Save()
    np.savez_compressed(directory / "source_geometry.npz", allow_pickle=False, **source_geometry)
    np.savez_compressed(
        directory / "collision_geometry.npz", allow_pickle=False, **collision_geometry
    )
    tcp = tcp_definition(config)
    parent = stage.GetPrimAtPath("/Gripper/" + config.tcp_parent_prim)
    derived_base = stage.GetPrimAtPath("/Gripper/" + config.base_body)
    if not np.allclose(relative_transform(parent, derived_base), tcp["T_B_P"], atol=1e-9):
        raise RuntimeError("Extracted TCP parent chain changed")
    urdf = Path(config.robot_snapshot["urdf_path"])
    urdf_joints = ET.parse(urdf).getroot().findall("joint")
    limits = {}
    for urdf_joint in urdf_joints:
        if urdf_joint.attrib["name"] in config.robot_snapshot["gripper"]["joint_names"]:
            limit = urdf_joint.find("limit")
            if limit is None:
                raise ValueError("Selected URDF joint has no limits")
            limits[urdf_joint.attrib["name"]] = limit.attrib
    return {
        "prim_mapping": mapping,
        "base_body": config.base_body,
        "colliders": retained_colliders,
        "source_body_transforms_B": {
            name: relative_transform(
                source.GetPrimAtPath(config.root_prim + "/" + name), base
            ).tolist()
            for name in config.retained_bodies
        },
        "source_collider_order": collider_paths,
        "retained_bodies": list(config.retained_bodies),
        "retained_joints": list(config.retained_joints),
        "tcp": tcp,
        "urdf_joint_limits": limits,
        "urdf_sha256": file_hash(urdf),
        "display": "source visual surfaces and materials; native collision prims retained",
    }


def contact_patch(
    mesh: trimesh.Trimesh, config: GripperConfig, inward_sign: float
) -> trimesh.Trimesh:
    approach = np.asarray(config.approach_axis_base)
    opening = np.asarray(config.opening_axis_base)
    lateral = np.cross(approach, opening)
    clipped = mesh.copy()
    for axis, interval in (
        (approach, config.approach_interval_m),
        (lateral, config.lateral_interval_m),
    ):
        for origin, normal in ((axis * interval[0], axis), (axis * interval[1], -axis)):
            vertices, faces, _ = trimesh.intersections.slice_faces_plane(
                clipped.vertices,
                clipped.faces,
                normal,
                origin,
            )
            clipped = trimesh.Trimesh(vertices, faces, process=False)
    mask = clipped.face_normals @ (inward_sign * opening) >= config.normal_alignment_min
    if not mask.any():
        raise ValueError("No inward-facing surface in configured contact region")
    patch = clipped.submesh([np.flatnonzero(mask)], append=True)
    if not isinstance(patch, trimesh.Trimesh):
        raise ValueError("Contact surface did not produce a single mesh")
    return patch


def calibrate_definition(
    runtime: PhysxRuntime, config: GripperConfig, directory: Path, extraction: dict[str, Any]
) -> dict[str, Any]:
    from pxr import UsdPhysics

    robot = config.robot_snapshot["gripper"]
    active = robot["command_joint_names"][0]
    follower = next(name for name in robot["joint_names"] if name != active)
    measured = runtime.measure_gripper(
        directory / "gripper.usdc", active, follower, config.calibration
    )
    source = open_stage(config.source, config.root_prim)
    base = source.GetPrimAtPath(config.root_prim + "/" + config.base_body)
    body_names = robot["finger_body_names"]
    with np.load(directory / "collision_geometry.npz", allow_pickle=False) as geometry:
        finger_indices = [
            extraction["source_collider_order"].index("/Gripper/" + p)
            for p in config.finger_colliders
        ]
        finger_meshes = [
            trimesh.Trimesh(
                geometry[f"mesh_{i:03d}_vertices_B_m"],
                geometry[f"mesh_{i:03d}_faces"],
                process=False,
            )
            for i in finger_indices
        ]
    centers = np.array(
        [np.mean(mesh.vertices @ config.opening_axis_base) for mesh in finger_meshes]
    )
    order = np.argsort(centers)
    signs = np.where(centers < centers.mean(), 1.0, -1.0)
    patches = [
        contact_patch(mesh, config, float(sign))
        for mesh, sign in zip(finger_meshes, signs, strict=True)
    ]
    patch_vertices_body = []
    for patch, name in zip(patches, body_names, strict=True):
        body = source.GetPrimAtPath(config.root_prim + "/" + name)
        T_body_B = np.asarray(np.linalg.inv(relative_transform(body, base)), dtype=np.float64)
        patch_vertices_body.append(transform_points(np.asarray(patch.vertices), T_body_B))
    poses = np.asarray(measured["link_poses_xyzw"])
    link_names = measured["link_names"]
    base_index = link_names.index(config.base_body)
    body_indices = [link_names.index(name) for name in body_names]
    T_B_fingers = np.empty((len(poses), 2, 4, 4))
    contact_points = np.empty((len(poses), 2, 3))
    openings = []
    for sample, state in enumerate(poses):
        inverse_base = np.linalg.inv(pose_matrix(state[base_index]))
        projected = []
        for side, body_index in enumerate(body_indices):
            T_B_fingers[sample, side] = inverse_base @ pose_matrix(state[body_index])
            points = transform_points(patch_vertices_body[side], T_B_fingers[sample, side])
            values = points @ config.opening_axis_base
            inner = values.max() if signs[side] > 0 else values.min()
            contact_points[sample, side] = points[
                np.abs(values - inner) <= config.calibration.position_tolerance_m
            ].mean(axis=0)
            projected.append(inner)
        openings.append(projected[order[1]] - projected[order[0]])
    signed_opening = np.asarray(openings)
    measured["surface_gaps_m"] = signed_opening.tolist()
    if not (np.all(np.diff(signed_opening) > 0) or np.all(np.diff(signed_opening) < 0)):
        raise RuntimeError("Measured contact aperture is not strictly monotonic")
    sorting = np.argsort(signed_opening)
    closed, opened = int(sorting[0]), int(sorting[-1])
    arrays: dict[str, np.ndarray] = {
        "opening_m": np.maximum(signed_opening[sorting], 0.0),
        "signed_surface_gap_m": signed_opening[sorting],
        "joint_positions_m": np.asarray(measured["positions_m"])[sorting],
        "joint_commands_m": np.asarray(measured["commands_m"])[sorting, None],
        "joint_velocities_m_s": np.asarray(measured["velocities_m_s"])[sorting],
        "T_B_fingers": T_B_fingers[sorting],
        "contact_points_B_m": contact_points[sorting],
        "T_B_tcp": np.asarray(extraction["tcp"]["T_B_tcp"]),
        "T_ee_B": np.asarray(extraction["tcp"]["T_ee_B"]),
        "joint_limits_m": np.asarray(measured["limits_m"]),
        "approach_axis_tcp": np.asarray(extraction["tcp"]["approach_axis_tcp"]),
        "opening_axis_tcp": np.asarray(extraction["tcp"]["opening_axis_tcp"]),
    }
    for i, (patch, points) in enumerate(zip(patches, patch_vertices_body, strict=True)):
        arrays[f"contact_patch_{i}_vertices_body_m"] = points
        arrays[f"contact_patch_{i}_faces"] = np.asarray(patch.faces, dtype=np.int64)
    if not all(np.isfinite(value).all() and value.dtype != object for value in arrays.values()):
        raise RuntimeError("Nonfinite or object array in gripper definition")
    active_joint = next(
        source.GetPrimAtPath(config.root_prim + "/" + p)
        for p in config.retained_joints
        if Path(p).name == active
    )
    drive = UsdPhysics.DriveAPI(active_joint, "linear")
    expected = [
        drive.GetStiffnessAttr().Get(),
        drive.GetDampingAttr().Get(),
        drive.GetMaxForceAttr().Get(),
    ]
    actual = [
        measured[key][measured["active_index"]] for key in ("stiffness", "damping", "max_forces_N")
    ]
    if not np.allclose(expected, actual):
        raise RuntimeError("Effective active drive differs from authoritative USD")
    definition = {
        "format": "graspdatagen.gripper.v1",
        "name": config.name,
        "joint_names": measured["joint_names"],
        "command_joint_names": [active],
        "finger_body_names": body_names,
        "tcp": extraction["tcp"],
        "closed_command_m": measured["commands_m"][closed],
        "open_command_m": measured["commands_m"][opened],
        "closing_command_direction": float(
            np.sign(measured["commands_m"][closed] - measured["commands_m"][opened])
        ),
        "opening_range_m": [float(max(signed_opening[closed], 0.0)), float(signed_opening[opened])],
        "signed_closed_surface_gap_m": float(signed_opening[closed]),
        "aperture_method": (
            "minimum projected gap between both inward-facing cooked collision surfaces "
            "in the configured contact region; negative closed overlap is retained separately"
        ),
        "array_shapes": {key: list(value.shape) for key, value in arrays.items()},
        "array_dtypes": {key: str(value.dtype) for key, value in arrays.items()},
        "calibration": measured,
        "extraction": extraction,
    }
    np.savez_compressed(directory / "definition.npz", allow_pickle=False, **arrays)
    with np.load(directory / "definition.npz", allow_pickle=False) as loaded:
        for name, value in arrays.items():
            if not np.array_equal(loaded[name], value):
                raise RuntimeError(f"Definition array round-trip failed: {name}")
    write_json(directory / "definition.json", definition)
    return definition


def prepare_gripper(runtime: PhysxRuntime, config: GripperConfig, cache_root: Path) -> Path:
    fingerprint = source_fingerprint(config.source)
    fingerprint["files"][str(config.robot_config)] = file_hash(config.robot_config)
    urdf = Path(config.robot_snapshot["urdf_path"])
    fingerprint["files"][str(urdf)] = file_hash(urdf)
    key = cache_identity(fingerprint, config.snapshot)
    destination = cache_root / "grippers" / key
    if destination.exists():
        check_cache(destination)
        return destination
    print(f"P1: extracting and calibrating {config.name}", flush=True)
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=".preparing-", dir=destination.parent) as temporary:
        directory = Path(temporary)
        extraction = extract_gripper(runtime, config, directory)
        references = check_references(directory / "gripper.usdc")
        definition = calibrate_definition(runtime, config, directory, extraction)
        definition["contact_probes"] = [
            runtime.probe_gripper_contact(directory / "gripper.usdc", config, definition, friction)
            for friction in config.sensitivity_friction
        ]
        nominal = [
            probe
            for probe in definition["contact_probes"]
            if np.isclose(probe["friction"], config.material.dynamic_friction)
        ]
        if len(nominal) != 1 or not nominal[0]["held"]:
            raise RuntimeError("The configured material did not hold the calibration gauge")
        from graspdatagen.inspection import inspect_gripper_3d

        inspect_gripper_3d(directory, definition)
        write_json(directory / "definition.json", definition)
        write_json(directory / "robot_snapshot.json", config.robot_snapshot)
        seal_cache(
            directory,
            {
                "kind": "gripper",
                "name": config.name,
                "key": key,
                "source": fingerprint,
                "config": config.snapshot,
                "definition": definition,
                "references": references,
            },
        )
        directory.rename(destination)
    return destination
