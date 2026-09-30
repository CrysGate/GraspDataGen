"""CPU-only USD surfaces and exact recorded closure transforms for the browser."""

from __future__ import annotations

import base64
import json
from pathlib import Path
from typing import Any

import numpy as np
import trimesh

from graspdatagen.assets import file_hash, inspect_object, open_stage
from graspdatagen.config import load_gripper, load_objects, read_mapping
from graspdatagen.geometry import mesh_in_frame, pose_matrices
from graspdatagen.grippers import tcp_definition
from graspdatagen.records import PreparedPair
from graspdatagen.viewer import closure_transforms


def discover_prepared_objects(prepared_root: Path) -> dict[str, Path]:
    """Return one exact prepared geometry per object, rejecting real ambiguity."""
    grouped: dict[str, list[tuple[Path, str | None]]] = {}
    for manifest_path in sorted(prepared_root.glob("*/manifest.json")):
        geometry = manifest_path.parent / "geometry.npz"
        if not geometry.is_file():
            continue
        manifest = json.loads(manifest_path.read_text())
        if manifest.get("kind") != "object":
            continue
        digest = manifest.get("artifacts", {}).get("geometry.npz")
        grouped.setdefault(manifest["name"], []).append(
            (geometry, digest or file_hash(geometry))
        )

    result: dict[str, Path] = {}
    for name, entries in grouped.items():
        hashes = {digest for _, digest in entries}
        if len(hashes) > 1:
            raise ValueError(
                f"{name}: prepared caches have different geometry hashes"
            )
        result[name] = entries[0][0].resolve()
    return dict(sorted(result.items()))


def _load_prepared_object(source: Path) -> dict[str, Any]:
    """Load an annotation-only scene directly from an existing prepared cache."""
    from pxr import UsdGeom

    directory = source.parent
    manifest = json.loads((directory / "manifest.json").read_text())
    if manifest.get("kind") != "object":
        raise ValueError(f"Not a prepared object cache: {directory}")

    stage = open_stage(directory / "replay.usdc", "")
    if UsdGeom.GetStageMetersPerUnit(stage) != 1:
        raise ValueError("Web viewer requires metre-authored USD assets")
    root = stage.GetDefaultPrim()
    if not root:
        raise ValueError("Prepared object USD needs a default prim")
    meshes = surface_payload(root, root)
    vertices = np.concatenate(
        [np.asarray(mesh["vertices"]).reshape(-1, 3) for mesh in meshes]
    )
    with np.load(source, allow_pickle=False) as geometry:
        annotation_vertices = np.asarray(
            geometry["surface_vertices_m"], dtype=np.float64
        )
        annotation_faces = np.asarray(
            geometry["surface_faces"], dtype=np.int64
        )

    return {
        "object": manifest["name"],
        "robot": "",
        "source": str(source),
        "provenance": "Prepared object / annotation only",
        "candidates": [],
        "object_meshes": meshes,
        "annotation_mesh": {
            "vertices": annotation_vertices.ravel(),
            "faces": annotation_faces.ravel(),
        },
        "parts": [],
        "bounds": [vertices.min(axis=0).tolist(), vertices.max(axis=0).tolist()],
    }


def _load_raw_object(source: Path, objects: Path) -> dict[str, Any]:
    """Build an annotation-only scene from source USD without Isaac Sim."""
    configs = load_objects(objects)
    matching = [item for item in configs if item.source.resolve() == source.resolve()]
    if len(matching) != 1:
        raise ValueError(f"Expected one object configuration for {source}")
    config = matching[0]
    stage = open_stage(config.source, config.root_prim)
    root = stage.GetDefaultPrim()
    if not root:
        raise ValueError(f"Object {config.name} needs a default prim")

    inspection = inspect_object(config)
    surface = trimesh.util.concatenate(
        [
            mesh_in_frame(stage.GetPrimAtPath(path), root)
            for path in inspection["surface_prims"]
        ]
    )
    surface.apply_scale(inspection["meters_per_unit"])
    surface.merge_vertices()
    surface.fix_normals(multibody=True)

    vertices = np.asarray(surface.vertices, dtype=np.float64)
    faces = np.asarray(surface.faces, dtype=np.int64)
    annotation_mesh = {
        "vertices": vertices.ravel(),
        "faces": faces.ravel(),
    }
    return {
        "object": config.name,
        "robot": "",
        "source": str(config.source),
        "provenance": "Source object / CPU annotation surface",
        "candidates": [],
        "object_meshes": [
            {
                **annotation_mesh,
                "color": [0.62, 0.66, 0.69],
                "representation": "visual",
            }
        ],
        "annotation_mesh": annotation_mesh,
        "parts": [],
        "bounds": [surface.bounds[0].tolist(), surface.bounds[1].tolist()],
    }


def surface_payload(root: Any, frame: Any) -> list[dict[str, Any]]:
    from pxr import Ar, Usd, UsdGeom, UsdShade

    surfaces = []
    visible = []
    for prim in Usd.PrimRange(root, Usd.TraverseInstanceProxies()):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        imageable = UsdGeom.Imageable(prim)
        if imageable.ComputeVisibility() == "invisible":
            continue
        visible.append(prim)
    visual = [
        prim
        for prim in visible
        if UsdGeom.Imageable(prim).ComputePurpose() not in ("proxy", "guide")
        and not any(
            p.lower() in ("collision", "collisions") for p in str(prim.GetPath()).split("/")
        )
    ]
    # Older prepared caches contain only cooked collision meshes.
    for prim in visual or visible:
        mesh = mesh_in_frame(prim, frame)
        item: dict[str, Any] = {
            "vertices": np.asarray(mesh.vertices).ravel(),
            "faces": np.asarray(mesh.faces).ravel(),
            "color": [0.62, 0.66, 0.69],
            "representation": "visual" if visual else "collision",
        }
        colors = UsdGeom.Mesh(prim).GetDisplayColorAttr().Get()
        if colors:
            item["color"] = list(colors[0])
        material, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()
        if material:
            for shader in Usd.PrimRange(material.GetPrim()):
                for name in ("inputs:diffuse_color_constant", "inputs:diffuseColor"):
                    color = shader.GetAttribute(name).Get()
                    if color is not None:
                        item["color"] = list(color)
                texture = shader.GetAttribute("inputs:diffuse_texture").Get()
                uv = UsdGeom.PrimvarsAPI(prim).GetPrimvar("st")
                if texture and uv and uv.HasValue():
                    coordinates = np.asarray(uv.ComputeFlattened())
                    interpolation = uv.GetInterpolation()
                    if interpolation == "vertex":
                        item["uv"] = coordinates.ravel()
                    elif interpolation == "faceVarying":
                        counts = UsdGeom.Mesh(prim).GetFaceVertexCountsAttr().Get()
                        # Preserve USD face-corner UVs by splitting triangle vertices.
                        if not all(count == 3 for count in counts):
                            continue
                        if UsdGeom.Mesh(prim).GetHoleIndicesAttr().Get():
                            continue
                        faces = np.asarray(mesh.faces)
                        item["vertices"] = np.asarray(mesh.vertices)[faces].ravel()
                        item["faces"] = np.arange(faces.size, dtype=np.uint32)
                        original = np.asarray(
                            UsdGeom.Mesh(prim).GetFaceVertexIndicesAttr().Get()
                        ).reshape(-1, 3)
                        if not np.array_equal(original, faces):
                            coordinates = coordinates.reshape(-1, 3, 2)[:, ::-1].reshape(-1, 2)
                        item["uv"] = coordinates.ravel()
                    else:
                        continue
                    resolved = texture.resolvedPath
                    if resolved:
                        asset = Ar.GetResolver().OpenAsset(Ar.ResolvedPath(resolved))
                        suffix = Path(texture.path).suffix.lower()
                        mime = "image/jpeg" if suffix in (".jpg", ".jpeg") else "image/png"
                        item["texture"] = (
                            f"data:{mime};base64," + base64.b64encode(asset.GetBuffer()).decode()
                        )
        surfaces.append(item)
    if not surfaces:
        raise ValueError(f"No visible mesh surfaces in {root.GetPath()}")
    return surfaces


def load_dataset(
    source: Path, objects: Path, grippers: tuple[Path, ...], overview_faces: int
) -> dict[str, Any]:
    from pxr import Usd, UsdGeom, UsdPhysics

    if source.name == "geometry.npz" and source.with_name("manifest.json").is_file():
        return _load_prepared_object(source)

    configured_sources = {item.source.resolve() for item in load_objects(objects)}
    if source.resolve() in configured_sources:
        return _load_raw_object(source, objects)

    data = read_mapping(source)
    if data["position_unit"] != "m" or data["pose_layout"] != [
        "x",
        "y",
        "z",
        "qx",
        "qy",
        "qz",
        "qw",
    ]:
        raise ValueError("Expected metre xyz/xyzw poses")
    candidates = data["candidates"]
    if not candidates:
        raise ValueError("This dataset contains no grasp poses")
    robot = candidates[0]["robot"]
    manifest_path = source.with_name("manifest.json")
    if manifest_path.exists():
        manifest = json.loads(manifest_path.read_text())
        pair = PreparedPair.load(Path(manifest["gripper_cache"]), Path(manifest["object_cache"]))
        if pair.object_manifest["name"] != data["object"]:
            raise ValueError("Object does not match the prepared manifest")
        if json.loads((pair.gripper / "robot_snapshot.json").read_text())["name"] != robot:
            raise ValueError("Robot does not match the prepared manifest")
        object_stage = open_stage(pair.object / "replay.usdc", "")
        gripper_stage = open_stage(pair.gripper / "gripper.usdc", "")
        root = gripper_stage.GetDefaultPrim()
        definition = pair.definition
        bodies = definition["extraction"]["retained_bodies"]
        base_body = definition["extraction"]["base_body"]
        names = definition["joint_names"]
        tcp = pair.arrays["T_B_tcp"]

        # Exact prepared surface used by region sampling.  Keep this separate
        # from the visual USD mesh because surface_payload() may alter topology
        # for rendering (for example when splitting face-varying UV vertices).
        with np.load(pair.object / "geometry.npz", allow_pickle=False) as geometry:
            annotation_vertices = np.asarray(
                geometry["surface_vertices_m"], dtype=np.float64
            )
            annotation_faces = np.asarray(
                geometry["surface_faces"], dtype=np.int64
            )
        annotation_mesh = {
            "vertices": annotation_vertices.ravel(),
            "faces": annotation_faces.ravel(),
        }

        provenance = "Prepared assets"
    else:
        object_config = next((o for o in load_objects(objects) if o.name == data["object"]), None)
        # Missing configuration is an input error, not an alternative asset guess.
        if object_config is None:
            raise ValueError(f"Object {data['object']} is absent from {objects}")
        configs = [load_gripper(path) for path in grippers]
        matching = [c for c in configs if c.robot_snapshot["name"] == robot]
        if len(matching) != 1:
            raise ValueError(f"Expected one gripper configuration for {robot}")
        config = matching[0]
        object_stage = open_stage(object_config.source, object_config.root_prim)
        source_stage = open_stage(config.source, config.root_prim)
        gripper_stage = Usd.Stage.Open(source_stage.Flatten())
        root = gripper_stage.GetPrimAtPath(config.root_prim)
        retained = {config.root_prim + "/" + p for p in config.retained_joints}
        for prim in list(gripper_stage.Traverse()):
            if prim.IsA(UsdPhysics.Joint) and str(prim.GetPath()) not in retained:
                gripper_stage.RemovePrim(prim.GetPath())
        bodies = config.retained_bodies
        base_body = config.base_body
        names = config.robot_snapshot["gripper"]["joint_names"]
        tcp = np.asarray(tcp_definition(config)["T_B_tcp"])
        annotation_mesh = None
        provenance = "Source assets"
    if any(UsdGeom.GetStageMetersPerUnit(s) != 1 for s in (object_stage, gripper_stage)):
        raise ValueError("Web viewer requires metre-authored USD assets")
    poses = pose_matrices(np.asarray([c["pose_object_tcp_xyz_xyzw"] for c in candidates]))
    body_prims = [gripper_stage.GetPrimAtPath(str(root.GetPath()) + "/" + b) for b in bodies]
    parts = []
    for body in body_prims:
        meshes = surface_payload(body, body)
        for mesh in meshes:
            if len(mesh["faces"]) // 3 > overview_faces:
                overview = trimesh.Trimesh(
                    np.asarray(mesh["vertices"]).reshape(-1, 3),
                    np.asarray(mesh["faces"]).reshape(-1, 3),
                    process=False,
                ).simplify_quadric_decimation(face_count=overview_faces)
                mesh["overview"] = {
                    "vertices": np.asarray(overview.vertices).ravel(),
                    "faces": np.asarray(overview.faces).ravel(),
                }
        parts.append({"name": body.GetName(), "meshes": meshes, "matrices": []})
    inverse_tcp = np.linalg.inv(tcp)
    for index, (candidate, pose) in enumerate(zip(candidates, poses, strict=True)):
        if candidate["candidate_id"] != index or candidate["robot"] != robot:
            raise ValueError("Expected sequential candidate IDs and one matching robot")
        joints = candidate["closed_joint_positions_m"]
        if set(joints) != set(names) or not np.isfinite(list(joints.values())).all():
            raise ValueError("Closure joints must match the gripper and contain finite values")
        axis = np.asarray(candidate["approach_axis_object"])
        if (
            axis.shape != (3,)
            or not np.isfinite(axis).all()
            or not np.isclose(np.linalg.norm(axis), 1, atol=1e-6)
        ):
            raise ValueError("Expected a finite unit approach axis")
        transforms = closure_transforms(root, base_body, joints)
        for part, body in zip(parts, body_prims, strict=True):
            matrix = pose @ inverse_tcp @ transforms[str(body.GetPath())]
            part["matrices"].append(matrix.T.ravel().tolist())
    object_root = object_stage.GetDefaultPrim()
    if not object_root:
        raise ValueError("Object USD needs a default prim")
    meshes = surface_payload(object_root, object_root)
    if any(m["representation"] == "collision" for p in parts for m in p["meshes"]):
        provenance += " / collision gripper"
    vertices = np.concatenate([np.asarray(m["vertices"]).reshape(-1, 3) for m in meshes])
    return {
        "object": data["object"],
        "robot": robot,
        "source": str(source),
        "provenance": provenance,
        "candidates": candidates,
        "object_meshes": meshes,
        "annotation_mesh": annotation_mesh,
        "parts": parts,
        "bounds": [vertices.min(axis=0).tolist(), vertices.max(axis=0).tolist()],
    }
