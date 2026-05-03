# /// script
# requires-python = ">=3.10"
# dependencies = ["usd-core"]
# ///
"""Inspect a Piper robot USD and extract a GraspDataGen-ready gripper USD.

Run with:
    uv run scripts/graspgen/tools/extract_piper_gripper_usd.py

The script is intentionally USD-only. It does not require Isaac Sim for the
inspection/extraction pass, but the generated USD should still be validated
with create_gripper_lab.py inside the Isaac Lab environment.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Iterable

from pxr import Gf, Sdf, Usd, UsdPhysics


DEFAULT_SOURCE = "piper_isaac_sim/USD/piper_v2.usd"
DEFAULT_OUTPUT = "bots/piper_v2_gripper.usd"
DEFAULT_REPORT_DIR = "debug_output/usd_inspect/piper_v2"
DEFAULT_BATCH_SOURCE_GLOB = "piper_isaac_sim/USD/*.usd"
DEFAULT_BATCH_REPORT_DIR = "debug_output/usd_inspect/piper_batch"


def as_path(value: str) -> Sdf.Path:
    path = Sdf.Path(value)
    if not path.IsAbsolutePath():
        raise ValueError(f"USD path must be absolute: {value}")
    return path


def resolve_asset(anchor_file: Path, asset_path: str) -> Path:
    asset = Path(asset_path)
    if asset.is_absolute():
        return asset
    return (anchor_file.parent / asset).resolve()


def open_stage(path: Path) -> Usd.Stage:
    stage = Usd.Stage.Open(str(path), load=Usd.Stage.LoadAll)
    if stage is None:
        raise RuntimeError(f"Could not open USD stage: {path}")
    return stage


def iter_mesh_prims(stage: Usd.Stage, root_path: str) -> Iterable[Usd.Prim]:
    root = stage.GetPrimAtPath(root_path)
    if not root:
        return []
    return (prim for prim in Usd.PrimRange(root) if prim.GetTypeName() == "Mesh")


def has_mesh(stage: Usd.Stage, root_path: str) -> bool:
    return any(True for _ in iter_mesh_prims(stage, root_path))


def reference_asset_paths(prim: Usd.Prim) -> list[str]:
    refs = prim.GetMetadata("references")
    if not refs:
        return []
    return [item.assetPath for item in refs.prependedItems if item.assetPath]


def payload_asset_paths(prim: Usd.Prim) -> list[str]:
    payloads = prim.GetMetadata("payload")
    if not payloads:
        return []
    return [item.assetPath for item in payloads.prependedItems if item.assetPath]


def infer_gripper(stage: Usd.Stage) -> dict[str, object]:
    """Infer Piper gripper paths from joint7/joint8 body targets."""

    joint7 = None
    joint8 = None
    for prim in stage.Traverse():
        if prim.GetTypeName() != "PhysicsPrismaticJoint":
            continue
        if prim.GetName() == "joint7":
            joint7 = prim
        elif prim.GetName() == "joint8":
            joint8 = prim

    if joint7 is None or joint8 is None:
        raise RuntimeError("Could not infer gripper: expected PhysicsPrismaticJoint prims named joint7 and joint8.")

    root_path = str(joint7.GetPath()).rsplit("/joints/joint7", 1)[0]
    joint8_root_path = str(joint8.GetPath()).rsplit("/joints/joint8", 1)[0]
    if joint8_root_path != root_path:
        raise RuntimeError(f"joint7 and joint8 are under different roots: {root_path}, {joint8_root_path}")

    def single_target(joint: Usd.Prim, rel_name: str) -> Sdf.Path:
        targets = joint.GetRelationship(rel_name).GetTargets()
        if len(targets) != 1:
            raise RuntimeError(f"{joint.GetPath()} relationship {rel_name} expected one target, got {targets}")
        return targets[0]

    base7 = single_target(joint7, "physics:body0")
    base8 = single_target(joint8, "physics:body0")
    if base7 != base8:
        raise RuntimeError(f"joint7 and joint8 do not share the same base body: {base7}, {base8}")

    finger7 = single_target(joint7, "physics:body1")
    finger8 = single_target(joint8, "physics:body1")
    return {
        "root_path": root_path,
        "base_link": base7.name,
        "finger_links": [finger7.name, finger8.name],
        "finger_joints": [joint7.GetName(), joint8.GetName()],
    }


def stage_has_gripper_meshes(stage: Usd.Stage, finger_links: list[str]) -> bool:
    return all(
        has_mesh(stage, f"/colliders/{name}") or has_mesh(stage, f"/visuals/{name}")
        for name in finger_links
    )


def discover_linked_assets(stage: Usd.Stage, candidate: Path, root_path: str) -> list[tuple[str, Path]]:
    discovered: list[tuple[str, Path]] = []
    roots_to_check: list[Usd.Prim] = []

    root = stage.GetPrimAtPath(root_path)
    if root:
        roots_to_check.append(root)

    default_prim = stage.GetDefaultPrim()
    if default_prim and default_prim not in roots_to_check:
        roots_to_check.append(default_prim)

    for prim in roots_to_check:
        for asset_path in reference_asset_paths(prim):
            discovered.append((f"{prim.GetPath()} reference {asset_path}", resolve_asset(candidate, asset_path)))
        for asset_path in payload_asset_paths(prim):
            discovered.append((f"{prim.GetPath()} payload {asset_path}", resolve_asset(candidate, asset_path)))
    return discovered


def find_extraction_stage(source: Path, source_root_path: str | None = None) -> tuple[Path, Usd.Stage, dict[str, object], list[str]]:
    """Find the layer that contains the actual /visuals and /colliders meshes.

    Piper USD files in this checkout are light wrappers. Some wrappers author
    payload paths relative to the referenced robot USD, not relative to the
    wrapper itself. This follows references/payloads and picks the first stage
    with /colliders or /visuals meshes for the inferred fingers.
    """

    notes: list[str] = []
    candidates: list[Path] = [source.resolve()]
    seen: set[Path] = set()

    idx = 0
    while idx < len(candidates):
        candidate = candidates[idx]
        idx += 1
        if candidate in seen or not candidate.exists():
            continue
        seen.add(candidate)
        stage = open_stage(candidate)

        try:
            gripper_info = infer_gripper(stage)
        except RuntimeError as exc:
            gripper_info = None
            notes.append(f"Could not infer gripper in {candidate}: {exc}")

        if gripper_info is not None:
            finger_links = list(gripper_info["finger_links"])
            if stage_has_gripper_meshes(stage, finger_links):
                notes.append(f"Using mesh-bearing stage: {candidate}")
                return candidate, stage, gripper_info, notes

            root_path = str(gripper_info["root_path"])
        else:
            root_path = source_root_path or ""

        for label, resolved in discover_linked_assets(stage, candidate, root_path):
            notes.append(f"Found {label} -> {resolved}")
            candidates.append(resolved)

    checked = "\n".join(str(path) for path in seen)
    raise RuntimeError(
        "Could not find a composed Piper USD with /colliders or /visuals meshes for the fingers. "
        f"Checked:\n{checked}"
    )


def attr_value_for_report(attr: Usd.Attribute, include_large_arrays: bool) -> object:
    try:
        value = attr.Get()
    except Exception as exc:  # pragma: no cover - defensive USD inspection
        return f"<unreadable: {exc}>"

    if value is None:
        return None

    if include_large_arrays:
        try:
            return str(value)
        except Exception:
            return repr(value)

    if hasattr(value, "__len__") and not isinstance(value, (str, bytes, Gf.Vec3f, Gf.Vec3d, Gf.Quatf, Gf.Quatd)):
        try:
            length = len(value)
            if length > 24:
                return f"<{type(value).__name__} len={length}>"
        except Exception:
            pass
    return str(value)


def write_stage_report(stage: Usd.Stage, output_path: Path, include_large_arrays: bool) -> dict[str, object]:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    summary: dict[str, object] = {
        "layer": stage.GetRootLayer().identifier,
        "defaultPrim": str(stage.GetDefaultPrim().GetPath()) if stage.GetDefaultPrim() else None,
        "prim_count": 0,
        "type_counts": {},
        "joints": [],
        "rigid_bodies": [],
        "collision_meshes": [],
    }

    with output_path.open("w", encoding="utf-8") as out:
        out.write(f"Layer: {summary['layer']}\n")
        out.write(f"Default prim: {summary['defaultPrim']}\n\n")

        for prim in stage.Traverse():
            summary["prim_count"] = int(summary["prim_count"]) + 1
            type_name = prim.GetTypeName() or "<typeless>"
            type_counts = summary["type_counts"]
            assert isinstance(type_counts, dict)
            type_counts[type_name] = type_counts.get(type_name, 0) + 1

            path = str(prim.GetPath())
            schemas = list(prim.GetAppliedSchemas())
            indent = "  " * max(0, path.count("/") - 1)
            out.write(f"{indent}{path}  type={type_name} schemas={schemas}\n")

            if "PhysicsRigidBodyAPI" in schemas:
                summary["rigid_bodies"].append(path)  # type: ignore[union-attr]

            if type_name.startswith("Physics") and "Joint" in type_name:
                joint_info = {
                    "path": path,
                    "type": type_name,
                    "body0": [str(target) for target in prim.GetRelationship("physics:body0").GetTargets()],
                    "body1": [str(target) for target in prim.GetRelationship("physics:body1").GetTargets()],
                    "axis": attr_value_for_report(prim.GetAttribute("physics:axis"), include_large_arrays),
                    "lowerLimit": attr_value_for_report(prim.GetAttribute("physics:lowerLimit"), include_large_arrays),
                    "upperLimit": attr_value_for_report(prim.GetAttribute("physics:upperLimit"), include_large_arrays),
                }
                summary["joints"].append(joint_info)  # type: ignore[union-attr]

            if type_name == "Mesh":
                counts = prim.GetAttribute("faceVertexCounts").Get()
                face_count = len(counts) if counts is not None else 0
                all_triangles = bool(counts is not None and all(count == 3 for count in counts))
                summary["collision_meshes"].append(  # type: ignore[union-attr]
                    {"path": path, "faces": face_count, "all_triangles": all_triangles}
                )

            for attr in prim.GetAttributes():
                out.write(f"{indent}  attr {attr.GetName()} = {attr_value_for_report(attr, include_large_arrays)}\n")
            for rel in prim.GetRelationships():
                targets = [str(target) for target in rel.GetTargets()]
                out.write(f"{indent}  rel  {rel.GetName()} -> {targets}\n")
            out.write("\n")

    json_path = output_path.with_suffix(".json")
    json_path.write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    return summary


def copy_spec(src_layer: Sdf.Layer, src_path: str, dst_layer: Sdf.Layer, dst_path: str) -> None:
    if not Sdf.CopySpec(src_layer, Sdf.Path(src_path), dst_layer, Sdf.Path(dst_path)):
        raise RuntimeError(f"Failed to copy USD spec {src_path} -> {dst_path}")


def clear_composition_arcs(prim: Usd.Prim) -> None:
    prim.GetReferences().ClearReferences()
    prim.GetPayloads().ClearPayloads()


def remove_children(stage: Usd.Stage, prim_path: str) -> None:
    prim = stage.GetPrimAtPath(prim_path)
    if not prim:
        return
    for child in list(prim.GetChildren()):
        stage.RemovePrim(child.GetPath())


def set_relationship_targets(prim: Usd.Prim, rel_name: str, targets: list[str]) -> None:
    rel = prim.GetRelationship(rel_name)
    if not rel:
        rel = prim.CreateRelationship(rel_name)
    rel.SetTargets([Sdf.Path(target) for target in targets])


def set_float_attr(prim: Usd.Prim, attr_name: str, value: float) -> None:
    attr = prim.GetAttribute(attr_name)
    if not attr:
        attr = prim.CreateAttribute(attr_name, Sdf.ValueTypeNames.Float)
    attr.Set(float(value))


def set_token_attr(prim: Usd.Prim, attr_name: str, value: str) -> None:
    attr = prim.GetAttribute(attr_name)
    if not attr:
        attr = prim.CreateAttribute(attr_name, Sdf.ValueTypeNames.Token)
    attr.Set(value)


def remove_property_if_present(prim: Usd.Prim, property_name: str) -> None:
    if prim.HasProperty(property_name):
        prim.RemoveProperty(property_name)


def remove_api_schema(prim: Usd.Prim, schema_name: str) -> None:
    api_schemas = prim.GetMetadata("apiSchemas")
    if not api_schemas:
        return
    explicit_items = list(api_schemas.GetAddedOrExplicitItems())
    if schema_name not in explicit_items:
        return
    explicit_items = [item for item in explicit_items if item != schema_name]
    prim.SetMetadata("apiSchemas", Sdf.TokenListOp.CreateExplicit(explicit_items))


def quat_times(a: Gf.Quatf, b: Gf.Quatf) -> Gf.Quatf:
    return a * b


def flip_prismatic_joint_coordinate(joint: Usd.Prim) -> None:
    """Reverse the joint coordinate sign by flipping both local joint frames."""

    flip = Gf.Quatf(0.0, Gf.Vec3f(1.0, 0.0, 0.0))
    for attr_name in ("physics:localRot0", "physics:localRot1"):
        attr = joint.GetAttribute(attr_name)
        old = attr.Get()
        if old is None:
            continue
        attr.Set(quat_times(old, flip))


def normalize_finger_joint_limits(
    stage: Usd.Stage,
    root_path: str,
    joint_names: list[str],
    mode: str,
) -> list[str]:
    notes: list[str] = []
    if mode == "none":
        return notes

    for joint_name in joint_names:
        joint = stage.GetPrimAtPath(f"{root_path}/joints/{joint_name}")
        if not joint:
            notes.append(f"Missing joint for normalization: {joint_name}")
            continue

        lower_attr = joint.GetAttribute("physics:lowerLimit")
        upper_attr = joint.GetAttribute("physics:upperLimit")
        lower = float(lower_attr.Get())
        upper = float(upper_attr.Get())

        if mode == "positive" and lower < 0.0 and upper <= 0.0:
            width = abs(upper - lower)
            flip_prismatic_joint_coordinate(joint)
            set_float_attr(joint, "physics:lowerLimit", 0.0)
            set_float_attr(joint, "physics:upperLimit", width)
            set_float_attr(joint, "state:linear:physics:position", 0.0)
            set_float_attr(joint, "drive:linear:physics:targetPosition", 0.0)
            notes.append(
                f"Normalized {joint_name}: old limits [{lower}, {upper}] -> [0.0, {width}], "
                "flipped localRot0/localRot1 so positive motion preserves the original physical direction."
            )
        elif mode == "positive":
            notes.append(f"Kept {joint_name}: limits already non-negative [{lower}, {upper}]")
        else:
            raise ValueError(f"Unknown normalization mode: {mode}")

    return notes


def configure_mimic_finger_pair(
    stage: Usd.Stage,
    root_path: str,
    finger_joints: list[str],
    mimic_axis: str = "rotZ",
    gearing: float = 1.0,
) -> list[str]:
    """Make the first finger joint active and the second a PhysX mimic follower."""

    notes: list[str] = []
    if len(finger_joints) != 2:
        return [f"WARNING: expected two finger joints for mimic setup, got {finger_joints}"]

    active = stage.GetPrimAtPath(f"{root_path}/joints/{finger_joints[0]}")
    follower = stage.GetPrimAtPath(f"{root_path}/joints/{finger_joints[1]}")
    if not active or not follower:
        return [f"WARNING: missing active/follower joint for mimic setup: {finger_joints}"]

    drive_names = [
        "drive:linear:physics:stiffness",
        "drive:linear:physics:damping",
        "drive:linear:physics:maxForce",
        "drive:linear:physics:targetPosition",
        "drive:linear:physics:targetVelocity",
        "drive:linear:physics:type",
    ]

    def attr_value(prim: Usd.Prim, name: str):
        attr = prim.GetAttribute(name)
        return attr.Get() if attr else None

    active_stiffness = attr_value(active, "drive:linear:physics:stiffness")
    follower_stiffness = attr_value(follower, "drive:linear:physics:stiffness")
    if (active_stiffness is None or float(active_stiffness) == 0.0) and follower_stiffness is not None and float(follower_stiffness) != 0.0:
        for name in drive_names:
            source_attr = follower.GetAttribute(name)
            if source_attr:
                target_attr = active.GetAttribute(name)
                if not target_attr:
                    target_attr = active.CreateAttribute(name, source_attr.GetTypeName())
                target_attr.Set(source_attr.Get())
        active.SetMetadata("apiSchemas", Sdf.TokenListOp.CreateExplicit(["PhysicsJointStateAPI:linear", "PhysxJointAPI", "PhysicsDriveAPI:linear"]))
        notes.append(f"Copied non-zero drive parameters from follower {finger_joints[1]} to active {finger_joints[0]}")

    for name in drive_names:
        remove_property_if_present(follower, name)
    remove_api_schema(follower, "PhysicsDriveAPI:linear")

    mimic_prefix = f"physxMimicJoint:{mimic_axis}"
    set_relationship_targets(follower, f"{mimic_prefix}:referenceJoint", [str(active.GetPath())])
    set_float_attr(follower, f"{mimic_prefix}:gearing", gearing)
    set_float_attr(follower, f"{mimic_prefix}:offset", 0.0)
    set_float_attr(follower, f"{mimic_prefix}:naturalFrequency", 0.0)
    set_float_attr(follower, f"{mimic_prefix}:dampingRatio", 0.0)
    notes.append(
        f"Configured {finger_joints[1]} as mimic follower of {finger_joints[0]} "
        f"with {mimic_prefix}:gearing={gearing}"
    )
    return notes


def apply_collision_api_to_mesh_parents(stage: Usd.Stage, collision_root_path: str) -> list[str]:
    notes: list[str] = []
    root = stage.GetPrimAtPath(collision_root_path)
    if not root:
        return [f"Collision root missing: {collision_root_path}"]

    for mesh in list(iter_mesh_prims(stage, collision_root_path)):
        parent = mesh.GetParent()
        if not parent or parent == root:
            parent = mesh

        UsdPhysics.CollisionAPI.Apply(parent)
        UsdPhysics.MeshCollisionAPI.Apply(parent)
        parent.GetAttribute("physics:collisionEnabled").Set(True)
        set_token_attr(parent, "physics:approximation", "convexHull")

        counts = mesh.GetAttribute("faceVertexCounts").Get()
        if counts is not None and not all(count == 3 for count in counts):
            notes.append(f"WARNING: non-triangular mesh remains at {mesh.GetPath()}")
    return notes


def copy_link_visuals_and_collisions(
    flat_layer: Sdf.Layer,
    stage: Usd.Stage,
    root_path: str,
    link_name: str,
    use_visual_as_collision: bool,
) -> list[str]:
    notes: list[str] = []
    dst_layer = stage.GetRootLayer()
    visual_dst = f"{root_path}/{link_name}/visuals"
    collision_dst = f"{root_path}/{link_name}/collisions"

    for path in (visual_dst, collision_dst):
        prim = stage.GetPrimAtPath(path)
        if prim:
            clear_composition_arcs(prim)
            remove_children(stage, path)

    src_visual = f"/visuals/{link_name}"
    src_collision = f"/visuals/{link_name}" if use_visual_as_collision else f"/colliders/{link_name}"

    visual_stage = Usd.Stage.Open(flat_layer)
    visual_src_prim = visual_stage.GetPrimAtPath(src_visual)
    if visual_src_prim:
        for child in visual_src_prim.GetChildren():
            copy_spec(flat_layer, str(child.GetPath()), dst_layer, f"{visual_dst}/{child.GetName()}")
        notes.append(f"Copied visuals {src_visual} -> {visual_dst}")
    else:
        notes.append(f"WARNING: missing visual source {src_visual}")

    collision_src_prim = visual_stage.GetPrimAtPath(src_collision)
    if collision_src_prim:
        for child in collision_src_prim.GetChildren():
            copy_spec(flat_layer, str(child.GetPath()), dst_layer, f"{collision_dst}/{child.GetName()}")
        notes.append(f"Copied collisions {src_collision} -> {collision_dst}")
    else:
        notes.append(f"WARNING: missing collision source {src_collision}")

    notes.extend(apply_collision_api_to_mesh_parents(stage, collision_dst))
    return notes


def extract_gripper(
    source_stage: Usd.Stage,
    output_path: Path,
    root_path: str,
    base_link: str,
    finger_links: list[str],
    finger_joints: list[str],
    normalize_mode: str,
    base_collision_source: str,
) -> list[str]:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    if output_path.exists():
        output_path.unlink()

    flat_layer = source_stage.Flatten()
    # Export the complete flattened layer first. Copying only /piper_camera can
    # strand OpenUSD's flattened prototype specs and create unresolved-reference
    # warnings. After exporting the whole layer, we destructively trim it down.
    flat_layer.Export(str(output_path))

    stage = open_stage(output_path)
    root = stage.GetPrimAtPath(root_path)
    if not root:
        raise RuntimeError(f"Output root missing after copy: {root_path}")

    notes: list[str] = []
    clear_composition_arcs(root)

    keep_root_children = {base_link, *finger_links, "joints", "Looks", "root_joint"}
    for child in list(root.GetChildren()):
        if child.GetName() not in keep_root_children:
            stage.RemovePrim(child.GetPath())
            notes.append(f"Removed non-gripper prim: {child.GetPath()}")

    base_prim = stage.GetPrimAtPath(f"{root_path}/{base_link}")
    if base_prim:
        for child in list(base_prim.GetChildren()):
            if child.GetName() not in {"visuals", "collisions"}:
                stage.RemovePrim(child.GetPath())
                notes.append(f"Removed non-gripper child under base link: {child.GetPath()}")

    joints_prim = stage.GetPrimAtPath(f"{root_path}/joints")
    if joints_prim:
        keep_joint_set = set(finger_joints)
        for child in list(joints_prim.GetChildren()):
            if child.GetName() not in keep_joint_set:
                stage.RemovePrim(child.GetPath())
                notes.append(f"Removed non-gripper joint: {child.GetPath()}")

    # Repoint articulation metadata and joint targets to the extracted gripper.
    set_relationship_targets(root, "isaac:physics:robotLinks", [f"{root_path}/{base_link}", *[f"{root_path}/{name}" for name in finger_links]])
    set_relationship_targets(root, "isaac:physics:robotJoints", [f"{root_path}/root_joint", *[f"{root_path}/joints/{name}" for name in finger_joints]])

    root_joint = stage.GetPrimAtPath(f"{root_path}/root_joint")
    if root_joint:
        set_relationship_targets(root_joint, "physics:body0", [])
        set_relationship_targets(root_joint, "physics:body1", [f"{root_path}/{base_link}"])
        notes.append(f"Repointed root_joint body1 -> {root_path}/{base_link}")

    for joint_name, finger_link in zip(finger_joints, finger_links):
        joint = stage.GetPrimAtPath(f"{root_path}/joints/{joint_name}")
        if joint:
            set_relationship_targets(joint, "physics:body0", [f"{root_path}/{base_link}"])
            set_relationship_targets(joint, "physics:body1", [f"{root_path}/{finger_link}"])

    for link_name in [base_link, *finger_links]:
        use_visual_as_collision = link_name == base_link and base_collision_source == "visuals"
        notes.extend(copy_link_visuals_and_collisions(flat_layer, stage, root_path, link_name, use_visual_as_collision))

    notes.extend(normalize_finger_joint_limits(stage, root_path, finger_joints, normalize_mode))
    notes.extend(configure_mimic_finger_pair(stage, root_path, finger_joints))

    stage.SetDefaultPrim(root)

    root_name = Sdf.Path(root_path).name
    for child in list(stage.GetPseudoRoot().GetChildren()):
        if child.GetName() != root_name:
            stage.RemovePrim(child.GetPath())
            notes.append(f"Removed non-gripper top-level prim: {child.GetPath()}")

    stage.GetRootLayer().Save()
    notes.append(f"Saved extracted gripper USD: {output_path}")
    return notes


def relative_string(path: Path) -> str:
    cwd = Path.cwd()
    try:
        return str(path.relative_to(cwd))
    except ValueError:
        return str(path)


def count_external_arcs(stage: Usd.Stage) -> int:
    count = 0
    for prim in stage.Traverse():
        refs = prim.GetMetadata("references")
        payloads = prim.GetMetadata("payload")
        if refs and refs.prependedItems:
            count += len(refs.prependedItems)
        if payloads and payloads.prependedItems:
            count += len(payloads.prependedItems)
    return count


def output_mesh_summary(stage: Usd.Stage, root_path: str, link_names: list[str]) -> dict[str, list[dict[str, object]]]:
    summary: dict[str, list[dict[str, object]]] = {}
    for link_name in link_names:
        meshes: list[dict[str, object]] = []
        for mesh in iter_mesh_prims(stage, f"{root_path}/{link_name}/collisions"):
            counts = mesh.GetAttribute("faceVertexCounts").Get()
            meshes.append(
                {
                    "path": str(mesh.GetPath()),
                    "faces": len(counts) if counts is not None else 0,
                    "all_triangles": bool(counts is not None and all(count == 3 for count in counts)),
                }
            )
        summary[link_name] = meshes
    return summary


def write_markdown_batch_report(manifests: list[dict[str, object]], report_path: Path) -> None:
    lines = [
        "# Piper Gripper Extraction Report",
        "",
        "Generated with `uv run scripts/graspgen/tools/extract_piper_gripper_usd.py --all`.",
        "",
        "## Summary",
        "",
        "| Source | Output | Root | Base | Fingers | Joints | Output prims | External arcs | Status |",
        "|---|---|---|---|---|---|---:|---:|---|",
    ]

    for manifest in manifests:
        status = "ok" if manifest.get("ok", False) else "failed"
        lines.append(
            "| {source} | {output} | `{root}` | `{base}` | `{fingers}` | `{joints}` | {prims} | {arcs} | {status} |".format(
                source=Path(str(manifest.get("source", ""))).name,
                output=relative_string(Path(str(manifest.get("output", "")))) if manifest.get("output") else "",
                root=manifest.get("root_path", ""),
                base=manifest.get("base_frame", ""),
                fingers=", ".join(manifest.get("finger_colliders", [])),  # type: ignore[arg-type]
                joints=", ".join(manifest.get("finger_joints", [])),  # type: ignore[arg-type]
                prims=manifest.get("summary", {}).get("output_prim_count", "") if isinstance(manifest.get("summary"), dict) else "",
                arcs=manifest.get("summary", {}).get("output_external_arcs", "") if isinstance(manifest.get("summary"), dict) else "",
                status=status,
            )
        )

    lines.extend(["", "## GraspDataGen Config Snippet", "", "```python", "GRIPPER_CONFIGS.update({"])
    for manifest in manifests:
        if not manifest.get("ok", False):
            continue
        cfg = manifest["recommended_graspdatagen_config"]  # type: ignore[index]
        assert isinstance(cfg, dict)
        name, values = next(iter(cfg.items()))
        lines.append(f"    {name!r}: {{")
        values = dict(values)  # type: ignore[arg-type]
        for key in ["gripper_file", "finger_colliders", "base_frame", "bite", "pinch_width_resolution"]:
            lines.append(f"        {key!r}: {values[key]!r},")
        lines.append("    },")
    lines.extend(["})", "```", ""])

    lines.extend(["## Per-File Details", ""])
    for manifest in manifests:
        lines.append(f"### {Path(str(manifest.get('source', ''))).name}")
        if not manifest.get("ok", False):
            lines.append(f"- Failed: {manifest.get('error')}")
            lines.append("")
            continue
        lines.append(f"- Source: `{relative_string(Path(str(manifest['source'])))}`")
        lines.append(f"- Mesh-bearing extraction source: `{relative_string(Path(str(manifest['extraction_source'])))}`")
        lines.append(f"- Output: `{relative_string(Path(str(manifest['output'])))}`")
        lines.append(f"- Reports: `{relative_string(Path(str(manifest['reports']['output'])))}`")  # type: ignore[index]
        mesh_summary = manifest.get("output_collision_meshes", {})
        if isinstance(mesh_summary, dict):
            for link_name, meshes in mesh_summary.items():
                face_total = sum(int(mesh.get("faces", 0)) for mesh in meshes)  # type: ignore[union-attr]
                all_tri = all(bool(mesh.get("all_triangles")) for mesh in meshes)  # type: ignore[union-attr]
                lines.append(f"- `{link_name}` collision meshes: {len(meshes)}, faces: {face_total}, all triangles: {all_tri}")
        lines.append("")

    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text("\n".join(lines), encoding="utf-8")


def process_one(
    source: Path,
    output: Path,
    report_dir: Path,
    normalize_finger_joints: str,
    base_collision_source: str,
    include_large_arrays: bool,
    source_root_path: str | None = None,
    config_name: str | None = None,
) -> dict[str, object]:
    source = source.resolve()
    output = output.resolve()
    report_dir = report_dir.resolve()
    report_dir.mkdir(parents=True, exist_ok=True)

    original_stage = open_stage(source)
    original_summary = write_stage_report(original_stage, report_dir / "wrapper_structure.txt", include_large_arrays)

    inferred_from_source = infer_gripper(original_stage)
    source_root_path = source_root_path or str(inferred_from_source["root_path"])
    extraction_path, extraction_stage, gripper_info, discovery_notes = find_extraction_stage(source, source_root_path)
    extraction_summary = write_stage_report(extraction_stage, report_dir / "composed_source_structure.txt", include_large_arrays)

    root_path = str(gripper_info["root_path"])
    base_link = str(gripper_info["base_link"])
    finger_links = list(gripper_info["finger_links"])
    finger_joints = list(gripper_info["finger_joints"])

    extraction_notes = extract_gripper(
        source_stage=extraction_stage,
        output_path=output,
        root_path=root_path,
        base_link=base_link,
        finger_links=finger_links,
        finger_joints=finger_joints,
        normalize_mode=normalize_finger_joints,
        base_collision_source=base_collision_source,
    )

    output_stage = open_stage(output)
    output_summary = write_stage_report(output_stage, report_dir / "gripper_output_structure.txt", include_large_arrays)
    output_external_arcs = count_external_arcs(output_stage)
    collision_meshes = output_mesh_summary(output_stage, root_path, [base_link, *finger_links])

    config_key = config_name or f"{source.stem}_gripper"
    manifest: dict[str, object] = {
        "ok": True,
        "source": str(source),
        "extraction_source": str(extraction_path),
        "output": str(output),
        "root_path": root_path,
        "base_frame": base_link,
        "finger_colliders": finger_links,
        "finger_joints": finger_joints,
        "recommended_graspdatagen_config": {
            config_key: {
                "gripper_file": relative_string(output),
                "finger_colliders": finger_links,
                "base_frame": base_link,
                "bite": 0.01,
                "pinch_width_resolution": 8,
            }
        },
        "reports": {
            "wrapper": str(report_dir / "wrapper_structure.txt"),
            "wrapper_json": str(report_dir / "wrapper_structure.json"),
            "composed_source": str(report_dir / "composed_source_structure.txt"),
            "composed_source_json": str(report_dir / "composed_source_structure.json"),
            "output": str(report_dir / "gripper_output_structure.txt"),
            "output_json": str(report_dir / "gripper_output_structure.json"),
        },
        "summary": {
            "wrapper_prim_count": original_summary["prim_count"],
            "composed_source_prim_count": extraction_summary["prim_count"],
            "output_prim_count": output_summary["prim_count"],
            "output_external_arcs": output_external_arcs,
        },
        "output_collision_meshes": collision_meshes,
        "notes": discovery_notes + extraction_notes,
    }
    (report_dir / "manifest.json").write_text(json.dumps(manifest, indent=2, ensure_ascii=False), encoding="utf-8")
    return manifest


def process_batch(args: argparse.Namespace) -> dict[str, object]:
    source_files = sorted(Path().glob(args.source_glob))
    batch_report_dir = Path(args.batch_report_dir).resolve()
    output_dir = Path(args.output_dir)
    manifests: list[dict[str, object]] = []

    for source in source_files:
        output = output_dir / f"{source.stem}_gripper.usd"
        report_dir = batch_report_dir / source.stem
        try:
            manifest = process_one(
                source=source,
                output=output,
                report_dir=report_dir,
                normalize_finger_joints=args.normalize_finger_joints,
                base_collision_source=args.base_collision_source,
                include_large_arrays=args.include_large_arrays,
            )
        except Exception as exc:  # pragma: no cover - batch should report all failures
            manifest = {
                "ok": False,
                "source": str(source.resolve()),
                "output": str(output.resolve()),
                "error": f"{type(exc).__name__}: {exc}",
            }
            report_dir.mkdir(parents=True, exist_ok=True)
            (report_dir / "manifest.json").write_text(json.dumps(manifest, indent=2, ensure_ascii=False), encoding="utf-8")
        manifests.append(manifest)

    summary = {
        "ok": all(manifest.get("ok", False) for manifest in manifests),
        "source_glob": args.source_glob,
        "count": len(manifests),
        "success_count": sum(1 for manifest in manifests if manifest.get("ok", False)),
        "failure_count": sum(1 for manifest in manifests if not manifest.get("ok", False)),
        "manifests": manifests,
    }
    batch_report_dir.mkdir(parents=True, exist_ok=True)
    (batch_report_dir / "batch_manifest.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    write_markdown_batch_report(manifests, batch_report_dir / "batch_report.md")
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", default=DEFAULT_SOURCE, help="Input Piper robot USD wrapper.")
    parser.add_argument("--output", default="", help="Output gripper USD. Default: bots/<source_stem>_gripper.usd.")
    parser.add_argument("--report-dir", default="", help="Directory for tree/json reports. Default: debug_output/usd_inspect/<source_stem>.")
    parser.add_argument("--source-glob", default=DEFAULT_BATCH_SOURCE_GLOB, help="Batch input glob used with --all.")
    parser.add_argument("--output-dir", default="bots", help="Batch output directory used with --all.")
    parser.add_argument("--batch-report-dir", default=DEFAULT_BATCH_REPORT_DIR, help="Batch report directory used with --all.")
    parser.add_argument("--all", action="store_true", help="Process all Piper USD files matched by --source-glob.")
    parser.add_argument("--root-path", default="", help="Optional robot root prim path override. By default it is inferred from joint7.")
    parser.add_argument(
        "--normalize-finger-joints",
        choices=["positive", "none"],
        default="positive",
        help="Normalize negative finger joint ranges to positive ranges. Default: positive.",
    )
    parser.add_argument(
        "--base-collision-source",
        choices=["visuals", "colliders"],
        default="visuals",
        help="Use visuals as base collision for Piper link6 because the source collider is camera-heavy.",
    )
    parser.add_argument("--include-large-arrays", action="store_true", help="Write full mesh arrays into reports.")
    args = parser.parse_args()

    if args.all:
        print(json.dumps(process_batch(args), indent=2, ensure_ascii=False))
        return

    source = Path(args.source)
    output = Path(args.output) if args.output else Path("bots") / f"{source.stem}_gripper.usd"
    report_dir = Path(args.report_dir) if args.report_dir else Path("debug_output/usd_inspect") / source.stem
    manifest = process_one(
        source=source,
        output=output,
        report_dir=report_dir,
        normalize_finger_joints=args.normalize_finger_joints,
        base_collision_source=args.base_collision_source,
        include_large_arrays=args.include_large_arrays,
        source_root_path=args.root_path or None,
    )
    print(json.dumps(manifest, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
