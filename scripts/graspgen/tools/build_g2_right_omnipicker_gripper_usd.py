#!/usr/bin/env python3
"""Extract the right-arm G2 OmniPicker gripper as a standalone USD.

The source robot is a full G2 robot. This tool builds a pure gripper asset for
GraspDataGen from /genie by copying only the right gripper links and their
internal joints. It keeps source physics settings where possible, removes
collision APIs from visual meshes, and creates collision meshes by copying the
visual mesh subtrees into /collisions.
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Iterable

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

try:
    from pxr import PhysxSchema
except ImportError:  # pragma: no cover - depends on Isaac Sim USD plugins.
    PhysxSchema = None


DEFAULT_SOURCE = "robot/G2_omnipicker/robot_fix.usda"
DEFAULT_OUTPUT = "bots/g2_right_omnipicker_gripper.usd"
DEFAULT_SOURCE_ROOT = "/genie"
DEFAULT_ROOT_NAME = "G2RightOmniPicker"
DEFAULT_LINK_PREFIX = "gripper_r_"
DEFAULT_BASE_LINK = "gripper_r_base_link"
DEFAULT_SKIP_LINKS = ("gripper_r_center_link",)
DEFAULT_SKIP_CHILDREN = ("Right_Camera", "Cam_R")
DEFAULT_FINGER_LINKS = ("gripper_r_inner_link4", "gripper_r_outer_link4")
DEFAULT_COLLISION_APPROXIMATION = "convexHull"
DEFAULT_DRIVE_STIFFNESS = 10.0
DEFAULT_DRIVE_DAMPING = 1.0


def _path_has_prefix(path: Sdf.Path, prefix: Sdf.Path) -> bool:
    return path == prefix or path.HasPrefix(prefix)


def _is_joint(prim: Usd.Prim) -> bool:
    return prim.GetTypeName().startswith("Physics") and prim.GetTypeName().endswith("Joint")


def _relationship_targets(prim: Usd.Prim, rel_name: str) -> list[Sdf.Path]:
    rel = prim.GetRelationship(rel_name)
    return list(rel.GetTargets()) if rel else []


def _body_targets(prim: Usd.Prim) -> list[Sdf.Path]:
    return _relationship_targets(prim, "physics:body0") + _relationship_targets(prim, "physics:body1")


def _copy_spec(source_layer: Sdf.Layer, dest_stage: Usd.Stage, source_path: Sdf.Path, dest_path: Sdf.Path) -> None:
    if not source_layer.GetPrimAtPath(source_path):
        return
    if dest_stage.GetPrimAtPath(dest_path):
        dest_stage.RemovePrim(dest_path)
    if not Sdf.CopySpec(source_layer, source_path, dest_stage.GetRootLayer(), dest_path):
        raise RuntimeError(f"failed to copy USD spec {source_path} -> {dest_path}")


def _iter_prims_under(root: Usd.Prim, excluded_prefixes: Iterable[Sdf.Path] = ()) -> Iterable[Usd.Prim]:
    excluded = tuple(excluded_prefixes)
    if not root:
        return
    for prim in Usd.PrimRange(root):
        prim_path = prim.GetPath()
        if any(_path_has_prefix(prim_path, prefix) for prefix in excluded):
            continue
        yield prim


def _discover_links(
    stage: Usd.Stage,
    source_root_path: Sdf.Path,
    link_prefix: str,
    skip_links: set[str],
) -> tuple[list[Sdf.Path], list[Sdf.Path]]:
    source_root = stage.GetPrimAtPath(source_root_path)
    if not source_root:
        raise ValueError(f"source root does not exist: {source_root_path}")

    discovered: list[Sdf.Path] = []
    robot_links = source_root.GetRelationship("isaac:physics:robotLinks")
    if robot_links:
        for target in robot_links.GetTargets():
            prim = stage.GetPrimAtPath(target)
            if prim and prim.GetName().startswith(link_prefix):
                discovered.append(target)

    seen = set(discovered)
    for child in source_root.GetChildren():
        child_path = child.GetPath()
        if child.GetName().startswith(link_prefix) and child_path not in seen:
            discovered.append(child_path)
            seen.add(child_path)

    skipped = [path for path in discovered if path.name in skip_links]
    kept = [path for path in discovered if path.name not in skip_links]
    return kept, skipped


def _discover_joints(stage: Usd.Stage, source_root_path: Sdf.Path, link_paths: list[Sdf.Path]) -> list[Sdf.Path]:
    link_set = set(link_paths)
    joint_paths: list[Sdf.Path] = []
    for scope_name in ("joints", "loop_joints"):
        scope = stage.GetPrimAtPath(source_root_path.AppendChild(scope_name))
        if not scope:
            continue
        for prim in Usd.PrimRange(scope):
            if not _is_joint(prim):
                continue
            targets = _body_targets(prim)
            if targets and all(target in link_set for target in targets):
                joint_paths.append(prim.GetPath())
    return joint_paths


def _discover_excluded_children(
    stage: Usd.Stage,
    link_paths: list[Sdf.Path],
    skip_child_names: set[str],
) -> list[Sdf.Path]:
    excluded: list[Sdf.Path] = []
    for link_path in link_paths:
        link = stage.GetPrimAtPath(link_path)
        for prim in Usd.PrimRange(link):
            if prim.GetName() in skip_child_names or prim.GetTypeName() == "Camera":
                excluded.append(prim.GetPath())
    return excluded


def _clear_xform_ops(prim: Usd.Prim) -> None:
    for prop in list(prim.GetProperties()):
        name = prop.GetName()
        if name == "xformOpOrder" or name.startswith("xformOp:"):
            prim.RemoveProperty(name)


def _set_relative_link_transforms(
    source_stage: Usd.Stage,
    dest_stage: Usd.Stage,
    source_to_dest: dict[Sdf.Path, Sdf.Path],
    source_base_path: Sdf.Path,
) -> None:
    base_world = UsdGeom.Xformable(source_stage.GetPrimAtPath(source_base_path)).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default()
    )
    base_world_inverse = base_world.GetInverse()

    for source_path, dest_path in source_to_dest.items():
        dest_prim = dest_stage.GetPrimAtPath(dest_path)
        _clear_xform_ops(dest_prim)
        if source_path == source_base_path:
            continue

        source_world = UsdGeom.Xformable(source_stage.GetPrimAtPath(source_path)).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()
        )
        relative = source_world * base_world_inverse
        UsdGeom.Xformable(dest_prim).AddTransformOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(relative)


def _internal_reference_paths(prim: Usd.Prim) -> list[Sdf.Path]:
    refs = prim.GetMetadata("references")
    if not refs:
        return []
    paths: list[Sdf.Path] = []
    for ref in refs.GetAddedOrExplicitItems():
        if not ref.assetPath and ref.primPath:
            paths.append(Sdf.Path(ref.primPath))
    return paths


def _materialize_internal_references(source_layer: Sdf.Layer, dest_stage: Usd.Stage) -> list[str]:
    materialized: list[str] = []
    for _ in range(8):
        changed = False
        for prim in list(dest_stage.Traverse()):
            reference_paths = _internal_reference_paths(prim)
            if not reference_paths:
                continue

            prim.GetReferences().ClearReferences()
            prim.SetInstanceable(False)
            for reference_path in reference_paths:
                reference_spec = source_layer.GetPrimAtPath(reference_path)
                if not reference_spec:
                    continue
                for child_spec in reference_spec.nameChildren:
                    dest_child_path = prim.GetPath().AppendChild(child_spec.name)
                    _copy_spec(source_layer, dest_stage, child_spec.path, dest_child_path)
                    materialized.append(f"{prim.GetPath()} -> {dest_child_path}")
                    changed = True
        if not changed:
            break
    return materialized


def _remove_api_if_present(prim: Usd.Prim, api_schema) -> None:
    if api_schema is None:
        return
    try:
        prim.RemoveAPI(api_schema)
    except Exception:
        pass


def _remove_authored_properties(prim: Usd.Prim, names: tuple[str, ...]) -> None:
    for name in names:
        if prim.HasProperty(name):
            prim.RemoveProperty(name)


def _strip_collision_from_visuals(dest_stage: Usd.Stage, link_paths: list[Sdf.Path]) -> list[str]:
    changed: list[str] = []
    physx_apis = []
    if PhysxSchema is not None:
        physx_apis = [
            PhysxSchema.PhysxCollisionAPI,
            PhysxSchema.PhysxConvexHullCollisionAPI,
            PhysxSchema.PhysxConvexDecompositionCollisionAPI,
            PhysxSchema.PhysxSDFMeshCollisionAPI,
            PhysxSchema.PhysxTriangleMeshCollisionAPI,
            PhysxSchema.PhysxTriangleMeshSimplificationCollisionAPI,
            PhysxSchema.PhysxSphereFillCollisionAPI,
        ]

    for link_path in link_paths:
        visuals = dest_stage.GetPrimAtPath(link_path.AppendChild("visuals"))
        if not visuals:
            continue
        for prim in Usd.PrimRange(visuals):
            before = set(prim.GetAppliedSchemas())
            _remove_api_if_present(prim, UsdPhysics.CollisionAPI)
            _remove_api_if_present(prim, UsdPhysics.MeshCollisionAPI)
            for api in physx_apis:
                _remove_api_if_present(prim, api)
            _remove_authored_properties(
                prim,
                (
                    "physics:collisionEnabled",
                    "physics:approximation",
                    "physics:simulationOwner",
                    "physxCollision:contactOffset",
                    "physxCollision:restOffset",
                    "material:binding:physics",
                ),
            )
            after = set(prim.GetAppliedSchemas())
            if before != after or (prim.GetTypeName() == "Mesh" and "PhysicsCollisionAPI" in before):
                changed.append(str(prim.GetPath()))
    return changed


def _apply_collision_apis(
    mesh_prim: Usd.Prim,
    approximation: str,
    physics_material_path: Sdf.Path | None,
) -> None:
    collision_api = UsdPhysics.CollisionAPI.Apply(mesh_prim)
    collision_api.CreateCollisionEnabledAttr().Set(True)
    mesh_collision_api = UsdPhysics.MeshCollisionAPI.Apply(mesh_prim)
    approximation_attr = mesh_collision_api.GetApproximationAttr()
    if not approximation_attr or approximation_attr.Get() in (None, "none"):
        mesh_collision_api.CreateApproximationAttr().Set(approximation)

    if PhysxSchema is not None:
        PhysxSchema.PhysxCollisionAPI.Apply(mesh_prim)

    if physics_material_path is not None:
        rel = mesh_prim.GetRelationship("material:binding:physics")
        if not rel or not rel.GetTargets():
            mesh_prim.CreateRelationship("material:binding:physics").SetTargets([physics_material_path])


def _copy_visuals_to_collisions(
    dest_stage: Usd.Stage,
    link_paths: list[Sdf.Path],
    approximation: str,
    physics_material_path: Sdf.Path | None,
) -> list[str]:
    layer = dest_stage.GetRootLayer()
    collision_meshes: list[str] = []

    for link_path in link_paths:
        visuals_path = link_path.AppendChild("visuals")
        collisions_path = link_path.AppendChild("collisions")
        if not dest_stage.GetPrimAtPath(visuals_path):
            continue
        if dest_stage.GetPrimAtPath(collisions_path):
            dest_stage.RemovePrim(collisions_path)
        if not Sdf.CopySpec(layer, visuals_path, layer, collisions_path):
            raise RuntimeError(f"failed to copy {visuals_path} -> {collisions_path}")

        collisions = dest_stage.GetPrimAtPath(collisions_path)
        collisions.SetActive(True)
        for prim in Usd.PrimRange(collisions):
            if prim.GetTypeName() != "Mesh":
                continue
            _apply_collision_apis(prim, approximation, physics_material_path)
            collision_meshes.append(str(prim.GetPath()))

    return collision_meshes


def _remap_path(path: Sdf.Path, prefix_map: list[tuple[Sdf.Path, Sdf.Path]]) -> Sdf.Path:
    for source_prefix, dest_prefix in prefix_map:
        if _path_has_prefix(path.GetPrimPath(), source_prefix):
            return path.ReplacePrefix(source_prefix, dest_prefix)
    return path


def _remap_targets_and_connections(dest_stage: Usd.Stage, prefix_map: dict[Sdf.Path, Sdf.Path]) -> None:
    sorted_prefixes = sorted(prefix_map.items(), key=lambda item: len(item[0].pathString), reverse=True)
    for prim in dest_stage.Traverse():
        for rel in prim.GetRelationships():
            targets = rel.GetTargets()
            if targets:
                rel.SetTargets([_remap_path(target, sorted_prefixes) for target in targets])
        for attr in prim.GetAttributes():
            connections = attr.GetConnections()
            if connections:
                attr.SetConnections([_remap_path(target, sorted_prefixes) for target in connections])


def _copy_root_articulation_settings(source_root: Usd.Prim, dest_root: Usd.Prim) -> None:
    UsdPhysics.ArticulationRootAPI.Apply(dest_root)
    # Apply PhysxArticulationAPI via metadata so it works even when the PhysxSchema
    # Python module is not importable (e.g. outside Isaac Sim).
    _apply_api_schema(dest_root, "PhysxArticulationAPI")
    for attr in source_root.GetAttributes():
        if attr.GetName().startswith("physxArticulation:"):
            copied = dest_root.CreateAttribute(attr.GetName(), attr.GetTypeName())
            copied.Set(attr.Get())
    # Stability settings for mimic-coupled linkages (critical for lightweight
    # grippers extracted from full robots — without these the articulation solver
    # defaults to ~4 position iterations, insufficient for parallel 4-bar linkages
    # and mimic followers, causing links to detach and the whole chain to explode).
    _ensure_attribute(dest_root, "physxArticulation:solverPositionIterationCount", Sdf.ValueTypeNames.Int, 32)
    _ensure_attribute(dest_root, "physxArticulation:solverVelocityIterationCount", Sdf.ValueTypeNames.Int, 1)
    _ensure_attribute(dest_root, "physxArticulation:enabledSelfCollisions", Sdf.ValueTypeNames.Bool, False)


def _ensure_attribute(prim: Usd.Prim, name: str, type_name: Sdf.ValueTypeName, value: object) -> None:
    """Set *value* on *prim* for *name*, creating the attribute only when it is
    absent or unauthored (avoids overwriting an explicit source-authored value)."""
    attr = prim.GetAttribute(name)
    if not attr or not attr.HasValue():
        prim.CreateAttribute(name, type_name).Set(value)


def _apply_api_schema(prim: Usd.Prim, api_name: str) -> None:
    """Add *api_name* to the prim's apiSchemas metadata if not already present.

    This sets the metadata directly as a string token rather than through a
    schema Python class, so it works even when the corresponding Python module
    (e.g. PhysxSchema) is not importable in the current environment.
    """
    existing = prim.GetMetadata("apiSchemas")
    api_names = _token_list_op_items(existing)
    if api_name in api_names:
        return
    api_names.append(api_name)
    prim.SetMetadata("apiSchemas", Sdf.TokenListOp.CreateExplicit(api_names))


def _token_list_op_items(op) -> list[str]:
    """Collect all items from an SdfTokenListOp into a plain list."""
    items: list[str] = []
    for field in ("prependedItems", "appendedItems", "explicitItems"):
        for item in getattr(op, field, ()) or ():
            items.append(item)
    return items


def _create_root_joint(dest_stage: Usd.Stage, root_path: Sdf.Path, base_path: Sdf.Path) -> Sdf.Path:
    root_joint_path = root_path.AppendChild("root_joint")
    joint = UsdPhysics.FixedJoint.Define(dest_stage, root_joint_path)
    joint.CreateBody1Rel().SetTargets([base_path])
    joint.CreateBreakForceAttr().Set(3.4028234663852886e38)
    joint.CreateBreakTorqueAttr().Set(3.4028234663852886e38)
    joint.CreateCollisionEnabledAttr().Set(False)
    joint.CreateJointEnabledAttr().Set(True)
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    identity = Gf.Quatf(1.0, Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot0Attr().Set(identity)
    joint.CreateLocalRot1Attr().Set(identity)
    return root_joint_path


def _override_drive_parameters(
    dest_stage: Usd.Stage,
    joint_map: dict[Sdf.Path, Sdf.Path],
    stiffness: float,
    damping: float,
) -> list[str]:
    """Override drive stiffness and damping on all driven joints.

    PD controllers with too-high stiffness cause the gripper to snap shut
    instantly (invisible in headed mode) and generate unstable contact forces
    that undermine grasp validation.  Matching the OmniPicker reference values
    (stiffness=10, damping=1) gives a visible ~1 s closing phase and stable
    object contact.
    """
    overridden: list[str] = []
    for _, dest_path in joint_map.items():
        prim = dest_stage.GetPrimAtPath(dest_path)
        if not prim:
            continue
        try:
            drive = UsdPhysics.DriveAPI.Get(prim, "angular")
        except Exception:
            drive = None
        if drive is None or not drive:
            continue
        # Only touch joints that already have a drive authored.
        overridden.append(str(dest_path))
        stiffness_attr = drive.GetStiffnessAttr()
        if not stiffness_attr:
            stiffness_attr = drive.CreateStiffnessAttr(1.0)
        stiffness_attr.Set(stiffness)

        damping_attr = drive.GetDampingAttr()
        if not damping_attr:
            damping_attr = drive.CreateDampingAttr(1.0)
        damping_attr.Set(damping)
    return overridden


def _author_robot_relationships(
    root: Usd.Prim,
    link_paths: list[Sdf.Path],
    joint_paths: list[Sdf.Path],
    root_joint_path: Sdf.Path,
) -> None:
    root.CreateRelationship("isaac:physics:robotLinks").SetTargets(link_paths)
    root.CreateRelationship("isaac:physics:robotJoints").SetTargets(joint_paths + [root_joint_path])


def _copy_external_roots(
    source_layer: Sdf.Layer,
    dest_stage: Usd.Stage,
    source_root_path: Sdf.Path,
    root_path: Sdf.Path,
) -> dict[Sdf.Path, Sdf.Path]:
    external_map: dict[Sdf.Path, Sdf.Path] = {}
    for name in ("Looks", "PhysicsMaterial"):
        source_path = source_root_path.AppendChild(name)
        if not source_layer.GetPrimAtPath(source_path):
            continue
        dest_path = root_path.AppendChild(name)
        _copy_spec(source_layer, dest_stage, source_path, dest_path)
        external_map[source_path] = dest_path
    return external_map


def _build(args: argparse.Namespace) -> dict[str, object]:
    source_file = Path(args.source)
    output_file = Path(args.output)
    source_root_path = Sdf.Path(args.source_root)
    root_path = Sdf.Path(f"/{args.root_name}")

    if not source_file.exists():
        raise FileNotFoundError(source_file)
    if output_file.exists() and not args.overwrite and not args.dry_run:
        raise FileExistsError(f"{output_file} already exists; pass --overwrite")

    source_stage = Usd.Stage.Open(str(source_file), Usd.Stage.LoadAll)
    source_root = source_stage.GetPrimAtPath(source_root_path)
    if not source_root:
        raise ValueError(f"source root does not exist: {source_root_path}")

    link_paths, skipped_links = _discover_links(source_stage, source_root_path, args.link_prefix, set(args.skip_link))
    if not link_paths:
        raise RuntimeError(f"no gripper links found under {source_root_path}")

    source_base_path = source_root_path.AppendChild(args.base_link)
    if source_base_path not in link_paths:
        raise RuntimeError(f"base link is not included in extracted links: {source_base_path}")

    joint_paths = _discover_joints(source_stage, source_root_path, link_paths)
    excluded_children = _discover_excluded_children(source_stage, link_paths, set(args.skip_child))
    flat_layer = source_stage.Flatten()

    link_map = {source: root_path.AppendChild(source.name) for source in link_paths}
    joint_map = {
        source: root_path.AppendChild(source.GetParentPath().name).AppendChild(source.name)
        for source in joint_paths
    }

    if args.dry_run:
        return {
            "source": str(source_file),
            "output": str(output_file),
            "root": str(root_path),
            "links": [str(path) for path in link_paths],
            "skipped_links": [str(path) for path in skipped_links],
            "joints": [str(path) for path in joint_paths],
            "excluded_children": [str(path) for path in excluded_children],
            "wrote": False,
        }

    output_file.parent.mkdir(parents=True, exist_ok=True)
    if output_file.exists():
        output_file.unlink()

    dest_stage = Usd.Stage.CreateNew(str(output_file))
    if not dest_stage:
        raise RuntimeError(f"failed to create destination stage: {output_file}")

    meters_per_unit = source_stage.GetMetadata("metersPerUnit")
    if meters_per_unit is not None:
        UsdGeom.SetStageMetersPerUnit(dest_stage, meters_per_unit)
    up_axis = source_stage.GetMetadata("upAxis")
    if up_axis is not None:
        UsdGeom.SetStageUpAxis(dest_stage, up_axis)

    root = UsdGeom.Xform.Define(dest_stage, root_path).GetPrim()
    dest_stage.SetDefaultPrim(root)
    _copy_root_articulation_settings(source_root, root)

    external_map = _copy_external_roots(flat_layer, dest_stage, source_root_path, root_path)
    for source, dest in link_map.items():
        _copy_spec(flat_layer, dest_stage, source, dest)
    materialized_references = _materialize_internal_references(flat_layer, dest_stage)

    for excluded in excluded_children:
        for source_link, dest_link in link_map.items():
            if _path_has_prefix(excluded, source_link):
                dest_stage.RemovePrim(excluded.ReplacePrefix(source_link, dest_link))
                break

    for source, dest in joint_map.items():
        UsdGeom.Scope.Define(dest_stage, dest.GetParentPath())
        _copy_spec(flat_layer, dest_stage, source, dest)

    overridden_drives = _override_drive_parameters(
        dest_stage, joint_map, args.drive_stiffness, args.drive_damping
    )

    _set_relative_link_transforms(source_stage, dest_stage, link_map, source_base_path)

    physics_material_path = external_map.get(source_root_path.AppendChild("PhysicsMaterial"))
    stripped_visual_collision = _strip_collision_from_visuals(dest_stage, list(link_map.values()))
    collision_meshes = _copy_visuals_to_collisions(
        dest_stage,
        list(link_map.values()),
        args.collision_approximation,
        physics_material_path,
    )
    materialized_references.extend(_materialize_internal_references(flat_layer, dest_stage))

    prefix_map: dict[Sdf.Path, Sdf.Path] = {}
    prefix_map.update(link_map)
    prefix_map.update(joint_map)
    prefix_map.update(external_map)
    _remap_targets_and_connections(dest_stage, prefix_map)

    root_joint_path = _create_root_joint(dest_stage, root_path, link_map[source_base_path])
    robot_joint_targets = [
        joint_map[source]
        for source in joint_paths
        if source.GetParentPath().name == "joints"
    ]
    _author_robot_relationships(root, list(link_map.values()), robot_joint_targets, root_joint_path)

    dest_stage.GetRootLayer().Save()
    return {
        "source": str(source_file),
        "output": str(output_file),
        "root": str(root_path),
        "links": [str(path) for path in link_paths],
        "skipped_links": [str(path) for path in skipped_links],
        "joints": [str(path) for path in joint_paths],
        "robot_joints": [str(path) for path in robot_joint_targets] + [str(root_joint_path)],
        "excluded_children": [str(path) for path in excluded_children],
        "materialized_references": materialized_references,
        "stripped_visual_collision": stripped_visual_collision,
        "collision_meshes": collision_meshes,
        "overridden_drives": overridden_drives,
        "drive_stiffness": args.drive_stiffness,
        "drive_damping": args.drive_damping,
        "finger_colliders": list(args.finger_colliders),
        "base_frame": args.base_link,
        "wrote": True,
    }


def _print_summary(summary: dict[str, object]) -> None:
    print(f"Source: {summary['source']}")
    print(f"Output: {summary['output']}")
    print(f"Root: {summary['root']}")
    print(f"Links: {len(summary['links'])}")
    for path in summary["links"]:
        print(f"  {path}")
    if summary.get("skipped_links"):
        print(f"Skipped links: {len(summary['skipped_links'])}")
        for path in summary["skipped_links"]:
            print(f"  {path}")
    print(f"Joints: {len(summary['joints'])}")
    for path in summary["joints"]:
        print(f"  {path}")
    if summary.get("robot_joints"):
        print(f"Robot joints: {len(summary['robot_joints'])}")
        for path in summary["robot_joints"]:
            print(f"  {path}")
    if summary.get("excluded_children"):
        print(f"Excluded children: {len(summary['excluded_children'])}")
        for path in summary["excluded_children"]:
            print(f"  {path}")
    if summary.get("materialized_references"):
        print(f"Materialized internal references: {len(summary['materialized_references'])}")
    if summary.get("stripped_visual_collision"):
        print(f"Stripped visual collision prims: {len(summary['stripped_visual_collision'])}")
    if summary.get("collision_meshes"):
        print(f"Copied visual meshes to collisions: {len(summary['collision_meshes'])}")
        for path in summary["collision_meshes"]:
            print(f"  {path}")
    if summary.get("overridden_drives"):
        print(
            f"Overridden drive parameters (stiffness={summary.get('drive_stiffness')}, damping={summary.get('drive_damping')}): "
            f"{len(summary['overridden_drives'])}"
        )
        for path in summary["overridden_drives"]:
            print(f"  {path}")
    print(f"Wrote output: {summary['wrote']}")
    print(f"Suggested base_frame: {summary.get('base_frame', DEFAULT_BASE_LINK)}")
    print(f"Suggested finger_colliders: {summary.get('finger_colliders', list(DEFAULT_FINGER_LINKS))}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", default=DEFAULT_SOURCE, help="Source full robot USD")
    parser.add_argument("--output", default=DEFAULT_OUTPUT, help="Destination standalone gripper USD")
    parser.add_argument("--source_root", default=DEFAULT_SOURCE_ROOT, help="Source robot root prim path")
    parser.add_argument("--root_name", default=DEFAULT_ROOT_NAME, help="Destination default prim name")
    parser.add_argument("--link_prefix", default=DEFAULT_LINK_PREFIX, help="Link prefix to extract")
    parser.add_argument("--base_link", default=DEFAULT_BASE_LINK, help="Base link name")
    parser.add_argument(
        "--skip_link",
        action="append",
        default=list(DEFAULT_SKIP_LINKS),
        help="Link name to omit from the standalone gripper; repeatable",
    )
    parser.add_argument(
        "--skip_child",
        action="append",
        default=list(DEFAULT_SKIP_CHILDREN),
        help="Child prim name to remove from copied links; repeatable",
    )
    parser.add_argument(
        "--finger_colliders",
        nargs=2,
        default=list(DEFAULT_FINGER_LINKS),
        help="Printed as the suggested GraspDataGen finger_colliders value",
    )
    parser.add_argument(
        "--collision_approximation",
        default=DEFAULT_COLLISION_APPROXIMATION,
        help="Mesh collision approximation authored when the copied visual mesh lacks one",
    )
    parser.add_argument("--dry_run", action="store_true", help="Print extraction set without writing")
    parser.add_argument("--overwrite", action="store_true", help="Overwrite output USD")
    parser.add_argument(
        "--drive_stiffness",
        type=float,
        default=DEFAULT_DRIVE_STIFFNESS,
        help=f"PD controller stiffness for driven joints (default: {DEFAULT_DRIVE_STIFFNESS})",
    )
    parser.add_argument(
        "--drive_damping",
        type=float,
        default=DEFAULT_DRIVE_DAMPING,
        help=f"PD controller damping for driven joints (default: {DEFAULT_DRIVE_DAMPING})",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    summary = _build(args)
    _print_summary(summary)


if __name__ == "__main__":
    main()
