# /// script
# requires-python = ">=3.10"
# dependencies = ["usd-core"]
# ///
"""Extract the right G2 OmniPicker gripper from robot_fix.usda.

This script intentionally does not trust the collision setup authored directly
on robot/G2_omnipicker/robot_fix.usda. In that stage the gripper collision
containers are inactive and the visual meshes carry collision APIs. For grasp
generation we copy articulation metadata from robot_fix.usda, but replace every
link's visuals/collisions from configuration/robot_physics.usd:

  /visuals/<link>   -> /genie/<link>/visuals
  /colliders/<link> -> /genie/<link>/collisions

The default output keeps the source link2 loop bodies as base-driven mimic
followers, matching the standalone OmniPicker graph. It drops only the source
spherical loop constraints that close link2 back to the base. Those closed-loop
constraints make IsaacLab's gripper definition solve discontinuous finger poses,
which produces non-monotonic or negative open widths.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Iterable

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics


DEFAULT_ROBOT_STAGE = Path("robot/G2_omnipicker/robot_fix.usda")
DEFAULT_GEOMETRY_STAGE = Path("robot/G2_omnipicker/configuration/robot_physics.usd")
DEFAULT_OUTPUT = Path("bots/robot_g2_omnipicker_gripper.usd")
DEFAULT_REPORT = Path("debug_output/usd_inspect/g2_omnipicker_from_robot_fix.json")

ROOT_PATH = "/genie"
BASE_LINK = "gripper_r_base_link"
FINGER_COLLIDERS = ("gripper_r_inner_link4", "gripper_r_outer_link4")
GRIPPER_LINKS = [
    "gripper_r_base_link",
    "gripper_r_inner_link1",
    "gripper_r_inner_link3",
    "gripper_r_inner_link4",
    "gripper_r_inner_link2",
    "gripper_r_outer_link1",
    "gripper_r_outer_link3",
    "gripper_r_outer_link4",
    "gripper_r_outer_link2",
]
MAIN_JOINTS = [
    "idx71_gripper_r_inner_joint1",
    "idx72_gripper_r_inner_joint3",
    "idx73_gripper_r_inner_joint4",
    "idx79_gripper_r_inner_joint0",
    "idx81_gripper_r_outer_joint1",
    "idx82_gripper_r_outer_joint3",
    "idx83_gripper_r_outer_joint4",
    "idx89_gripper_r_outer_joint0",
]
REMOVED_CLOSED_LOOP_JOINTS = [
    "idx93_gripper_r_outer_joint2",
    "idx94_gripper_r_inner_joint2",
]

PHYSICS_COLLISION_SCHEMAS = (
    "PhysicsCollisionAPI",
    "PhysxCollisionAPI",
    "PhysicsMeshCollisionAPI",
    "PhysxConvexHullCollisionAPI",
    "PhysxConvexDecompositionCollisionAPI",
    "PhysxSDFMeshCollisionAPI",
    "PhysxTriangleMeshSimplificationCollisionAPI",
    "PhysxSphereFillCollisionAPI",
)
PHYSICS_ATTR_PREFIXES = (
    "physics:collisionEnabled",
    "physics:approximation",
    "physxCollision:",
    "physxConvex",
    "physxSDF",
)


def open_stage(path: Path) -> Usd.Stage:
    stage = Usd.Stage.Open(str(path), load=Usd.Stage.LoadAll)
    if stage is None:
        raise RuntimeError(f"Could not open USD stage: {path}")
    return stage


def path_exists(layer: Sdf.Layer, path: str) -> bool:
    return layer.GetObjectAtPath(Sdf.Path(path)) is not None


def copy_spec(src_layer: Sdf.Layer, src: str, dst_layer: Sdf.Layer, dst: str) -> None:
    if not path_exists(src_layer, src):
        raise RuntimeError(f"Missing source USD spec: {src}")
    if not Sdf.CopySpec(src_layer, Sdf.Path(src), dst_layer, Sdf.Path(dst)):
        raise RuntimeError(f"Failed to copy USD spec {src} -> {dst}")


def authored_api_schemas(prim: Usd.Prim) -> list[str]:
    list_op = prim.GetMetadata("apiSchemas")
    if isinstance(list_op, Sdf.TokenListOp):
        items: list[str] = []
        for item in list(list_op.explicitItems) + list(list_op.prependedItems) + list(list_op.addedItems):
            if item not in items:
                items.append(item)
        if items:
            return items
    return list(prim.GetAppliedSchemas())


def set_api_schemas(prim: Usd.Prim, schemas: Iterable[str]) -> None:
    unique: list[str] = []
    for schema in schemas:
        if schema not in unique:
            unique.append(schema)
    prim.SetMetadata("apiSchemas", Sdf.TokenListOp.CreateExplicit(unique))


def ensure_api_schema(prim: Usd.Prim, schema: str) -> None:
    schemas = authored_api_schemas(prim)
    if schema not in schemas:
        schemas.append(schema)
    set_api_schemas(prim, schemas)


def remove_api_schemas(prim: Usd.Prim, names: Iterable[str]) -> None:
    names = set(names)
    set_api_schemas(prim, [schema for schema in authored_api_schemas(prim) if schema not in names])


def remove_api_schema_prefixes(prim: Usd.Prim, prefixes: tuple[str, ...]) -> None:
    set_api_schemas(
        prim,
        [schema for schema in authored_api_schemas(prim) if not any(schema.startswith(prefix) for prefix in prefixes)],
    )


def remove_properties_by_prefix(prim: Usd.Prim, prefixes: tuple[str, ...]) -> None:
    for prop in list(prim.GetProperties()):
        if any(prop.GetName().startswith(prefix) for prefix in prefixes):
            prim.RemoveProperty(prop.GetName())


def set_relationship_targets(prim: Usd.Prim, rel_name: str, targets: Iterable[str]) -> None:
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


def set_vec3_attr(prim: Usd.Prim, attr_name: str, value: tuple[float, float, float]) -> None:
    attr = prim.GetAttribute(attr_name)
    if not attr:
        attr = prim.CreateAttribute(attr_name, Sdf.ValueTypeNames.Float3)
    attr.Set(Gf.Vec3f(*value))


def set_quatf_attr(prim: Usd.Prim, attr_name: str, value: tuple[float, float, float, float]) -> None:
    attr = prim.GetAttribute(attr_name)
    if not attr:
        attr = prim.CreateAttribute(attr_name, Sdf.ValueTypeNames.Quatf)
    attr.Set(Gf.Quatf(value[0], Gf.Vec3f(value[1], value[2], value[3])))


def set_joint_limits(prim: Usd.Prim, lower: float, upper: float) -> None:
    set_float_attr(prim, "physics:lowerLimit", lower)
    set_float_attr(prim, "physics:upperLimit", upper)


def set_common_xform_from_matrix(prim: Usd.Prim, matrix: Gf.Matrix4d) -> None:
    transform = Gf.Transform(matrix)
    quat = transform.GetRotation().GetQuat()
    imag = quat.GetImaginary()
    xformable = UsdGeom.Xformable(prim)
    xformable.ClearXformOpOrder()
    xformable.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(transform.GetTranslation()))
    xformable.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(quat.GetReal(), Gf.Vec3d(imag)))
    xformable.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(transform.GetScale()))


def strip_composition_arcs(prim: Usd.Prim) -> None:
    prim.SetInstanceable(False)
    prim.GetReferences().ClearReferences()
    prim.GetPayloads().ClearPayloads()


def strip_descendant_composition_arcs(prim: Usd.Prim) -> None:
    for descendant in Usd.PrimRange(prim):
        strip_composition_arcs(descendant)


def strip_material_bindings(prim: Usd.Prim) -> None:
    for prop in list(prim.GetProperties()):
        name = prop.GetName()
        if name.startswith("material:binding"):
            prim.RemoveProperty(name)
    remove_api_schemas(prim, ("MaterialBindingAPI",))


def strip_visual_collision_and_materials(visuals: Usd.Prim) -> None:
    if not visuals:
        return
    for prim in Usd.PrimRange(visuals):
        remove_api_schemas(prim, PHYSICS_COLLISION_SCHEMAS)
        remove_properties_by_prefix(prim, PHYSICS_ATTR_PREFIXES)
        strip_material_bindings(prim)


def apply_collision_api(prim: Usd.Prim, physics_material_path: str) -> None:
    ensure_api_schema(prim, "PhysicsCollisionAPI")
    if prim.GetTypeName() == "Mesh":
        ensure_api_schema(prim, "PhysicsMeshCollisionAPI")
        approximation = prim.GetAttribute("physics:approximation")
        if not approximation:
            approximation = prim.CreateAttribute("physics:approximation", Sdf.ValueTypeNames.Token)
        if approximation.Get() in (None, "", "none"):
            approximation.Set("convexHull")
    enabled = prim.GetAttribute("physics:collisionEnabled")
    if not enabled:
        enabled = prim.CreateAttribute("physics:collisionEnabled", Sdf.ValueTypeNames.Bool)
    enabled.Set(True)
    set_relationship_targets(prim, "material:binding:physics", [physics_material_path])


def enable_collider_tree(collisions: Usd.Prim, physics_material_path: str) -> None:
    if not collisions:
        return
    collisions.SetActive(True)
    for prim in Usd.PrimRange(collisions):
        strip_material_bindings(prim)
        if prim.GetTypeName() == "Mesh":
            apply_collision_api(prim, physics_material_path)


def create_physics_material(stage: Usd.Stage, root_path: str) -> str:
    material_path = f"{root_path}/PhysicsMaterial"
    material = UsdPhysics.MaterialAPI.Apply(stage.DefinePrim(material_path, "Material"))
    material.CreateStaticFrictionAttr().Set(1.0)
    material.CreateDynamicFrictionAttr().Set(1.0)
    material.CreateRestitutionAttr().Set(0.1)
    return material_path


def replace_link_geometry(
    stage: Usd.Stage,
    geometry_layer: Sdf.Layer,
    root_path: str,
    link_names: list[str],
    physics_material_path: str,
    notes: list[str],
) -> None:
    dst_layer = stage.GetRootLayer()
    for link_name in link_names:
        link_path = f"{root_path}/{link_name}"
        for child in ("visuals", "collisions"):
            child_path = f"{link_path}/{child}"
            if stage.GetPrimAtPath(child_path):
                stage.RemovePrim(child_path)

        copy_spec(geometry_layer, f"/visuals/{link_name}", dst_layer, f"{link_path}/visuals")
        copy_spec(geometry_layer, f"/colliders/{link_name}", dst_layer, f"{link_path}/collisions")

        visuals = stage.GetPrimAtPath(f"{link_path}/visuals")
        collisions = stage.GetPrimAtPath(f"{link_path}/collisions")
        strip_descendant_composition_arcs(visuals)
        strip_descendant_composition_arcs(collisions)
        strip_visual_collision_and_materials(visuals)
        enable_collider_tree(collisions, physics_material_path)
        notes.append(f"Replaced geometry for {link_name} from /visuals and /colliders libraries")


def remove_sensor_descendants(stage: Usd.Stage, root_path: str, link_names: list[str], notes: list[str]) -> None:
    for link_name in link_names:
        link = stage.GetPrimAtPath(f"{root_path}/{link_name}")
        if not link:
            continue
        for prim in reversed(list(Usd.PrimRange(link))):
            if prim == link:
                continue
            lowered = prim.GetName().lower()
            if (
                lowered.startswith("cam")
                or "camera" in lowered
                or prim.GetTypeName() in {"Camera", "IsaacContactSensor"}
                or "contact_sensor" in lowered
            ):
                notes.append(f"Removed sensor/camera descendant: {prim.GetPath()}")
                stage.RemovePrim(prim.GetPath())


def rebase_to_base_frame(stage: Usd.Stage, root_path: str, link_names: list[str], notes: list[str]) -> None:
    root = stage.GetPrimAtPath(root_path)
    base = stage.GetPrimAtPath(f"{root_path}/{BASE_LINK}")
    if not root or not base:
        raise RuntimeError(f"Missing root/base for rebase: {root_path}/{BASE_LINK}")

    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    base_world_inv = cache.GetLocalToWorldTransform(base).GetInverse()
    link_matrices: dict[str, Gf.Matrix4d] = {}
    for link_name in link_names:
        link = stage.GetPrimAtPath(f"{root_path}/{link_name}")
        if link:
            link_matrices[link_name] = cache.GetLocalToWorldTransform(link) * base_world_inv

    set_common_xform_from_matrix(root, Gf.Matrix4d(1.0))
    for link_name, matrix in link_matrices.items():
        set_common_xform_from_matrix(stage.GetPrimAtPath(f"{root_path}/{link_name}"), matrix)
    notes.append("Rebased gripper root to gripper_r_base_link")


def copy_required_articulation(robot_layer: Sdf.Layer, stage: Usd.Stage, notes: list[str]) -> None:
    dst_layer = stage.GetRootLayer()
    root = stage.DefinePrim(ROOT_PATH, "Xform")
    set_api_schemas(root, ["PhysicsArticulationRootAPI", "PhysxArticulationAPI"])

    for link in GRIPPER_LINKS:
        copy_spec(robot_layer, f"{ROOT_PATH}/{link}", dst_layer, f"{ROOT_PATH}/{link}")
        strip_descendant_composition_arcs(stage.GetPrimAtPath(f"{ROOT_PATH}/{link}"))
        notes.append(f"Copied articulation link metadata: {link}")

    stage.DefinePrim(f"{ROOT_PATH}/joints", "Xform")
    for joint in MAIN_JOINTS:
        copy_spec(robot_layer, f"{ROOT_PATH}/joints/{joint}", dst_layer, f"{ROOT_PATH}/joints/{joint}")
        strip_descendant_composition_arcs(stage.GetPrimAtPath(f"{ROOT_PATH}/joints/{joint}"))
        notes.append(f"Copied joint: {joint}")

    notes.append(
        "Dropped spherical closed-loop constraints and rewired link2 bodies as base mimic followers: "
        + ", ".join(REMOVED_CLOSED_LOOP_JOINTS)
    )


def create_root_joint(stage: Usd.Stage, root_path: str, notes: list[str]) -> None:
    joint = UsdPhysics.FixedJoint.Define(stage, f"{root_path}/root_joint").GetPrim()
    set_relationship_targets(joint, "physics:body0", [])
    set_relationship_targets(joint, "physics:body1", [f"{root_path}/{BASE_LINK}"])
    notes.append(f"Created root_joint body1 -> {root_path}/{BASE_LINK}")


def configure_active_drive(stage: Usd.Stage, joint_path: str, stiffness: float, damping: float, max_force: float) -> None:
    joint = stage.GetPrimAtPath(joint_path)
    if not joint:
        raise RuntimeError(f"Missing active drive joint: {joint_path}")
    remove_api_schema_prefixes(joint, ("PhysxMimicJointAPI:",))
    remove_properties_by_prefix(joint, ("physxMimicJoint:",))
    ensure_api_schema(joint, "PhysicsDriveAPI:angular")
    set_float_attr(joint, "drive:angular:physics:stiffness", stiffness)
    set_float_attr(joint, "drive:angular:physics:damping", damping)
    set_float_attr(joint, "drive:angular:physics:maxForce", max_force)
    set_float_attr(joint, "drive:angular:physics:targetPosition", 0.0)
    set_float_attr(joint, "drive:angular:physics:targetVelocity", 0.0)
    set_token_attr(joint, "drive:angular:physics:type", "force")


def configure_mimic_follower(stage: Usd.Stage, joint_path: str, reference_joint_path: str, gearing: float) -> None:
    joint = stage.GetPrimAtPath(joint_path)
    if not joint:
        raise RuntimeError(f"Missing mimic follower joint: {joint_path}")
    if not stage.GetPrimAtPath(reference_joint_path):
        raise RuntimeError(f"Missing mimic reference joint: {reference_joint_path}")
    remove_api_schema_prefixes(joint, ("PhysicsDriveAPI:", "PhysxMimicJointAPI:"))
    remove_properties_by_prefix(joint, ("drive:angular:", "drive:linear:", "physxMimicJoint:"))
    ensure_api_schema(joint, "PhysxMimicJointAPI:rotZ")
    set_relationship_targets(joint, "physxMimicJoint:rotZ:referenceJoint", [reference_joint_path])
    set_float_attr(joint, "physxMimicJoint:rotZ:gearing", gearing)
    set_float_attr(joint, "physxMimicJoint:rotZ:offset", 0.0)
    set_float_attr(joint, "physxMimicJoint:rotZ:naturalFrequency", 0.0)
    set_float_attr(joint, "physxMimicJoint:rotZ:dampingRatio", 0.0)


def configure_passive_joint(stage: Usd.Stage, joint_path: str) -> None:
    joint = stage.GetPrimAtPath(joint_path)
    if not joint:
        raise RuntimeError(f"Missing passive joint: {joint_path}")
    remove_api_schema_prefixes(joint, ("PhysicsDriveAPI:",))
    remove_properties_by_prefix(joint, ("drive:angular:", "drive:linear:"))


def configure_base_loop_follower(
    stage: Usd.Stage,
    joint_path: str,
    body1_path: str,
    local_pos0: tuple[float, float, float],
    local_pos1: tuple[float, float, float],
    local_rot0: tuple[float, float, float, float],
    local_rot1: tuple[float, float, float, float],
) -> None:
    joint = stage.GetPrimAtPath(joint_path)
    if not joint:
        raise RuntimeError(f"Missing loop follower joint: {joint_path}")
    set_relationship_targets(joint, "physics:body0", [f"{ROOT_PATH}/{BASE_LINK}"])
    set_relationship_targets(joint, "physics:body1", [body1_path])
    set_vec3_attr(joint, "physics:localPos0", local_pos0)
    set_vec3_attr(joint, "physics:localPos1", local_pos1)
    set_quatf_attr(joint, "physics:localRot0", local_rot0)
    set_quatf_attr(joint, "physics:localRot1", local_rot1)
    set_token_attr(joint, "physics:axis", "Z")


def normalize_to_standalone_omnipicker(stage: Usd.Stage, notes: list[str]) -> None:
    joints = f"{ROOT_PATH}/joints"
    active = f"{joints}/idx71_gripper_r_inner_joint1"
    configure_active_drive(stage, active, stiffness=10.0, damping=1.0, max_force=50.0)

    limit_overrides = {
        "idx71_gripper_r_inner_joint1": (0.0, 57.2957763671875),
        "idx72_gripper_r_inner_joint3": (-0.22918309271335602, 1.3750985860824585),
        "idx73_gripper_r_inner_joint4": (-4.583662033081055, 27.501972198486328),
        "idx79_gripper_r_inner_joint0": (-17.188732147216797, 103.13239288330078),
        "idx81_gripper_r_outer_joint1": (-11.459155082702637, 68.75492858886719),
        "idx82_gripper_r_outer_joint3": (-0.22918309271335602, 1.3750985860824585),
        "idx83_gripper_r_outer_joint4": (-4.583662033081055, 27.501972198486328),
        "idx89_gripper_r_outer_joint0": (-17.188732147216797, 103.13239288330078),
    }
    mimic_gearing = {
        "idx72_gripper_r_inner_joint3": -0.02,
        "idx73_gripper_r_inner_joint4": -0.4,
        "idx79_gripper_r_inner_joint0": -1.5,
        "idx81_gripper_r_outer_joint1": -1.0,
        "idx82_gripper_r_outer_joint3": -0.02,
        "idx83_gripper_r_outer_joint4": -0.4,
        "idx89_gripper_r_outer_joint0": -1.5,
    }
    for joint_name, (lower, upper) in limit_overrides.items():
        set_joint_limits(stage.GetPrimAtPath(f"{joints}/{joint_name}"), lower, upper)
    for joint_name, gearing in mimic_gearing.items():
        configure_mimic_follower(stage, f"{joints}/{joint_name}", active, gearing)
    configure_base_loop_follower(
        stage,
        f"{joints}/idx79_gripper_r_inner_joint0",
        f"{ROOT_PATH}/gripper_r_inner_link2",
        local_pos0=(0.0, -0.022, 0.073),
        local_pos1=(-0.032, 0.01, 0.0),
        local_rot0=(-0.004648268, -0.70709145, 0.004648268, -0.70709145),
        local_rot1=(1.0, 0.0, 0.0, 0.0),
    )
    configure_base_loop_follower(
        stage,
        f"{joints}/idx89_gripper_r_outer_joint0",
        f"{ROOT_PATH}/gripper_r_outer_link2",
        local_pos0=(0.0, 0.022, 0.073),
        local_pos1=(-0.032, -0.01, 0.0),
        local_rot0=(-0.004648268, 0.70709145, -0.004648268, -0.70709145),
        local_rot1=(0.0, 0.0, 1.0, 0.0),
    )
    notes.append("Normalized actuation to standalone OmniPicker single-drive mimic graph")


def preserve_robot_fix_actuation(stage: Usd.Stage, notes: list[str]) -> None:
    drive_joints = []
    for prim in stage.Traverse():
        if any(schema.startswith("PhysicsDriveAPI:") for schema in authored_api_schemas(prim)):
            drive_joints.append(str(prim.GetPath()))
    notes.append(f"Preserved robot_fix actuation; drive joints: {drive_joints}")


def relationship_target_exists(stage: Usd.Stage, target: Sdf.Path) -> bool:
    return bool(stage.GetPrimAtPath(target.GetPrimPath()))


def remove_or_repair_material_relationships(stage: Usd.Stage, root_path: str, notes: list[str]) -> None:
    for prim in stage.Traverse():
        for rel in prim.GetRelationships():
            old_targets = list(rel.GetTargets())
            if not old_targets:
                continue
            if all(relationship_target_exists(stage, target) for target in old_targets):
                continue
            if rel.GetName().startswith("material:binding"):
                rel.SetTargets([target for target in old_targets if relationship_target_exists(stage, target)])
                notes.append(f"Removed stale material target(s) on {prim.GetPath()}.{rel.GetName()}")
            elif rel.GetName().startswith("physics:body"):
                missing = [str(target) for target in old_targets if not relationship_target_exists(stage, target)]
                raise RuntimeError(f"Joint/body relationship has missing target on {prim.GetPath()}: {missing}")


def mesh_count(prim: Usd.Prim) -> int:
    if not prim:
        return 0
    return sum(1 for descendant in Usd.PrimRange(prim) if descendant.GetTypeName() == "Mesh")


def summarize(stage: Usd.Stage, notes: list[str], output_path: Path) -> dict[str, object]:
    rigid_bodies: list[str] = []
    collision_meshes: list[str] = []
    visual_collision_meshes: list[str] = []
    inactive_collision_containers: list[str] = []
    nontri_collision_meshes: list[str] = []
    cameras: list[str] = []
    invalid_relationship_targets: list[dict[str, str]] = []
    drive_joints: list[str] = []
    mimic_joints: list[str] = []
    link_collision_mesh_counts: dict[str, int] = {}
    external_arcs = 0

    for prim in stage.TraverseAll():
        path = str(prim.GetPath())
        if prim.GetTypeName() == "Camera" or "camera" in prim.GetName().lower() or prim.GetName().lower().startswith("cam"):
            cameras.append(path)

        refs = prim.GetMetadata("references")
        payload = prim.GetMetadata("payload")
        if refs and (refs.prependedItems or refs.addedItems or refs.explicitItems):
            external_arcs += len(refs.prependedItems) + len(refs.addedItems) + len(refs.explicitItems)
        if payload and (payload.prependedItems or payload.addedItems or payload.explicitItems):
            external_arcs += len(payload.prependedItems) + len(payload.addedItems) + len(payload.explicitItems)

        if "PhysicsRigidBodyAPI" in prim.GetAppliedSchemas():
            rigid_bodies.append(path)
            link_collision_mesh_counts[prim.GetName()] = sum(
                1
                for descendant in Usd.PrimRange(prim)
                if descendant.GetTypeName() == "Mesh" and "PhysicsCollisionAPI" in descendant.GetAppliedSchemas()
            )

        if path.endswith("/collisions") and not prim.IsActive():
            inactive_collision_containers.append(path)

        if prim.GetTypeName() == "Mesh" and "PhysicsCollisionAPI" in prim.GetAppliedSchemas():
            collision_meshes.append(path)
            if "/visuals/" in path:
                visual_collision_meshes.append(path)
            counts = prim.GetAttribute("faceVertexCounts").Get()
            if counts is not None and not all(count == 3 for count in counts):
                nontri_collision_meshes.append(path)

        if prim.GetTypeName().startswith("Physics") and "Joint" in prim.GetTypeName():
            if any(schema.startswith("PhysicsDriveAPI:") for schema in authored_api_schemas(prim)):
                drive_joints.append(path)
            if any(rel.GetName().startswith("physxMimicJoint:") for rel in prim.GetRelationships()):
                mimic_joints.append(path)

        for rel in prim.GetRelationships():
            for target in rel.GetTargets():
                if not relationship_target_exists(stage, target):
                    invalid_relationship_targets.append(
                        {"prim": path, "relationship": rel.GetName(), "target": str(target)}
                    )

    return {
        "ok": True,
        "root": ROOT_PATH,
        "defaultPrim": str(stage.GetDefaultPrim().GetPath()) if stage.GetDefaultPrim() else None,
        "metersPerUnit": UsdGeom.GetStageMetersPerUnit(stage),
        "upAxis": str(UsdGeom.GetStageUpAxis(stage)),
        "base_frame": BASE_LINK,
        "finger_colliders": list(FINGER_COLLIDERS),
        "link_names": GRIPPER_LINKS,
        "rigid_bodies": rigid_bodies,
        "collision_meshes": collision_meshes,
        "visual_collision_meshes": visual_collision_meshes,
        "inactive_collision_containers": inactive_collision_containers,
        "nontri_collision_meshes": nontri_collision_meshes,
        "cameras": cameras,
        "external_arcs": external_arcs,
        "invalid_relationship_targets": invalid_relationship_targets,
        "drive_joints": drive_joints,
        "mimic_joints": mimic_joints,
        "base_collision_mesh_count": link_collision_mesh_counts.get(BASE_LINK, 0),
        "finger_collision_mesh_counts": {name: link_collision_mesh_counts.get(name, 0) for name in FINGER_COLLIDERS},
        "link_collision_mesh_counts": link_collision_mesh_counts,
        "recommended_graspdatagen_config": {
            "robot_g2_omnipicker_gripper": {
                "gripper_file": str(output_path),
                "finger_colliders": list(FINGER_COLLIDERS),
                "base_frame": BASE_LINK,
                "bite": 0.016,
                "pinch_width_resolution": 8,
            }
        },
        "notes": notes,
    }


def validate_report(report: dict[str, object], expected_drive_count: int) -> None:
    errors: list[str] = []
    if report["defaultPrim"] != ROOT_PATH:
        errors.append(f"defaultPrim={report['defaultPrim']}")
    if report["metersPerUnit"] != 1.0:
        errors.append(f"metersPerUnit={report['metersPerUnit']}")
    if report["upAxis"] != "Z":
        errors.append(f"upAxis={report['upAxis']}")
    if report["external_arcs"] != 0:
        errors.append(f"external_arcs={report['external_arcs']}")
    if report["visual_collision_meshes"]:
        errors.append(f"visual_collision_meshes={report['visual_collision_meshes'][:5]}")
    if report["inactive_collision_containers"]:
        errors.append(f"inactive_collision_containers={report['inactive_collision_containers'][:5]}")
    if report["nontri_collision_meshes"]:
        errors.append(f"nontri_collision_meshes={report['nontri_collision_meshes'][:5]}")
    if report["cameras"]:
        errors.append(f"camera remnants={report['cameras'][:5]}")
    if report["invalid_relationship_targets"]:
        errors.append(f"invalid_relationship_targets={report['invalid_relationship_targets'][:5]}")
    if len(report["drive_joints"]) != expected_drive_count:
        errors.append(f"expected {expected_drive_count} drive joint(s), got {report['drive_joints']}")
    if report["base_collision_mesh_count"] == 0:
        errors.append("base frame has no collision mesh")
    missing_fingers = [
        name for name, count in report["finger_collision_mesh_counts"].items() if count == 0
    ]
    if missing_fingers:
        errors.append(f"finger colliders missing collision mesh: {missing_fingers}")
    if errors:
        raise RuntimeError("extraction validation failed: " + "; ".join(errors))


def extract(args: argparse.Namespace) -> dict[str, object]:
    robot_stage_path = Path(args.robot_stage)
    geometry_stage_path = Path(args.geometry_stage)
    output_path = Path(args.output)
    report_path = Path(args.report)

    if output_path.exists() and not args.force:
        raise FileExistsError(f"{output_path} already exists. Pass --force to overwrite it.")

    robot_stage = open_stage(robot_stage_path)
    geometry_stage = open_stage(geometry_stage_path)
    robot_layer = robot_stage.Flatten()
    geometry_layer = geometry_stage.Flatten()

    output_path.parent.mkdir(parents=True, exist_ok=True)
    if output_path.exists():
        output_path.unlink()
    stage = Usd.Stage.CreateNew(str(output_path))
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

    notes: list[str] = [
        f"robot_stage={robot_stage_path}",
        f"geometry_stage={geometry_stage_path}",
        "Collision geometry is copied only from /colliders, never from robot_fix visuals.",
    ]
    copy_required_articulation(robot_layer, stage, notes)
    remove_sensor_descendants(stage, ROOT_PATH, GRIPPER_LINKS, notes)
    rebase_to_base_frame(stage, ROOT_PATH, GRIPPER_LINKS, notes)
    physics_material_path = create_physics_material(stage, ROOT_PATH)
    replace_link_geometry(stage, geometry_layer, ROOT_PATH, GRIPPER_LINKS, physics_material_path, notes)
    create_root_joint(stage, ROOT_PATH, notes)

    if args.actuation_mode == "standalone":
        normalize_to_standalone_omnipicker(stage, notes)
        expected_drive_count = 1
    else:
        preserve_robot_fix_actuation(stage, notes)
        expected_drive_count = 1

    remove_or_repair_material_relationships(stage, ROOT_PATH, notes)
    stage.SetDefaultPrim(stage.GetPrimAtPath(ROOT_PATH))
    stage.GetRootLayer().Save()

    stage = open_stage(output_path)
    report = summarize(stage, notes, output_path)
    report["output"] = str(output_path)
    report["actuation_mode"] = args.actuation_mode
    validate_report(report, expected_drive_count)

    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-stage", default=str(DEFAULT_ROBOT_STAGE), help="Source robot_fix.usda stage.")
    parser.add_argument("--geometry-stage", default=str(DEFAULT_GEOMETRY_STAGE), help="Stage that owns /visuals and /colliders libraries.")
    parser.add_argument("--output", default=str(DEFAULT_OUTPUT), help="Output pure gripper USD.")
    parser.add_argument("--report", default=str(DEFAULT_REPORT), help="JSON validation report path.")
    parser.add_argument(
        "--actuation-mode",
        choices=("standalone", "preserve-fix"),
        default="standalone",
        help="standalone matches bots/omnipicker single-drive mimic graph; preserve-fix keeps robot_fix joint authoring.",
    )
    parser.add_argument("--force", action="store_true", help="Overwrite --output if it already exists.")
    args = parser.parse_args()

    report = extract(args)
    print(json.dumps(
        {
            "output": report["output"],
            "report": str(Path(args.report)),
            "actuation_mode": report["actuation_mode"],
            "drive_joints": report["drive_joints"],
            "finger_collision_mesh_counts": report["finger_collision_mesh_counts"],
            "visual_collision_meshes": len(report["visual_collision_meshes"]),
            "invalid_relationship_targets": len(report["invalid_relationship_targets"]),
        },
        indent=2,
        ensure_ascii=False,
    ))


if __name__ == "__main__":
    main()
