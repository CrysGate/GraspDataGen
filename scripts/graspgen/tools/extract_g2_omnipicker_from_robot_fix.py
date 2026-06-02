# /// script
# requires-python = ">=3.10"
# dependencies = ["usd-core"]
# ///
"""Extract right-hand grippers from robot USDs into clean GraspDataGen assets.

The robot/ assets are full robot stages. They should not be used directly as
GraspDataGen grippers because several of them keep inactive collision
containers, collision APIs on visual meshes, closed-loop constraints, sensors,
or multiple independent drives. This tool builds a pure gripper stage per
profile:

  * one articulation root at /genie;
  * only right-hand gripper links and required joints;
  * visuals under each link's /visuals;
  * collision meshes under each link's /collisions;
  * no cameras/sensors/external arcs;
  * a normalized single-drive actuation graph unless a profile explicitly says
    otherwise.

The default profile is g2_omnipicker, preserving the existing command behavior.
"""

from __future__ import annotations

import argparse
import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable, Mapping

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics


ROOT_PATH = "/genie"

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


@dataclass(frozen=True)
class DriveSpec:
    joint: str
    kind: str
    stiffness: float
    damping: float
    max_force: float


@dataclass(frozen=True)
class MimicSpec:
    joint: str
    axis: str
    gearing: float
    reference_joint: str | None = None
    offset: float = 0.0


@dataclass(frozen=True)
class LoopFollowerSpec:
    joint: str
    body1: str
    local_pos0: tuple[float, float, float]
    local_pos1: tuple[float, float, float]
    local_rot0: tuple[float, float, float, float]
    local_rot1: tuple[float, float, float, float]
    axis: str = "Z"


@dataclass(frozen=True)
class GripperProfile:
    name: str
    config_name: str
    robot_stage: Path
    geometry_stage: Path
    output: Path
    report: Path
    base_link: str
    finger_colliders: tuple[str, str]
    links: tuple[str, ...]
    joints: tuple[str, ...]
    active_drive: DriveSpec
    bite: float
    pinch_width_resolution: int = 8
    source_root: str = ROOT_PATH
    root_path: str = ROOT_PATH
    limit_overrides: Mapping[str, tuple[float, float]] = field(default_factory=dict)
    mimic_joints: tuple[MimicSpec, ...] = ()
    loop_followers: tuple[LoopFollowerSpec, ...] = ()
    dropped_joints: tuple[str, ...] = ()
    visual_collision_fallback_links: frozenset[str] = frozenset()


OMNIPICKER_LINKS = (
    "gripper_r_base_link",
    "gripper_r_inner_link1",
    "gripper_r_inner_link3",
    "gripper_r_inner_link4",
    "gripper_r_inner_link2",
    "gripper_r_outer_link1",
    "gripper_r_outer_link3",
    "gripper_r_outer_link4",
    "gripper_r_outer_link2",
)
OMNIPICKER_JOINTS = (
    "idx71_gripper_r_inner_joint1",
    "idx72_gripper_r_inner_joint3",
    "idx73_gripper_r_inner_joint4",
    "idx79_gripper_r_inner_joint0",
    "idx81_gripper_r_outer_joint1",
    "idx82_gripper_r_outer_joint3",
    "idx83_gripper_r_outer_joint4",
    "idx89_gripper_r_outer_joint0",
)
OMNIPICKER_LIMITS = {
    "idx71_gripper_r_inner_joint1": (0.0, 57.2957763671875),
    "idx72_gripper_r_inner_joint3": (-0.22918309271335602, 1.3750985860824585),
    "idx73_gripper_r_inner_joint4": (-4.583662033081055, 27.501972198486328),
    "idx79_gripper_r_inner_joint0": (-17.188732147216797, 103.13239288330078),
    "idx81_gripper_r_outer_joint1": (-11.459155082702637, 68.75492858886719),
    "idx82_gripper_r_outer_joint3": (-0.22918309271335602, 1.3750985860824585),
    "idx83_gripper_r_outer_joint4": (-4.583662033081055, 27.501972198486328),
    "idx89_gripper_r_outer_joint0": (-17.188732147216797, 103.13239288330078),
}
OMNIPICKER_MIMICS = (
    MimicSpec("idx72_gripper_r_inner_joint3", "rotZ", -0.02),
    MimicSpec("idx73_gripper_r_inner_joint4", "rotZ", -0.4),
    MimicSpec("idx79_gripper_r_inner_joint0", "rotZ", -1.5),
    MimicSpec("idx81_gripper_r_outer_joint1", "rotZ", -1.0),
    MimicSpec("idx82_gripper_r_outer_joint3", "rotZ", -0.02),
    MimicSpec("idx83_gripper_r_outer_joint4", "rotZ", -0.4),
    MimicSpec("idx89_gripper_r_outer_joint0", "rotZ", -1.5),
)
OMNIPICKER_LOOP_FOLLOWERS = (
    LoopFollowerSpec(
        "idx79_gripper_r_inner_joint0",
        "gripper_r_inner_link2",
        local_pos0=(0.0, -0.022, 0.073),
        local_pos1=(-0.032, 0.01, 0.0),
        local_rot0=(-0.004648268, -0.70709145, 0.004648268, -0.70709145),
        local_rot1=(1.0, 0.0, 0.0, 0.0),
    ),
    LoopFollowerSpec(
        "idx89_gripper_r_outer_joint0",
        "gripper_r_outer_link2",
        local_pos0=(0.0, 0.022, 0.073),
        local_pos1=(-0.032, -0.01, 0.0),
        local_rot0=(-0.004648268, 0.70709145, -0.004648268, -0.70709145),
        local_rot1=(0.0, 0.0, 1.0, 0.0),
    ),
)

G1_120S_LINKS = (
    "gripper_r_base_link",
    "gripper_r_inner_link1",
    "gripper_r_inner_link3",
    "gripper_r_inner_link4",
    "gripper_r_inner_link5",
    "gripper_r_inner_link2",
    "gripper_r_outer_link1",
    "gripper_r_outer_link3",
    "gripper_r_outer_link4",
    "gripper_r_outer_link5",
    "gripper_r_outer_link2",
)
G1_120S_JOINTS = (
    "idx71_gripper_r_inner_joint1",
    "idx72_gripper_r_inner_joint3",
    "idx73_gripper_r_inner_joint4",
    "idx74_gripper_r_inner_joint5",
    "idx79_gripper_r_inner_joint2",
    "idx81_gripper_r_outer_joint1",
    "idx82_gripper_r_outer_joint3",
    "idx83_gripper_r_outer_joint4",
    "idx84_gripper_r_outer_joint5",
    "idx89_gripper_r_outer_joint2",
)
G1_120S_LIMITS = {
    "idx71_gripper_r_inner_joint1": (0.0, 57.2957763671875),
    "idx72_gripper_r_inner_joint3": (-57.2957763671875, 0.0),
    "idx73_gripper_r_inner_joint4": (0.0, 0.0),
    "idx79_gripper_r_inner_joint2": (-57.2957763671875, 0.0),
    "idx81_gripper_r_outer_joint1": (0.0, 57.2957763671875),
    "idx82_gripper_r_outer_joint3": (0.0, 57.2957763671875),
    "idx83_gripper_r_outer_joint4": (0.0, 0.0),
    "idx89_gripper_r_outer_joint2": (-57.2957763671875, 0.0),
}
G1_120S_MIMICS = (
    MimicSpec("idx72_gripper_r_inner_joint3", "rotZ", -1.0),
    MimicSpec("idx79_gripper_r_inner_joint2", "rotZ", -1.0),
    MimicSpec("idx81_gripper_r_outer_joint1", "rotZ", 1.0),
    MimicSpec("idx82_gripper_r_outer_joint3", "rotZ", 1.0),
    MimicSpec("idx89_gripper_r_outer_joint2", "rotZ", -1.0),
)

G2_90D_LINKS = (
    "gripper_r_base_link",
    "gripper_r_left_inner_link",
    "gripper_r_left_outer_link",
    "gripper_r_left_support_link",
    "gripper_r_right_inner_link",
    "gripper_r_right_outer_link",
    "gripper_r_right_support_link",
)
G2_90D_JOINTS = (
    "idx71_gripper_r_inner_joint1",
    "idx72_gripper_r_outer_joint1",
    "idx73_gripper_r_left_support_joint",
    "idx74_gripper_r_inner_joint2",
    "idx75_gripper_r_outer_joint2",
    "idx76_gripper_r_right_support_joint",
)
G2_90D_LIMITS = {
    "idx71_gripper_r_inner_joint1": (-52.139156341552734, 0.0),
    "idx72_gripper_r_outer_joint1": (-52.139156341552734, 0.0),
    "idx73_gripper_r_left_support_joint": (0.0, 52.139156341552734),
    "idx74_gripper_r_inner_joint2": (0.0, 52.139156341552734),
    "idx75_gripper_r_outer_joint2": (0.0, 52.139156341552734),
    "idx76_gripper_r_right_support_joint": (0.0, 52.139156341552734),
}
G2_90D_MIMICS = (
    MimicSpec("idx72_gripper_r_outer_joint1", "rotZ", 1.0),
    MimicSpec("idx73_gripper_r_left_support_joint", "rotZ", -1.0),
    MimicSpec("idx74_gripper_r_inner_joint2", "rotZ", -1.0),
    MimicSpec("idx75_gripper_r_outer_joint2", "rotZ", -1.0),
    MimicSpec("idx76_gripper_r_right_support_joint", "rotZ", -1.0),
)

G2_PLACE_LINKS = (
    "gripper_r_base_link",
    "gripper_r_inner_link1",
    "gripper_r_outer_link1",
)
G2_PLACE_JOINTS = (
    "idx71_gripper_r_inner_joint1",
    "idx81_gripper_r_outer_joint1",
)
G2_PLACE_LIMITS = {
    "idx71_gripper_r_inner_joint1": (0.0, 0.02500000037252903),
    "idx81_gripper_r_outer_joint1": (0.0, 0.02500000037252903),
}
G2_PLACE_MIMICS = (
    MimicSpec("idx81_gripper_r_outer_joint1", "transY", 1.0),
)

PROFILES: dict[str, GripperProfile] = {
    "g2_omnipicker": GripperProfile(
        name="g2_omnipicker",
        config_name="robot_g2_omnipicker_gripper",
        robot_stage=Path("robot/G2_omnipicker/robot_fix.usda"),
        geometry_stage=Path("robot/G2_omnipicker/configuration/robot_physics.usd"),
        output=Path("bots/robot_g2_omnipicker_gripper.usd"),
        report=Path("debug_output/usd_inspect/g2_omnipicker_from_robot_fix.json"),
        base_link="gripper_r_base_link",
        finger_colliders=("gripper_r_inner_link4", "gripper_r_outer_link4"),
        links=OMNIPICKER_LINKS,
        joints=OMNIPICKER_JOINTS,
        active_drive=DriveSpec("idx71_gripper_r_inner_joint1", "angular", 10.0, 1.0, 50.0),
        limit_overrides=OMNIPICKER_LIMITS,
        mimic_joints=OMNIPICKER_MIMICS,
        loop_followers=OMNIPICKER_LOOP_FOLLOWERS,
        dropped_joints=(
            "idx93_gripper_r_outer_joint2",
            "idx94_gripper_r_inner_joint2",
        ),
        bite=0.016,
    ),
    "g1_omnipicker": GripperProfile(
        name="g1_omnipicker",
        config_name="robot_g1_omnipicker_gripper",
        robot_stage=Path("robot/G1_omnipicker/configuration/robot_physics.usd"),
        geometry_stage=Path("robot/G1_omnipicker/configuration/robot_physics.usd"),
        output=Path("bots/robot_g1_omnipicker_gripper.usd"),
        report=Path("debug_output/usd_inspect/g1_omnipicker_from_robot.json"),
        base_link="gripper_r_base_link",
        finger_colliders=("gripper_r_inner_link4", "gripper_r_outer_link4"),
        links=OMNIPICKER_LINKS,
        joints=OMNIPICKER_JOINTS,
        active_drive=DriveSpec("idx71_gripper_r_inner_joint1", "angular", 10.0, 1.0, 50.0),
        limit_overrides=OMNIPICKER_LIMITS,
        mimic_joints=OMNIPICKER_MIMICS,
        loop_followers=OMNIPICKER_LOOP_FOLLOWERS,
        dropped_joints=(
            "idx93_gripper_r_outer_joint2",
            "idx94_gripper_r_inner_joint2",
        ),
        bite=0.016,
    ),
    "g1_120s": GripperProfile(
        name="g1_120s",
        config_name="robot_g1_120s_gripper",
        robot_stage=Path("robot/G1_120s/configuration/G1_120s_physics.usd"),
        geometry_stage=Path("robot/G1_120s/configuration/G1_120s_physics.usd"),
        output=Path("bots/robot_g1_120s_gripper.usd"),
        report=Path("debug_output/usd_inspect/g1_120s_from_robot.json"),
        base_link="gripper_r_base_link",
        finger_colliders=("gripper_r_inner_link5", "gripper_r_outer_link5"),
        links=G1_120S_LINKS,
        joints=G1_120S_JOINTS,
        active_drive=DriveSpec("idx71_gripper_r_inner_joint1", "angular", 10.0, 1.0, 50.0),
        limit_overrides=G1_120S_LIMITS,
        mimic_joints=G1_120S_MIMICS,
        dropped_joints=(
            "idx93_gripper_r_outer_joint0",
            "idx94_gripper_r_inner_joint0",
        ),
        bite=0.018,
    ),
    "g2_90d": GripperProfile(
        name="g2_90d",
        config_name="robot_g2_90d_gripper",
        robot_stage=Path("robot/G2_90d/configuration/robot_physics.usd"),
        geometry_stage=Path("robot/G2_90d/configuration/robot_physics.usd"),
        output=Path("bots/robot_g2_90d_gripper.usd"),
        report=Path("debug_output/usd_inspect/g2_90d_from_robot.json"),
        base_link="gripper_r_base_link",
        finger_colliders=("gripper_r_left_support_link", "gripper_r_right_support_link"),
        links=G2_90D_LINKS,
        joints=G2_90D_JOINTS,
        active_drive=DriveSpec("idx71_gripper_r_inner_joint1", "angular", 10.0, 1.0, 50.0),
        limit_overrides=G2_90D_LIMITS,
        mimic_joints=G2_90D_MIMICS,
        visual_collision_fallback_links=frozenset(
            {
                "gripper_r_left_inner_link",
                "gripper_r_left_outer_link",
                "gripper_r_left_support_link",
                "gripper_r_right_inner_link",
                "gripper_r_right_outer_link",
                "gripper_r_right_support_link",
            }
        ),
        bite=0.018,
    ),
    "g2_place_workpiece": GripperProfile(
        name="g2_place_workpiece",
        config_name="robot_g2_place_workpiece_gripper",
        robot_stage=Path("robot/G2_place_workpiece/configuration/robot_physics.usd"),
        geometry_stage=Path("robot/G2_place_workpiece/configuration/robot_physics.usd"),
        output=Path("bots/robot_g2_place_workpiece_gripper.usd"),
        report=Path("debug_output/usd_inspect/g2_place_workpiece_from_robot.json"),
        base_link="gripper_r_base_link",
        finger_colliders=("gripper_r_inner_link1", "gripper_r_outer_link1"),
        links=G2_PLACE_LINKS,
        joints=G2_PLACE_JOINTS,
        active_drive=DriveSpec("idx71_gripper_r_inner_joint1", "linear", 100.0, 1.0, 500.0),
        limit_overrides=G2_PLACE_LIMITS,
        mimic_joints=G2_PLACE_MIMICS,
        bite=0.012,
    ),
}


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


def copy_spec_if_exists(src_layer: Sdf.Layer, src: str, dst_layer: Sdf.Layer, dst: str) -> bool:
    if not path_exists(src_layer, src):
        return False
    if not Sdf.CopySpec(src_layer, Sdf.Path(src), dst_layer, Sdf.Path(dst)):
        raise RuntimeError(f"Failed to copy USD spec {src} -> {dst}")
    return True


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
    if not prim:
        raise RuntimeError("Cannot set limits on missing joint")
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


def mesh_count(prim: Usd.Prim) -> int:
    if not prim:
        return 0
    return sum(1 for descendant in Usd.PrimRange(prim) if descendant.GetTypeName() == "Mesh")


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
    profile: GripperProfile,
    physics_material_path: str,
    notes: list[str],
) -> None:
    dst_layer = stage.GetRootLayer()
    for link_name in profile.links:
        link_path = f"{profile.root_path}/{link_name}"
        for child in ("visuals", "collisions"):
            child_path = f"{link_path}/{child}"
            if stage.GetPrimAtPath(child_path):
                stage.RemovePrim(child_path)

        copy_spec(geometry_layer, f"/visuals/{link_name}", dst_layer, f"{link_path}/visuals")
        copied_collider = copy_spec_if_exists(
            geometry_layer,
            f"/colliders/{link_name}",
            dst_layer,
            f"{link_path}/collisions",
        )

        visuals = stage.GetPrimAtPath(f"{link_path}/visuals")
        collisions = stage.GetPrimAtPath(f"{link_path}/collisions")
        if not copied_collider or mesh_count(collisions) == 0:
            if link_name not in profile.visual_collision_fallback_links:
                raise RuntimeError(
                    f"{profile.name}: /colliders/{link_name} has no mesh and visual fallback is not enabled"
                )
            if collisions:
                stage.RemovePrim(collisions.GetPath())
            copy_spec(geometry_layer, f"/visuals/{link_name}", dst_layer, f"{link_path}/collisions")
            collisions = stage.GetPrimAtPath(f"{link_path}/collisions")
            notes.append(f"Used /visuals/{link_name} as collision fallback because /colliders/{link_name} is empty")

        strip_descendant_composition_arcs(visuals)
        strip_descendant_composition_arcs(collisions)
        strip_visual_collision_and_materials(visuals)
        enable_collider_tree(collisions, physics_material_path)
        notes.append(f"Installed geometry for {link_name}")


def remove_sensor_descendants(stage: Usd.Stage, profile: GripperProfile, notes: list[str]) -> None:
    for link_name in profile.links:
        link = stage.GetPrimAtPath(f"{profile.root_path}/{link_name}")
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


def rebase_to_base_frame(stage: Usd.Stage, profile: GripperProfile, notes: list[str]) -> None:
    root = stage.GetPrimAtPath(profile.root_path)
    base = stage.GetPrimAtPath(f"{profile.root_path}/{profile.base_link}")
    if not root or not base:
        raise RuntimeError(f"Missing root/base for rebase: {profile.root_path}/{profile.base_link}")

    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    base_world_inv = cache.GetLocalToWorldTransform(base).GetInverse()
    link_matrices: dict[str, Gf.Matrix4d] = {}
    for link_name in profile.links:
        link = stage.GetPrimAtPath(f"{profile.root_path}/{link_name}")
        if link:
            link_matrices[link_name] = cache.GetLocalToWorldTransform(link) * base_world_inv

    set_common_xform_from_matrix(root, Gf.Matrix4d(1.0))
    for link_name, matrix in link_matrices.items():
        set_common_xform_from_matrix(stage.GetPrimAtPath(f"{profile.root_path}/{link_name}"), matrix)
    notes.append(f"Rebased gripper root to {profile.base_link}")


def copy_required_articulation(robot_layer: Sdf.Layer, stage: Usd.Stage, profile: GripperProfile, notes: list[str]) -> None:
    dst_layer = stage.GetRootLayer()
    root = stage.DefinePrim(profile.root_path, "Xform")
    set_api_schemas(root, ["PhysicsArticulationRootAPI", "PhysxArticulationAPI"])

    for link in profile.links:
        copy_spec(robot_layer, f"{profile.source_root}/{link}", dst_layer, f"{profile.root_path}/{link}")
        strip_descendant_composition_arcs(stage.GetPrimAtPath(f"{profile.root_path}/{link}"))
        notes.append(f"Copied link metadata: {link}")

    stage.DefinePrim(f"{profile.root_path}/joints", "Xform")
    for joint in profile.joints:
        copy_spec(robot_layer, f"{profile.source_root}/joints/{joint}", dst_layer, f"{profile.root_path}/joints/{joint}")
        strip_descendant_composition_arcs(stage.GetPrimAtPath(f"{profile.root_path}/joints/{joint}"))
        notes.append(f"Copied joint: {joint}")

    if profile.dropped_joints:
        notes.append("Dropped closed-loop/sensor/non-gripper joints: " + ", ".join(profile.dropped_joints))


def create_root_joint(stage: Usd.Stage, profile: GripperProfile, notes: list[str]) -> None:
    joint = UsdPhysics.FixedJoint.Define(stage, f"{profile.root_path}/root_joint").GetPrim()
    set_relationship_targets(joint, "physics:body0", [])
    set_relationship_targets(joint, "physics:body1", [f"{profile.root_path}/{profile.base_link}"])
    notes.append(f"Created root_joint body1 -> {profile.root_path}/{profile.base_link}")


def configure_active_drive(stage: Usd.Stage, profile: GripperProfile) -> None:
    spec = profile.active_drive
    joint_path = f"{profile.root_path}/joints/{spec.joint}"
    joint = stage.GetPrimAtPath(joint_path)
    if not joint:
        raise RuntimeError(f"Missing active drive joint: {joint_path}")
    remove_api_schema_prefixes(joint, ("PhysicsDriveAPI:", "PhysxMimicJointAPI:"))
    remove_properties_by_prefix(joint, ("drive:angular:", "drive:linear:", "physxMimicJoint:"))
    ensure_api_schema(joint, f"PhysicsDriveAPI:{spec.kind}")
    set_float_attr(joint, f"drive:{spec.kind}:physics:stiffness", spec.stiffness)
    set_float_attr(joint, f"drive:{spec.kind}:physics:damping", spec.damping)
    set_float_attr(joint, f"drive:{spec.kind}:physics:maxForce", spec.max_force)
    set_float_attr(joint, f"drive:{spec.kind}:physics:targetPosition", 0.0)
    set_float_attr(joint, f"drive:{spec.kind}:physics:targetVelocity", 0.0)
    set_token_attr(joint, f"drive:{spec.kind}:physics:type", "force")


def configure_mimic_follower(stage: Usd.Stage, profile: GripperProfile, spec: MimicSpec) -> None:
    joint_path = f"{profile.root_path}/joints/{spec.joint}"
    reference_name = spec.reference_joint or profile.active_drive.joint
    reference_joint_path = f"{profile.root_path}/joints/{reference_name}"
    joint = stage.GetPrimAtPath(joint_path)
    if not joint:
        raise RuntimeError(f"Missing mimic follower joint: {joint_path}")
    if not stage.GetPrimAtPath(reference_joint_path):
        raise RuntimeError(f"Missing mimic reference joint: {reference_joint_path}")
    remove_api_schema_prefixes(joint, ("PhysicsDriveAPI:", "PhysxMimicJointAPI:"))
    remove_properties_by_prefix(joint, ("drive:angular:", "drive:linear:", "physxMimicJoint:"))
    ensure_api_schema(joint, f"PhysxMimicJointAPI:{spec.axis}")
    prefix = f"physxMimicJoint:{spec.axis}"
    set_relationship_targets(joint, f"{prefix}:referenceJoint", [reference_joint_path])
    set_float_attr(joint, f"{prefix}:gearing", spec.gearing)
    set_float_attr(joint, f"{prefix}:offset", spec.offset)
    set_float_attr(joint, f"{prefix}:naturalFrequency", 0.0)
    set_float_attr(joint, f"{prefix}:dampingRatio", 0.0)


def configure_passive_joint(stage: Usd.Stage, joint_path: str) -> None:
    joint = stage.GetPrimAtPath(joint_path)
    if not joint:
        raise RuntimeError(f"Missing passive joint: {joint_path}")
    remove_api_schema_prefixes(joint, ("PhysicsDriveAPI:", "PhysxMimicJointAPI:"))
    remove_properties_by_prefix(joint, ("drive:angular:", "drive:linear:", "physxMimicJoint:"))


def configure_base_loop_follower(stage: Usd.Stage, profile: GripperProfile, spec: LoopFollowerSpec) -> None:
    joint_path = f"{profile.root_path}/joints/{spec.joint}"
    joint = stage.GetPrimAtPath(joint_path)
    if not joint:
        raise RuntimeError(f"Missing loop follower joint: {joint_path}")
    set_relationship_targets(joint, "physics:body0", [f"{profile.root_path}/{profile.base_link}"])
    set_relationship_targets(joint, "physics:body1", [f"{profile.root_path}/{spec.body1}"])
    set_vec3_attr(joint, "physics:localPos0", spec.local_pos0)
    set_vec3_attr(joint, "physics:localPos1", spec.local_pos1)
    set_quatf_attr(joint, "physics:localRot0", spec.local_rot0)
    set_quatf_attr(joint, "physics:localRot1", spec.local_rot1)
    set_token_attr(joint, "physics:axis", spec.axis)


def normalize_actuation(stage: Usd.Stage, profile: GripperProfile, notes: list[str]) -> None:
    joints_path = f"{profile.root_path}/joints"
    configure_active_drive(stage, profile)

    for joint_name, (lower, upper) in profile.limit_overrides.items():
        set_joint_limits(stage.GetPrimAtPath(f"{joints_path}/{joint_name}"), lower, upper)

    mimic_names = set()
    for mimic in profile.mimic_joints:
        configure_mimic_follower(stage, profile, mimic)
        mimic_names.add(mimic.joint)

    for follower in profile.loop_followers:
        configure_base_loop_follower(stage, profile, follower)

    for joint_name in profile.joints:
        if joint_name == profile.active_drive.joint or joint_name in mimic_names:
            continue
        joint = stage.GetPrimAtPath(f"{joints_path}/{joint_name}")
        if joint and joint.GetTypeName() != "PhysicsFixedJoint":
            configure_passive_joint(stage, f"{joints_path}/{joint_name}")

    notes.append(
        f"Normalized actuation: active={profile.active_drive.joint}, "
        f"mimics={[m.joint for m in profile.mimic_joints]}"
    )


def relationship_target_exists(stage: Usd.Stage, target: Sdf.Path) -> bool:
    return bool(stage.GetPrimAtPath(target.GetPrimPath()))


def remove_or_repair_material_relationships(stage: Usd.Stage, notes: list[str]) -> None:
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


def summarize(stage: Usd.Stage, profile: GripperProfile, notes: list[str], output_path: Path) -> dict[str, object]:
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
    fallback_collision_links: list[str] = []
    external_arcs = 0

    for link_name in profile.links:
        collisions = stage.GetPrimAtPath(f"{profile.root_path}/{link_name}/collisions")
        if collisions and mesh_count(collisions) > 0 and link_name in profile.visual_collision_fallback_links:
            fallback_collision_links.append(link_name)

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
        "profile": profile.name,
        "config_name": profile.config_name,
        "root": profile.root_path,
        "defaultPrim": str(stage.GetDefaultPrim().GetPath()) if stage.GetDefaultPrim() else None,
        "metersPerUnit": UsdGeom.GetStageMetersPerUnit(stage),
        "upAxis": str(UsdGeom.GetStageUpAxis(stage)),
        "base_frame": profile.base_link,
        "finger_colliders": list(profile.finger_colliders),
        "link_names": list(profile.links),
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
        "base_collision_mesh_count": link_collision_mesh_counts.get(profile.base_link, 0),
        "finger_collision_mesh_counts": {
            name: link_collision_mesh_counts.get(name, 0) for name in profile.finger_colliders
        },
        "link_collision_mesh_counts": link_collision_mesh_counts,
        "fallback_collision_links": fallback_collision_links,
        "recommended_graspdatagen_config": {
            profile.config_name: {
                "gripper_file": str(output_path),
                "finger_colliders": list(profile.finger_colliders),
                "base_frame": profile.base_link,
                "bite": profile.bite,
                "pinch_width_resolution": profile.pinch_width_resolution,
            }
        },
        "notes": notes,
    }


def validate_report(report: dict[str, object], profile: GripperProfile) -> None:
    errors: list[str] = []
    if report["defaultPrim"] != profile.root_path:
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
    if len(report["drive_joints"]) != 1:
        errors.append(f"expected one drive joint, got {report['drive_joints']}")
    if report["base_collision_mesh_count"] == 0:
        errors.append("base frame has no collision mesh")
    missing_fingers = [name for name, count in report["finger_collision_mesh_counts"].items() if count == 0]
    if missing_fingers:
        errors.append(f"finger colliders missing collision mesh: {missing_fingers}")
    if errors:
        raise RuntimeError(f"{profile.name}: extraction validation failed: " + "; ".join(errors))


def with_overrides(profile: GripperProfile, args: argparse.Namespace) -> GripperProfile:
    if args.robot_stage:
        profile = dataclass_replace(profile, robot_stage=Path(args.robot_stage))
    if args.geometry_stage:
        profile = dataclass_replace(profile, geometry_stage=Path(args.geometry_stage))
    if args.output:
        profile = dataclass_replace(profile, output=Path(args.output))
    if args.report:
        profile = dataclass_replace(profile, report=Path(args.report))
    return profile


def dataclass_replace(profile: GripperProfile, **changes: object) -> GripperProfile:
    data = {
        "name": profile.name,
        "config_name": profile.config_name,
        "robot_stage": profile.robot_stage,
        "geometry_stage": profile.geometry_stage,
        "output": profile.output,
        "report": profile.report,
        "base_link": profile.base_link,
        "finger_colliders": profile.finger_colliders,
        "links": profile.links,
        "joints": profile.joints,
        "active_drive": profile.active_drive,
        "bite": profile.bite,
        "pinch_width_resolution": profile.pinch_width_resolution,
        "source_root": profile.source_root,
        "root_path": profile.root_path,
        "limit_overrides": profile.limit_overrides,
        "mimic_joints": profile.mimic_joints,
        "loop_followers": profile.loop_followers,
        "dropped_joints": profile.dropped_joints,
        "visual_collision_fallback_links": profile.visual_collision_fallback_links,
    }
    data.update(changes)
    return GripperProfile(**data)


def extract_profile(profile: GripperProfile, force: bool) -> dict[str, object]:
    if profile.output.exists() and not force:
        raise FileExistsError(f"{profile.output} already exists. Pass --force to overwrite it.")
    if not profile.robot_stage.exists():
        raise FileNotFoundError(profile.robot_stage)
    if not profile.geometry_stage.exists():
        raise FileNotFoundError(profile.geometry_stage)

    robot_stage = open_stage(profile.robot_stage)
    geometry_stage = open_stage(profile.geometry_stage)
    robot_layer = robot_stage.Flatten()
    geometry_layer = geometry_stage.Flatten()

    profile.output.parent.mkdir(parents=True, exist_ok=True)
    if profile.output.exists():
        profile.output.unlink()
    stage = Usd.Stage.CreateNew(str(profile.output))
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

    notes: list[str] = [
        f"profile={profile.name}",
        f"robot_stage={profile.robot_stage}",
        f"geometry_stage={profile.geometry_stage}",
        "Collision geometry is installed under /collisions and stripped from /visuals.",
    ]
    copy_required_articulation(robot_layer, stage, profile, notes)
    remove_sensor_descendants(stage, profile, notes)
    rebase_to_base_frame(stage, profile, notes)
    physics_material_path = create_physics_material(stage, profile.root_path)
    replace_link_geometry(stage, geometry_layer, profile, physics_material_path, notes)
    create_root_joint(stage, profile, notes)
    normalize_actuation(stage, profile, notes)
    remove_or_repair_material_relationships(stage, notes)
    stage.SetDefaultPrim(stage.GetPrimAtPath(profile.root_path))
    stage.GetRootLayer().Save()

    stage = open_stage(profile.output)
    report = summarize(stage, profile, notes, profile.output)
    report["output"] = str(profile.output)
    report["report"] = str(profile.report)
    validate_report(report, profile)

    profile.report.parent.mkdir(parents=True, exist_ok=True)
    profile.report.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    return report


def selected_profiles(args: argparse.Namespace) -> list[GripperProfile]:
    if args.all:
        names = list(PROFILES)
    else:
        names = args.profile or ["g2_omnipicker"]

    unknown = [name for name in names if name not in PROFILES]
    if unknown:
        raise ValueError(f"Unknown profile(s): {unknown}. Available: {sorted(PROFILES)}")

    if len(names) > 1 and (args.robot_stage or args.geometry_stage or args.output or args.report):
        raise ValueError("--robot-stage/--geometry-stage/--output/--report overrides require exactly one profile")

    profiles = [PROFILES[name] for name in names]
    if len(profiles) == 1:
        profiles = [with_overrides(profiles[0], args)]
    return profiles


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--profile",
        action="append",
        choices=sorted(PROFILES),
        help="Profile to extract. Can be repeated. Defaults to g2_omnipicker.",
    )
    parser.add_argument("--all", action="store_true", help="Extract all robot gripper profiles.")
    parser.add_argument("--robot-stage", help="Override source articulation stage for a single profile.")
    parser.add_argument("--geometry-stage", help="Override stage that owns /visuals and /colliders for a single profile.")
    parser.add_argument("--output", help="Override output USD path for a single profile.")
    parser.add_argument("--report", help="Override JSON report path for a single profile.")
    parser.add_argument("--force", action="store_true", help="Overwrite output USD if it already exists.")
    args = parser.parse_args()

    reports = [extract_profile(profile, args.force) for profile in selected_profiles(args)]
    print(
        json.dumps(
            [
                {
                    "profile": report["profile"],
                    "output": report["output"],
                    "report": report["report"],
                    "drive_joints": report["drive_joints"],
                    "finger_collision_mesh_counts": report["finger_collision_mesh_counts"],
                    "fallback_collision_links": report["fallback_collision_links"],
                    "visual_collision_meshes": len(report["visual_collision_meshes"]),
                    "invalid_relationship_targets": len(report["invalid_relationship_targets"]),
                }
                for report in reports
            ],
            indent=2,
            ensure_ascii=False,
        )
    )


if __name__ == "__main__":
    main()
