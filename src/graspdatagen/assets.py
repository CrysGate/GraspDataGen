"""USD composition, dependency fingerprints and immutable prepared object caches."""

from __future__ import annotations

import hashlib
import json
import tempfile
from dataclasses import asdict
from importlib.metadata import version
from pathlib import Path
from typing import TYPE_CHECKING, Any

import numpy as np
import trimesh

from graspdatagen.config import MaterialConfig, ObjectConfig, digest
from graspdatagen.geometry import (
    mesh_in_frame,
    relative_transform,
    set_transform,
)

if TYPE_CHECKING:
    from pxr import Usd

    from graspdatagen.runtime import PhysxRuntime


def file_hash(path: Path) -> str:
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def write_json(path: Path, data: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(data, indent=2, allow_nan=False) + "\n")
    temporary.replace(path)


def tool_signature() -> dict[str, Any]:
    import ast

    from pxr import Usd

    # Preparation identities depend on preparation code. Adding a replay or
    # validator must not recook an unchanged high-resolution asset.
    boundaries = {
        "assets": set(),
        "grippers": set(),
        "inspection": set(),
        "runtime": {"RuntimeConfig", "PhysxRuntime"},
        "config": {
            "read_mapping",
            "fields",
            "positive",
            "vector",
            "digest",
            "MaterialConfig",
            "ObjectConfig",
            "load_objects",
            "CalibrationConfig",
            "GripperConfig",
            "load_gripper",
        },
        "geometry": {
            "transform_points",
            "relative_transform",
            "set_transform",
            "mesh_in_frame",
            "author_mesh",
        },
    }
    implementation = {}
    for module, names in boundaries.items():
        tree = ast.parse(Path(__file__).with_name(module + ".py").read_text())
        if names:
            tree.body = [
                node
                for node in tree.body
                if isinstance(node, (ast.Import, ast.ImportFrom))
                or isinstance(node, (ast.FunctionDef, ast.ClassDef))
                and node.name in names
            ]
        implementation[module] = digest(ast.dump(tree, include_attributes=False))
    return {
        "format": "graspdatagen.prepared.v1",
        "packages": {n: version(n) for n in ("isaacsim", "numpy", "trimesh", "warp-lang")},
        "usd": list(Usd.GetVersion()),
        "implementation": implementation,
    }


def open_stage(source: Path, root_path: str) -> Usd.Stage:
    """An empty manifest path selects the unique default prim, never a first mesh."""
    from pxr import Sdf, Usd

    if source.suffix.lower() not in (".usd", ".usda", ".usdc", ".usdz"):
        raise ValueError(f"Only native USD/USDZ inputs are supported: {source}")
    stage = Usd.Stage.Open(str(source.resolve()))
    if stage is None:
        raise ValueError(f"Cannot open {source}")
    root = stage.GetPrimAtPath(root_path) if root_path else stage.GetDefaultPrim()
    if not root or not root.IsActive() or root.GetPath() == Sdf.Path.absoluteRootPath:
        raise ValueError(f"Missing explicit root or unique default prim: {source}")
    stage.SetEditTarget(stage.GetSessionLayer())
    # Flatten preserves instance prototypes unless they are first expanded.
    # Expand in the anonymous session layer so selected-body copies are complete.
    while instances := [p for p in stage.Traverse() if p.IsInstance()]:
        for instance in instances:
            instance.SetInstanceable(False)
    # A selected root becomes the default only in this anonymous composition.
    layer = stage.Flatten()
    composed = Usd.Stage.Open(layer)
    composed.SetDefaultPrim(composed.GetPrimAtPath(root.GetPath()))
    return composed


def source_fingerprint(source: Path) -> dict[str, Any]:
    """Hash composed layers and asset attributes through the USD resolver.

    Kit MDL library names are not file dependencies of the generated assets:
    gripper displays use vertex colours, and object replay uses UsdPreviewSurface.
    Their authored names are recorded; unresolved non-MDL dependencies fail.
    USDZ layers and textures are read with Ar, not extracted by path manipulation.
    """
    from pxr import Ar, Sdf, Usd

    stage = Usd.Stage.Open(str(source.resolve()))
    if stage is None:
        raise ValueError(f"Cannot compose {source}")
    resolver = Ar.GetResolver()
    dependencies: dict[str, str] = {str(source): file_hash(source)}
    omitted_shader_libraries: set[str] = set()
    for layer in stage.GetUsedLayers():
        if layer.anonymous:
            continue
        path = resolver.Resolve(layer.realPath or layer.identifier)
        if not path:
            raise ValueError(f"Unresolved USD layer {layer.identifier}")
        dependencies[str(path)] = hashlib.sha256(resolver.OpenAsset(path).GetBuffer()).hexdigest()
    for prim in Usd.PrimRange(stage.GetPseudoRoot(), Usd.TraverseInstanceProxies()):
        for attr in prim.GetAttributes():
            if attr.GetNumTimeSamples():
                raise ValueError(f"Animated assets are unsupported: {attr.GetPath()}")
            if attr.GetTypeName() not in (Sdf.ValueTypeNames.Asset, Sdf.ValueTypeNames.AssetArray):
                continue
            value = attr.Get()
            values = value if attr.GetTypeName() == Sdf.ValueTypeNames.AssetArray else [value]
            for asset in values:
                if not asset or not asset.path:
                    continue
                if asset.path.endswith(".mdl"):
                    omitted_shader_libraries.add(asset.path)
                    continue
                resolved = resolver.Resolve(asset.resolvedPath or asset.path)
                if not resolved:
                    raise ValueError(f"Unresolved dependency {asset.path} at {attr.GetPath()}")
                dependencies[str(resolved)] = hashlib.sha256(
                    resolver.OpenAsset(resolved).GetBuffer()
                ).hexdigest()
    return {
        "files": dependencies,
        "omitted_display_shader_libraries": sorted(omitted_shader_libraries),
    }


def cache_identity(fingerprint: dict[str, Any], config: dict[str, Any]) -> str:
    return digest(
        {
            "contents": sorted(set(fingerprint["files"].values())),
            "config": config,
            "tools": tool_signature(),
        }
    )


def check_cache(directory: Path) -> dict[str, Any]:
    """Existing but incomplete or modified caches are errors, never cache misses."""
    manifest = json.loads((directory / "manifest.json").read_text())
    for relative, expected in manifest["artifacts"].items():
        path = directory / relative
        if not path.is_file() or file_hash(path) != expected:
            raise ValueError(f"Corrupt prepared cache: {path}")
    if not manifest["complete"]:
        raise ValueError(f"Incomplete prepared cache: {directory}")
    return manifest


def seal_cache(directory: Path, report: dict[str, Any]) -> None:
    report["tools"] = tool_signature()
    report["complete"] = True
    report["artifacts"] = {
        str(path.relative_to(directory)): file_hash(path)
        for path in sorted(directory.rglob("*"))
        if path.is_file() and path.name != "manifest.json"
    }
    write_json(directory / "manifest.json", report)


def bind_material(stage: Usd.Stage, prims: list[Usd.Prim], config: MaterialConfig) -> str:
    from pxr import UsdPhysics, UsdShade

    path = str(stage.GetDefaultPrim().GetPath()) + "/PreparedPhysicsMaterial"
    material = UsdShade.Material.Define(stage, path)
    physics = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    physics.CreateStaticFrictionAttr(config.static_friction)
    physics.CreateDynamicFrictionAttr(config.dynamic_friction)
    physics.CreateRestitutionAttr(config.restitution)
    for prim in prims:
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(
            material,
            bindingStrength=UsdShade.Tokens.strongerThanDescendants,
            materialPurpose="physics",
        )
    return path


def localize_assets(stage: Usd.Stage, directory: Path) -> None:
    """Copy texture bytes from resolver assets, including USDZ package members."""
    from pxr import Ar, Sdf, UsdShade

    resolver = Ar.GetResolver()
    for prim in list(stage.Traverse()):
        if prim.IsA(UsdShade.Shader):
            for attr in list(prim.GetAttributes()):
                if ":mdl:" in attr.GetName():
                    prim.RemoveProperty(attr.GetName())
        for attr in prim.GetAttributes():
            if attr.GetTypeName() != Sdf.ValueTypeNames.Asset:
                continue
            asset = attr.Get()
            if not asset or not asset.path:
                continue
            resolved = resolver.Resolve(asset.resolvedPath or asset.path)
            if not resolved:
                raise ValueError(f"Unresolved cached asset: {attr.GetPath()} = {asset.path}")
            contents = resolver.OpenAsset(resolved).GetBuffer()
            extension = Path(Ar.SplitPackageRelativePathInner(str(resolved))[1]).suffix
            if not extension:
                extension = Path(str(resolved)).suffix
            filename = hashlib.sha256(contents).hexdigest() + extension
            target = directory / "dependencies" / filename
            target.parent.mkdir(exist_ok=True)
            target.write_bytes(contents)
            attr.Set(Sdf.AssetPath("dependencies/" + filename))


def retain_object_dependencies(stage: Usd.Stage) -> None:
    """Keep the selected object and referenced prim dependencies, not saved viewports."""
    from pxr import Usd

    roots = {stage.GetDefaultPrim().GetPath()}
    pending = list(roots)
    while pending:
        for prim in Usd.PrimRange(stage.GetPrimAtPath(pending.pop())):
            targets = [target for rel in prim.GetRelationships() for target in rel.GetTargets()]
            targets += [target for attr in prim.GetAttributes() for target in attr.GetConnections()]
            for target in targets:
                top = target.GetPrimPath().GetPrefixes()[0]
                if top not in roots:
                    if not stage.GetPrimAtPath(top):
                        raise ValueError(f"Missing selected-object dependency: {target}")
                    roots.add(top)
                    pending.append(top)
    for prim in list(stage.GetPseudoRoot().GetChildren()):
        if prim.GetPath() not in roots:
            stage.RemovePrim(prim.GetPath())


def check_references(source: Path) -> dict[str, int]:
    from pxr import Sdf, Usd, UsdUtils

    layers, assets, unresolved = UsdUtils.ComputeAllDependencies(
        Sdf.AssetPath(str(source.resolve()))
    )
    if unresolved:
        raise ValueError(f"Unresolved cached dependencies: {unresolved}")
    stage = Usd.Stage.Open(str(source.resolve()))
    if stage.GetCompositionErrors():
        raise ValueError(f"Cached USD composition errors: {stage.GetCompositionErrors()}")
    for prim in stage.Traverse():
        for rel in prim.GetRelationships():
            for target in rel.GetTargets():
                if not stage.GetObjectAtPath(target):
                    raise ValueError(f"Dangling relationship: {rel.GetPath()} -> {target}")
        for attr in prim.GetAttributes():
            for target in attr.GetConnections():
                if not stage.GetObjectAtPath(target):
                    raise ValueError(f"Dangling connection: {attr.GetPath()} -> {target}")
    return {"layers": len(layers), "assets": len(assets), "unresolved": 0}


def inspect_object(config: ObjectConfig) -> dict[str, Any]:
    from pxr import Usd, UsdGeom, UsdPhysics, UsdShade

    stage = open_stage(config.source, config.root_prim)
    root = stage.GetDefaultPrim()
    prims = list(Usd.PrimRange(root, Usd.TraverseInstanceProxies()))
    bodies = [p for p in prims if p.HasAPI(UsdPhysics.RigidBodyAPI)]
    if len(bodies) != 1:
        raise ValueError(f"Object must have exactly one rigid body: {config.source}")
    body = bodies[0]
    rigid = UsdPhysics.RigidBodyAPI(body)
    if not rigid.GetRigidBodyEnabledAttr().Get() or rigid.GetKinematicEnabledAttr().Get():
        raise ValueError("Object must be an enabled dynamic rigid body")
    for prim in prims:
        if prim.IsA(UsdPhysics.Joint) or any(
            word in schema.lower()
            for schema in prim.GetAppliedSchemas()
            for word in ("deformable", "particle", "cloth")
        ):
            raise ValueError(f"Articulated/deformable object unsupported: {prim.GetPath()}")
    colliders = [
        p
        for p in prims
        if p.HasAPI(UsdPhysics.CollisionAPI)
        and UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get()
    ]
    if not colliders:
        raise ValueError("Object has no enabled collision geometry")
    for prim in colliders:
        if not prim.GetPath().HasPrefix(body.GetPath()) or not prim.IsA(UsdGeom.Mesh):
            raise ValueError(f"Unsupported collider ownership or shape: {prim.GetPath()}")
    surfaces = [
        p
        for p in prims
        if p.IsA(UsdGeom.Mesh)
        and not p.HasAPI(UsdPhysics.CollisionAPI)
        and UsdGeom.Imageable(p).ComputeVisibility() != "invisible"
    ]
    if not surfaces:
        surfaces = colliders
    unit = float(UsdGeom.GetStageMetersPerUnit(stage))
    positive_units = np.isfinite(unit) and unit > 0
    if not positive_units:
        raise ValueError("Invalid stage units")
    root_rotation = np.asarray(UsdGeom.XformCache().GetLocalToWorldTransform(root))[:3, :3]
    if (
        not np.allclose(root_rotation @ root_rotation.T, np.eye(3), atol=1e-6)
        or np.linalg.det(root_rotation) < 0
    ):
        raise ValueError("A scaled/reflected object root requires an explicit frame adapter")
    meshes = [mesh_in_frame(p, root) for p in surfaces]
    surface = trimesh.util.concatenate(meshes)
    surface.apply_scale(unit)
    mass_api = UsdPhysics.MassAPI(body)
    metadata = json.loads(config.metadata.read_text())["physics"]
    if not np.isclose(config.mass_kg, metadata["mass"]):
        raise ValueError("P1 manifest mass must explicitly select the supplied metadata mass")
    materials = []
    for collider in colliders:
        material, _ = UsdShade.MaterialBindingAPI(collider).ComputeBoundMaterial("physics")
        materials.append(
            {
                "collider": str(collider.GetPath()),
                "material": str(material.GetPath()) if material else "",
                "has_physics_material": bool(
                    material and material.GetPrim().HasAPI(UsdPhysics.MaterialAPI)
                ),
                "approximation": str(
                    UsdPhysics.MeshCollisionAPI(collider).GetApproximationAttr().Get()
                ),
            }
        )
    return {
        "name": config.name,
        "source": str(config.source),
        "root_prim": str(root.GetPath()),
        "body_prim": str(body.GetPath()),
        "meters_per_unit": unit,
        "up_axis": str(UsdGeom.GetStageUpAxis(stage)),
        "bounds_object_m": surface.bounds.tolist(),
        "size_m": surface.extents.tolist(),
        "source_vertices": len(surface.vertices),
        "source_faces": len(surface.faces),
        "surface_prims": [str(p.GetPath()) for p in surfaces],
        "colliders": materials,
        "usd_mass_kg": float(mass_api.GetMassAttr().Get()),
        "metadata_physics": metadata,
        "resolved_mass_kg": config.mass_kg,
        "resolved_material": asdict(config.material),
        "automatic_mass_properties": {
            name: {"authored": attr.HasAuthoredValueOpinion(), "usd_value": str(attr.Get())}
            for name, attr in (
                ("center_of_mass", mass_api.GetCenterOfMassAttr()),
                ("inertia", mass_api.GetDiagonalInertiaAttr()),
                ("principal_axes", mass_api.GetPrincipalAxesAttr()),
            )
        },
    }


def prepare_object(runtime: PhysxRuntime, config: ObjectConfig, cache_root: Path) -> Path:
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

    fingerprint = source_fingerprint(config.source)
    fingerprint["files"][str(config.metadata)] = file_hash(config.metadata)
    key = cache_identity(fingerprint, config.snapshot)
    destination = cache_root / "objects" / key
    if destination.exists():
        check_cache(destination)
        return destination
    inspection = inspect_object(config)
    print(f"P1: loading source collisions for {config.name}", flush=True)
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=".preparing-", dir=destination.parent) as temporary:
        directory = Path(temporary)
        stage = open_stage(config.source, config.root_prim)
        root = stage.GetDefaultPrim()
        unit = inspection["meters_per_unit"]
        # Object axes remain those of the source root, regardless of stage up-axis.
        surface = trimesh.util.concatenate(
            [mesh_in_frame(stage.GetPrimAtPath(p), root) for p in inspection["surface_prims"]]
        )
        surface.apply_scale(unit)
        surface.merge_vertices()
        surface.fix_normals(multibody=True)
        # PhysX resolves the asset's own collision settings. Preserve that geometry
        # for sampling and simulation instead of decomposing the visual surface.
        source_path = directory / "source.usdc"
        stage.GetRootLayer().Export(str(source_path))
        runtime.new_scene()
        runtime.reference(source_path, "/World/source")
        hulls = []
        for item in inspection["colliders"]:
            path = item["collider"]
            transform = relative_transform(stage.GetPrimAtPath(path), root)
            cooked = runtime.cook_convexes("/World/source" + path[len(inspection["root_prim"]) :])
            for hull in cooked:
                hull.apply_transform(transform)
                hull.apply_scale(unit)
            hulls.extend(cooked)
        runtime.new_scene()
        source_path.unlink()
        hulls.sort(key=lambda mesh: tuple(mesh.bounds.ravel()) + (len(mesh.vertices),))
        prepared = Usd.Stage.CreateNew(str(directory / "object.usdc"))
        body = UsdGeom.Xform.Define(prepared, "/root").GetPrim()
        prepared.SetDefaultPrim(body)
        UsdGeom.SetStageMetersPerUnit(prepared, 1.0)
        UsdGeom.SetStageUpAxis(prepared, UsdGeom.Tokens.z)
        UsdPhysics.RigidBodyAPI.Apply(body)
        mass = UsdPhysics.MassAPI.Apply(body)
        mass.CreateMassAttr(config.mass_kg)
        source_body = stage.GetPrimAtPath(inspection["body_prim"])
        source_mass = UsdPhysics.MassAPI(source_body)
        com = np.asarray(source_mass.GetCenterOfMassAttr().Get(), dtype=np.float64)
        inertia = np.asarray(source_mass.GetDiagonalInertiaAttr().Get(), dtype=np.float64)
        if np.isfinite(com).all():
            from graspdatagen.geometry import transform_points

            resolved_com = (
                transform_points(com[None, :], relative_transform(source_body, root))[0] * unit
            )
            mass.CreateCenterOfMassAttr(Gf.Vec3f(*resolved_com))
        elif not np.isneginf(com).all():
            raise ValueError("Invalid authored centre of mass")
        if (inertia != 0).any():
            if not np.isfinite(inertia).all() or (inertia <= 0).any():
                raise ValueError("Invalid authored inertia")
            # P1 does not silently reinterpret inertia under a changed mass or body frame.
            if config.mass_kg != inspection["usd_mass_kg"] or source_body != root or unit != 1:
                raise ValueError("Authored inertia requires a matching mass and metre root frame")
            mass.CreateDiagonalInertiaAttr(Gf.Vec3f(*inertia))
            mass.CreatePrincipalAxesAttr(source_mass.GetPrincipalAxesAttr().Get())
        collider_prims = []
        UsdGeom.Xform.Define(prepared, "/root/collisions")
        for index, item in enumerate(inspection["colliders"]):
            path = f"/root/collisions/mesh_{index:03d}"
            Sdf.CopySpec(stage.GetRootLayer(), item["collider"], prepared.GetRootLayer(), path)
            prim = prepared.GetPrimAtPath(path)
            transform = relative_transform(stage.GetPrimAtPath(item["collider"]), root)
            transform[:3, :] *= unit
            set_transform(prim, transform)
            UsdShade.MaterialBindingAPI(prim).UnbindAllBindings()
            collider_prims.append(prim)
        bind_material(prepared, collider_prims, config.material)
        prepared.GetRootLayer().Save()
        # Full-resolution, textured replay lives separately from the production collision display.
        for item in inspection["colliders"]:
            UsdPhysics.CollisionAPI(
                stage.GetPrimAtPath(item["collider"])
            ).CreateCollisionEnabledAttr(False)
        for prim in stage.Traverse():
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                UsdPhysics.RigidBodyAPI(prim).CreateRigidBodyEnabledAttr(False)
        set_transform(root, np.eye(4))
        retain_object_dependencies(stage)
        localize_assets(stage, directory)
        stage.GetRootLayer().Export(str(directory / "replay.usdc"))
        arrays = {
            "surface_vertices_m": np.asarray(surface.vertices),
            "surface_faces": np.asarray(surface.faces, dtype=np.int64),
            "T_object_C": np.eye(4),
        }
        for i, hull in enumerate(hulls):
            arrays[f"hull_{i:03d}_vertices_m"] = np.asarray(hull.vertices)
            arrays[f"hull_{i:03d}_faces"] = np.asarray(hull.faces, dtype=np.int64)
        np.savez_compressed(directory / "geometry.npz", allow_pickle=False, **arrays)
        references = {
            name: check_references(directory / name) for name in ("object.usdc", "replay.usdc")
        }
        physical = runtime.measure_object(directory / "object.usdc")
        if not np.isclose(physical["mass_kg"], config.mass_kg):
            raise RuntimeError("Cooked object mass differs from manifest")
        expected = [
            config.material.static_friction,
            config.material.dynamic_friction,
            config.material.restitution,
        ]
        if not np.allclose(physical["materials"], expected, atol=1e-6):
            raise RuntimeError("Cooked object material differs from manifest")
        write_json(directory / "inspection.json", inspection)
        from graspdatagen.inspection import inspect_object_3d

        inspect_object_3d(directory, config.name)
        seal_cache(
            directory,
            {
                "kind": "object",
                "name": config.name,
                "key": key,
                "source": fingerprint,
                "config": config.snapshot,
                "inspection": inspection,
                "collision_source": "source_asset",
                "cooked_hulls": len(hulls),
                "collision_vertices": sum(len(h.vertices) for h in hulls),
                "collision_triangles": sum(len(h.faces) for h in hulls),
                "references": references,
                "physical": physical,
                "T_object_C": np.eye(4).tolist(),
            },
        )
        directory.rename(destination)
    return destination
