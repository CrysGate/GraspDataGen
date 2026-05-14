#!/usr/bin/env python3
"""输出面向 gripper 适配的 USD 精简报告。

脚本会刻意避开 mesh 顶点数组这类噪声，只保留 gripper 适配最相关的信息：
rigid bodies、joints、collisions、mass、materials，以及 GraspDataGen config 匹配结果。
"""

from __future__ import annotations

import argparse
import json
import os
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable

from rich import box
from rich.console import Console
from rich.panel import Panel
from rich.table import Table
from rich.text import Text


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR
GRASPGEN_DIR = REPO_ROOT / "scripts" / "graspgen"
sys.path.insert(0, str(GRASPGEN_DIR))

console = Console(highlight=False)


IMPORTANT_ATTR_PREFIXES = (
    "physics:",
    "physx",
    "drive:",
    "material:",
    "collection:",
    "xformOp:",
)
IMPORTANT_ATTR_NAMES = {
    "extent",
    "visibility",
    "purpose",
}
SKIPPED_ROOT_NAMES = {
    "Render",
}


@dataclass
class Prim:
    path: str
    name: str
    type_name: str
    specifier: str
    parent: str | None
    line: int
    apis: list[str] = field(default_factory=list)
    attrs: dict[str, str] = field(default_factory=dict)
    rels: dict[str, str] = field(default_factory=dict)
    references: list[str] = field(default_factory=list)
    children: list[str] = field(default_factory=list)
    brace_depth: int = 0


@dataclass
class UsdSummary:
    usd_path: str
    metadata: dict[str, str]
    prims: dict[str, Prim]
    root_paths: list[str]
    warnings: list[str]


@dataclass
class Check:
    stage: str
    status: str
    title: str
    detail: str


PRIM_RE = re.compile(r'^\s*(def|over|class)\s+(?:(\w+)\s+)?"([^"]+)"')
API_RE = re.compile(r"apiSchemas\s*=\s*\[([^\]]*)\]")
REF_RE = re.compile(r"(?:add|prepend|append)\s+references\s*=\s*<([^>]+)>")
REL_RE = re.compile(r"^\s*rel\s+([\w:]+)\s*=\s*(.*)")
ATTR_RE = re.compile(r"^\s*(?:custom\s+)?(?:uniform\s+)?(?:[\w\[\]:<>]+)\s+([\w:]+)\s*=\s*(.*)")
META_RE = re.compile(r'^\s*(defaultPrim|metersPerUnit|upAxis)\s*=\s*(.*)')


def strip_value(value: str, limit: int = 140) -> str:
    value = value.strip().rstrip(",")
    if len(value) <= limit:
        return value
    return value[: limit - 3] + "..."


def parse_api_list(raw: str) -> list[str]:
    return [item.strip().strip('"') for item in raw.split(",") if item.strip()]


def clean_target(value: str) -> str:
    value = value.strip()
    if value.startswith("None"):
        return "None"
    if value.startswith("<") and ">" in value:
        return value[1 : value.index(">")]
    return strip_value(value)


def iter_usd_text(usd_path: str) -> Iterable[tuple[int, str]]:
    try:
        from pxr import Sdf
    except ImportError as exc:
        raise RuntimeError("无法导入 pxr.Sdf。请安装 USD Python bindings，例如 `pip install usd-core`。") from exc

    layer = Sdf.Layer.FindOrOpen(usd_path)
    if layer is None:
        raise RuntimeError(f"无法打开 USD layer: {usd_path}")

    for line_no, line in enumerate(layer.ExportToString().splitlines(), 1):
        yield line_no, line


def parse_usd(usd_path: str) -> UsdSummary:
    prims: dict[str, Prim] = {}
    root_paths: list[str] = []
    metadata: dict[str, str] = {}
    warnings: list[str] = []
    stack: list[Prim] = []

    for line_no, line in iter_usd_text(usd_path):
        meta = META_RE.match(line)
        if meta and not stack:
            metadata[meta.group(1)] = strip_value(meta.group(2))

        prim_match = PRIM_RE.match(line)
        if prim_match:
            specifier, maybe_type, name = prim_match.groups()
            parent = stack[-1].path if stack else None
            path = f"{parent}/{name}" if parent else f"/{name}"
            prim = Prim(
                path=path,
                name=name,
                type_name=maybe_type or "",
                specifier=specifier,
                parent=parent,
                line=line_no,
            )
            prims[path] = prim
            if parent and parent in prims:
                prims[parent].children.append(path)
            else:
                root_paths.append(path)
            stack.append(prim)

        if not stack:
            continue

        current = stack[-1]

        api_match = API_RE.search(line)
        if api_match:
            for api in parse_api_list(api_match.group(1)):
                if api not in current.apis:
                    current.apis.append(api)

        ref_match = REF_RE.search(line)
        if ref_match:
            current.references.append(ref_match.group(1))

        rel_match = REL_RE.match(line)
        if rel_match:
            current.rels[rel_match.group(1)] = clean_target(rel_match.group(2))

        attr_match = ATTR_RE.match(line)
        if attr_match and not rel_match and len(line) < 1000:
            name, value = attr_match.groups()
            if name in IMPORTANT_ATTR_NAMES or name.startswith(IMPORTANT_ATTR_PREFIXES):
                current.attrs[name] = strip_value(value)

        open_count = line.count("{")
        close_count = line.count("}")
        current.brace_depth += open_count - close_count
        while stack and stack[-1].brace_depth <= 0 and close_count > 0:
            stack.pop()
            close_count -= 1

    summary = UsdSummary(os.path.abspath(usd_path), metadata, prims, root_paths, warnings)
    add_warnings(summary)
    return summary


def has_api(prim: Prim, needle: str) -> bool:
    return any(needle in api for api in prim.apis)


def is_joint(prim: Prim) -> bool:
    return "Joint" in prim.type_name


def is_body(prim: Prim) -> bool:
    return has_api(prim, "RigidBodyAPI") or has_api(prim, "MassAPI")


def is_collision(prim: Prim) -> bool:
    return has_api(prim, "CollisionAPI") or "collision" in prim.name.lower()


def is_material(prim: Prim) -> bool:
    return prim.type_name == "Material" or has_api(prim, "MaterialAPI")


def is_physics_material(prim: Prim) -> bool:
    return has_api(prim, "PhysicsMaterial") or any(
        key.endswith("staticFriction") or key.endswith("dynamicFriction") or key.endswith("restitution")
        for key in prim.attrs
    )


def interesting_root(path: str, include_prototypes: bool) -> bool:
    name = path.rsplit("/", 1)[-1]
    if name in SKIPPED_ROOT_NAMES:
        return False
    if not include_prototypes and name.startswith("Flattened_Prototype"):
        return False
    return True


def descendants(summary: UsdSummary, path: str) -> list[Prim]:
    out: list[Prim] = []
    pending = list(summary.prims.get(path, Prim(path, "", "", "", None, 0)).children)
    while pending:
        child_path = pending.pop(0)
        child = summary.prims[child_path]
        out.append(child)
        pending.extend(child.children)
    return out


def referenced_extents(summary: UsdSummary, refs: list[str]) -> list[str]:
    extents: list[str] = []
    for ref in refs:
        ref_path = ref if ref.startswith("/") else f"/{ref}"
        ref_prim = summary.prims.get(ref_path)
        if not ref_prim:
            continue
        for prim in [ref_prim, *descendants(summary, ref_path)]:
            extent = prim.attrs.get("extent")
            if extent:
                extents.append(f"{ref}: {extent}")
    return extents


def parse_extent(value: str) -> tuple[tuple[float, float, float], tuple[float, float, float]] | None:
    numbers = [float(item) for item in re.findall(r"[-+]?(?:\d+\.\d*|\.\d+|\d+)(?:e[-+]?\d+)?", value, flags=re.IGNORECASE)]
    if len(numbers) < 6:
        return None
    return (numbers[0], numbers[1], numbers[2]), (numbers[3], numbers[4], numbers[5])


def extent_dims(value: str) -> tuple[float, float, float] | None:
    parsed = parse_extent(value)
    if parsed is None:
        return None
    lo, hi = parsed
    return tuple(abs(hi[i] - lo[i]) for i in range(3))


def fmt_dims(dims: tuple[float, float, float] | None) -> str:
    if dims is None:
        return "-"
    return "x".join(f"{value:.4g}" for value in dims)


def referenced_extent_dims(summary: UsdSummary, refs: list[str]) -> list[tuple[float, float, float]]:
    dims: list[tuple[float, float, float]] = []
    for extent in referenced_extents(summary, refs):
        value = extent.split(": ", 1)[1] if ": " in extent else extent
        parsed = extent_dims(value)
        if parsed is not None:
            dims.append(parsed)
    return dims


def max_extent_dims(summary: UsdSummary, refs: list[str]) -> tuple[float, float, float] | None:
    dims = referenced_extent_dims(summary, refs)
    if not dims:
        return None
    return tuple(max(dim[i] for dim in dims) for i in range(3))


def short_path(path: str | None) -> str:
    if not path:
        return "-"
    return path.rsplit("/", 1)[-1]


def prim_by_name(summary: UsdSummary, name: str | object) -> Prim | None:
    if not isinstance(name, str):
        return None
    matches = [prim for prim in summary.prims.values() if prim.name == name]
    return matches[0] if matches else None


def body_prims(summary: UsdSummary) -> list[Prim]:
    return [prim for prim in summary.prims.values() if is_body(prim)]


def joint_prims(summary: UsdSummary) -> list[Prim]:
    return [prim for prim in summary.prims.values() if is_joint(prim)]


def collision_children(summary: UsdSummary, prim: Prim) -> list[Prim]:
    return [summary.prims[path] for path in prim.children if is_collision(summary.prims[path])]


def visual_children(summary: UsdSummary, prim: Prim) -> list[Prim]:
    return [summary.prims[path] for path in prim.children if "visual" in summary.prims[path].name.lower()]


def physics_material_prims(summary: UsdSummary) -> list[Prim]:
    return [prim for prim in summary.prims.values() if is_physics_material(prim)]


def articulation_prims(summary: UsdSummary) -> list[Prim]:
    return [prim for prim in summary.prims.values() if has_api(prim, "Articulation")]


def root_candidates(summary: UsdSummary, include_prototypes: bool = False) -> list[Prim]:
    return [summary.prims[path] for path in summary.root_paths if interesting_root(path, include_prototypes)]


def status_rank(status: str) -> int:
    return {"FAIL": 0, "RISK": 1, "CHECK": 2, "OK": 3}.get(status, 2)


def overall_status(checks: list[Check], stage: str) -> str:
    stage_checks = [check for check in checks if check.stage == stage]
    if not stage_checks:
        return "CHECK"
    return min(stage_checks, key=lambda check: status_rank(check.status)).status


def config_values(config: dict[str, object] | None) -> tuple[str | None, list[str]]:
    if not config or "_error" in config:
        return None, []
    base = config.get("base_frame")
    fingers = config.get("finger_colliders")
    finger_names = fingers if isinstance(fingers, list) else []
    return base if isinstance(base, str) else None, [str(item) for item in finger_names]


def build_checks(summary: UsdSummary, config: dict[str, object] | None) -> list[Check]:
    checks: list[Check] = []
    bodies = body_prims(summary)
    joints = joint_prims(summary)
    moving_joints = [joint for joint in joints if "FixedJoint" not in joint.type_name]
    articulations = articulation_prims(summary)
    physics_materials = physics_material_prims(summary)
    base_name, finger_names = config_values(config)

    if not bodies:
        checks.append(Check("guess", "FAIL", "没有 rigid bodies", "gripper definition 无法映射 base/finger bodies。"))
        checks.append(Check("sim", "FAIL", "没有 rigid bodies", "没有 rigid bodies 时，PhysX 无法把它当作 gripper articulation 仿真。"))
    else:
        checks.append(Check("guess", "OK", "已找到 rigid bodies", f"发现 {len(bodies)} 个 body/link prim。"))

    if base_name:
        base = prim_by_name(summary, base_name)
        if base is None:
            checks.append(Check("guess", "FAIL", "配置中的 base_frame 缺失", f"在 USD prim names 中找不到 '{base_name}'。"))
            checks.append(Check("sim", "RISK", "配置中的 base_frame 缺失", "Grasp YAML 的 frame metadata 可能和实际加载的 articulation 对不上。"))
        else:
            checks.append(Check("guess", "OK", "base_frame 可解析", f"{base_name} -> {base.path}"))

    if finger_names:
        missing = [name for name in finger_names if prim_by_name(summary, name) is None]
        if missing:
            checks.append(Check("guess", "FAIL", "配置中的 finger_colliders 缺失", f"缺失: {', '.join(missing)}。"))
            checks.append(Check("sim", "FAIL", "Contact sensor 目标缺失", "grasp_sim 会按 finger_colliders 的 body name 挂 contact sensors。"))
        else:
            checks.append(Check("guess", "OK", "finger_colliders 可解析", ", ".join(finger_names)))
            checks.append(Check("sim", "OK", "contact sensor body names 可解析", ", ".join(finger_names)))
            for name in finger_names:
                finger = prim_by_name(summary, name)
                if finger and not collision_children(summary, finger):
                    checks.append(Check("guess", "FAIL", f"{name} 没有 collision child", "grasp_guess 依赖 finger collision meshes 做 raycast 和 opening 检查。"))
                    checks.append(Check("sim", "FAIL", f"{name} 没有 collision child", "这个 finger 可能无法产生 PhysX contacts。"))

    if not moving_joints:
        checks.append(Check("guess", "FAIL", "没有可运动 joints", "无法推导 open/closed c-space samples。"))
        checks.append(Check("sim", "FAIL", "没有可运动 joints", "gripper 无法通过 articulation control 闭合。"))
    else:
        driven = [joint for joint in moving_joints if any("DriveAPI" in api for api in joint.apis)]
        mimic = [joint for joint in moving_joints if any("MimicJointAPI" in api for api in joint.apis)]
        checks.append(Check("guess", "OK", "已找到可运动 joints", f"{len(moving_joints)} 个 moving joints，{len(driven)} 个 driven，{len(mimic)} 个 mimic/follower。"))
        if not driven:
            checks.append(Check("guess", "RISK", "未检测到 driven joints", "create_gripper_lab 采样 c-space 时可能会把 non-driven joints 置零。"))
            checks.append(Check("sim", "RISK", "未检测到 driven joints", "grasp_sim 可能没有可用于闭合 gripper 的 actuator target。"))
        for joint in moving_joints:
            lower = joint.attrs.get("physics:lowerLimit")
            upper = joint.attrs.get("physics:upperLimit")
            if lower is None or upper is None:
                checks.append(Check("guess", "RISK", f"{joint.name} 缺少 limits", "open/closed 采样依赖 joint limits。"))
            elif lower == upper:
                checks.append(Check("guess", "RISK", f"{joint.name} 的活动范围为零", f"lowerLimit == upperLimit == {lower}。"))

    if articulations:
        checks.append(Check("sim", "OK", "已找到 Articulation root/API", ", ".join(short_path(prim.path) for prim in articulations)))
    else:
        checks.append(Check("sim", "FAIL", "没有 articulation API", "IsaacLab 期望 gripper 能作为 articulation 加载。"))

    collision_prims = [prim for prim in summary.prims.values() if is_collision(prim)]
    if collision_prims:
        checks.append(Check("guess", "OK", "已找到 collision geometry", f"发现 {len(collision_prims)} 个 collision prims。"))
        checks.append(Check("sim", "OK", "已找到 collision geometry", f"发现 {len(collision_prims)} 个 collision prims。"))
    else:
        checks.append(Check("guess", "FAIL", "没有 collision geometry", "grasp_guess 无法进行有意义的 collision checks。"))
        checks.append(Check("sim", "FAIL", "没有 collision geometry", "PhysX 无法产生 contacts。"))

    if physics_materials:
        checks.append(Check("sim", "OK", "已找到 physics material", f"发现 {len(physics_materials)} 个 physics material/friction prims。"))
    else:
        checks.append(Check("sim", "RISK", "gripper USD 中没有显式 finger friction", "除非其他 layer 覆盖，否则 finger friction 会依赖 simulator defaults。"))

    for body in bodies:
        mass = body.attrs.get("physics:mass")
        if mass is not None and mass.strip() in {"0", "0.0"}:
            stage = "sim"
            checks.append(Check(stage, "RISK", f"{body.name} 的 mass 为 0", "如果它是 fixed base 可能合理，但仍需要在 simulation 里确认 finger/base dynamics。"))
    return checks


def compact_apis(apis: list[str]) -> str:
    if not apis:
        return "-"
    return ", ".join(apis)


def format_attrs(prim: Prim, names: Iterable[str]) -> list[str]:
    lines = []
    for name in names:
        if name in prim.attrs:
            lines.append(f"{name}: {prim.attrs[name]}")
    return lines


def print_attrs(prim: Prim, names: Iterable[str], indent: str = "    ") -> None:
    for line in format_attrs(prim, names):
        print(f"{indent}{line}")


def load_gripper_config(config_name: str | None) -> dict[str, object] | None:
    if not config_name:
        return None
    try:
        from gripper_configurations import get_gripper_config

        return get_gripper_config(config_name)
    except Exception as exc:  # noqa: BLE001 - this is a diagnostic helper.
        return {"_error": str(exc)}


def add_warnings(summary: UsdSummary) -> None:
    bodies = [prim for prim in summary.prims.values() if is_body(prim)]
    joints = [prim for prim in summary.prims.values() if is_joint(prim)]
    collisions = [prim for prim in summary.prims.values() if is_collision(prim)]
    articulations = [prim for prim in summary.prims.values() if has_api(prim, "Articulation")]
    physics_materials = [prim for prim in summary.prims.values() if is_physics_material(prim)]

    if not bodies:
        summary.warnings.append("没有找到 rigid bodies 或 MassAPI prims。")
    if not joints:
        summary.warnings.append("没有找到 physics joints。")
    if not collisions:
        summary.warnings.append("没有找到 collision prims。")
    if not articulations:
        summary.warnings.append("没有找到 articulation root/API。")
    if not physics_materials:
        summary.warnings.append("没有找到显式 physics material/friction attributes；friction 可能依赖 simulator defaults。")

    for body in bodies:
        mass = body.attrs.get("physics:mass")
        if mass is not None and mass.strip() in {"0", "0.0"}:
            summary.warnings.append(f"{body.path} 的 physics:mass = {mass}。请确认这是有意设置。")
        body_collisions = [summary.prims[path] for path in body.children if is_collision(summary.prims[path])]
        if not body_collisions:
            summary.warnings.append(f"{body.path} 是 rigid body，但没有直接的 collision child。")


def print_section(title: str) -> None:
    print(f"\n{title}")
    print("-" * len(title))


def status_word(status: str) -> str:
    return {
        "OK": "OK",
        "CHECK": "CHECK",
        "RISK": "RISK",
        "FAIL": "FAIL",
    }.get(status, status)


def status_style(status: str) -> str:
    return {
        "OK": "bold green",
        "CHECK": "bold cyan",
        "RISK": "bold yellow",
        "FAIL": "bold red",
    }.get(status, "white")


def stage_style(status: str) -> str:
    return {
        "OK": "green",
        "CHECK": "cyan",
        "RISK": "yellow",
        "FAIL": "red",
    }.get(status, "white")


def status_badge(status: str) -> Text:
    return Text(f" {status_word(status)} ", style=f"{status_style(status)} reverse")


def checks_table(checks: list[Check], stage: str) -> Table:
    table = Table(box=box.SIMPLE_HEAVY, expand=True, show_lines=False)
    table.add_column("状态", width=8, justify="center")
    table.add_column("检查项", style="bold", overflow="fold")
    table.add_column("说明", ratio=2, overflow="fold")
    stage_checks = [check for check in checks if check.stage == stage]
    for check in sorted(stage_checks, key=lambda item: status_rank(item.status)):
        table.add_row(status_badge(check.status), check.title, check.detail)
    return table


def key_value_table(rows: list[tuple[str, object]], title: str | None = None) -> Table:
    table = Table(title=title, box=box.SIMPLE, show_header=False, expand=True)
    table.add_column("字段", style="bold cyan", no_wrap=True)
    table.add_column("值", overflow="fold")
    for key, value in rows:
        table.add_row(str(key), str(value))
    return table


def compact_api_tags(prim: Prim) -> str:
    tags = []
    if has_api(prim, "PhysxRigidBodyAPI"):
        tags.append("PhysXRigid")
    if has_api(prim, "RigidBodyAPI"):
        tags.append("Rigid")
    if has_api(prim, "MassAPI"):
        tags.append("Mass")
    if has_api(prim, "CollisionAPI"):
        tags.append("Collision")
    if has_api(prim, "Articulation"):
        tags.append("Articulation")
    return ",".join(tags) if tags else "-"


def collision_summary(summary: UsdSummary, body: Prim) -> str:
    collisions = collision_children(summary, body)
    if not collisions:
        return "none"
    parts = []
    for collision in collisions:
        approx = collision.attrs.get("physics:approximation", "-").strip('"')
        material = collision.rels.get("material:binding") or collision.attrs.get("material:binding")
        material_note = "mat=none" if material == "None" else ("mat=yes" if material else "mat=-")
        dims = fmt_dims(max_extent_dims(summary, collision.references))
        parts.append(f"{collision.name}:{approx}, {material_note}, bbox={dims}")
    return "; ".join(parts)


def visual_summary(summary: UsdSummary, body: Prim) -> str:
    visuals = visual_children(summary, body)
    if not visuals:
        return "none"
    return "; ".join(f"{visual.name}:bbox={fmt_dims(max_extent_dims(summary, visual.references))}" for visual in visuals)


def body_visual_lines(summary: UsdSummary, body: Prim) -> list[str]:
    visuals = visual_children(summary, body)
    if not visuals:
        return ["visual: none"]
    lines = []
    for visual in visuals:
        lines.append(f"visual {visual.name}: bbox={fmt_dims(max_extent_dims(summary, visual.references))}")
    return lines


def body_collision_lines(summary: UsdSummary, body: Prim) -> list[str]:
    collisions = collision_children(summary, body)
    if not collisions:
        return ["collision: none"]
    lines = []
    for collision in collisions:
        approx = collision.attrs.get("physics:approximation", "-").strip('"')
        material = collision.rels.get("material:binding") or collision.attrs.get("material:binding")
        material_note = "none" if material == "None" else ("yes" if material else "-")
        lines.append(f"collision {collision.name}:")
        lines.append(f"  approximation={approx}")
        lines.append(f"  material={material_note}")
        lines.append(f"  bbox={fmt_dims(max_extent_dims(summary, collision.references))}")
    return lines


def print_link_table(summary: UsdSummary, config: dict[str, object] | None) -> None:
    base_name, finger_names = config_values(config)
    for body in body_prims(summary):
        roles = []
        if body.name == base_name:
            roles.append("base")
        if body.name in finger_names:
            roles.append("finger")
        role = ",".join(roles) if roles else "-"
        mass = body.attrs.get("physics:mass", "-")
        transform = body.attrs.get("xformOp:translate", "-")
        print(f"{body.name} ({role})")
        print(f"  路径: {body.path}")
        print(f"  APIs: {compact_api_tags(body)} | mass: {mass} | translate: {transform}")
        print(f"  visual:    {visual_summary(summary, body)}")
        print(f"  collision: {collision_summary(summary, body)}")


def print_joint_table(summary: UsdSummary) -> None:
    for joint in joint_prims(summary):
        body0 = short_path(joint.rels.get("physics:body0"))
        body1 = short_path(joint.rels.get("physics:body1"))
        axis = joint.attrs.get("physics:axis", "-").strip('"')
        lower = joint.attrs.get("physics:lowerLimit", "-")
        upper = joint.attrs.get("physics:upperLimit", "-")
        drive_bits = []
        for key in (
            "drive:linear:physics:stiffness",
            "drive:linear:physics:damping",
            "drive:linear:physics:maxForce",
            "drive:angular:physics:stiffness",
            "drive:angular:physics:damping",
            "drive:angular:physics:maxForce",
        ):
            if key in joint.attrs:
                drive_bits.append(f"{key.split(':')[-1]}={joint.attrs[key]}")
        mimic_bits = []
        for rel_name, rel_value in joint.rels.items():
            if "MimicJoint" in rel_name:
                mimic_bits.append(f"ref={short_path(rel_value)}")
        for key, value in joint.attrs.items():
            if "MimicJoint" in key and (key.endswith(":gearing") or key.endswith(":offset")):
                mimic_bits.append(f"{key.rsplit(':', 1)[-1]}={value}")
        print(f"{joint.name} [{joint.type_name}]")
        print(f"  连接: {body0} -> {body1} | axis: {axis} | limits: [{lower}, {upper}]")
        print(f"  APIs: {compact_apis(joint.apis)}")
        print(f"  drive: {', '.join(drive_bits) if drive_bits else '-'}")
        print(f"  mimic: {', '.join(mimic_bits) if mimic_bits else '-'}")


def print_stage_explanation() -> None:
    console.print(
        Panel(
            "guess 使用从 USD 派生出的 geometry 和 kinematics：配置里的 base/finger names、"
            "joint limits、collision meshes、body transforms、open_axis、approach_axis 和 bite_point。\n"
            "sim 使用真正加载进 PhysX 的资产：articulation root、rigid bodies、joints/drives、"
            "contact collider names、collision geometry、mass/inertia，以及 physics material/friction。",
            title="读法提示",
            border_style="blue",
            box=box.ROUNDED,
        )
    )


def print_next_actions(checks: list[Check]) -> None:
    guess_status = overall_status(checks, "guess")
    sim_status = overall_status(checks, "sim")
    lines = []
    if guess_status in {"FAIL", "RISK"}:
        lines.append("guess/definition: 先重新生成 gripper definition，再检查 open_limit、open_axis、approach_axis、bite_point、open_widths 和 joint_cspace_pos。")
    else:
        lines.append("guess/definition: 核心 USD 字段都存在；但在信任 candidate grasps 前，仍要检查生成的 .npz open/closed states。")
    if sim_status in {"FAIL", "RISK"}:
        lines.append(
            "sim: 重点看 articulation loading、close direction、finger contact sensors、"
            "friction/materials、mass/inertia，以及 grasp_sim 是否真的记录到双指 contacts。"
        )
    else:
        lines.append("sim: 核心 PhysX 字段都存在；跑一个小规模 grasp_sim smoke test，确认真实 contacts 和 hold behavior。")
    console.print(Panel("\n".join(lines), title="建议验证重点", border_style="magenta", box=box.ROUNDED))


def print_summary(
    summary: UsdSummary,
    config_name: str | None,
    include_prototypes: bool,
    show_visual_materials: bool,
    details: bool,
) -> None:
    config = load_gripper_config(config_name)
    checks = build_checks(summary, config)
    bodies = body_prims(summary)
    joints = joint_prims(summary)
    collisions = [prim for prim in summary.prims.values() if is_collision(prim)]
    articulations = articulation_prims(summary)
    physics_materials = physics_material_prims(summary)
    roots = root_candidates(summary, include_prototypes)

    stage_text = ", ".join(f"{key}={value}" for key, value in summary.metadata.items()) if summary.metadata else "-"
    header = key_value_table(
        [
            ("文件", summary.usd_path),
            ("stage", stage_text),
            (
                "数量",
                f"roots={len(roots)}, bodies={len(bodies)}, joints={len(joints)}, "
                f"collisions={len(collisions)}, articulations={len(articulations)}, physics_materials={len(physics_materials)}",
            ),
        ]
    )
    console.print(Panel(header, title="Gripper USD 报告", border_style="bold blue", box=box.ROUNDED))

    readiness = Table(box=box.ROUNDED, expand=True)
    readiness.add_column("阶段", style="bold", width=18)
    readiness.add_column("就绪度", width=10, justify="center")
    readiness.add_column("最相关的 USD 内容")
    guess_status = overall_status(checks, "guess")
    sim_status = overall_status(checks, "sim")
    readiness.add_row(
        "grasp_guess",
        status_badge(guess_status),
        "base_frame / finger_colliders / joint limits / collision meshes / open_axis / approach_axis / bite_point",
    )
    readiness.add_row(
        "grasp_sim",
        status_badge(sim_status),
        "ArticulationRootAPI / rigid bodies / joints & drives / contact colliders / mass / friction",
    )
    console.print(Panel(readiness, title="与 pipeline 的关系", border_style=stage_style(min([guess_status, sim_status], key=status_rank)), box=box.ROUNDED))

    console.print(Panel(checks_table(checks, "guess"), title="guess 阶段检查", border_style=stage_style(guess_status), box=box.ROUNDED))
    console.print(Panel(checks_table(checks, "sim"), title="sim 阶段检查", border_style=stage_style(sim_status), box=box.ROUNDED))
    print_stage_explanation()
    print_next_actions(checks)

    if config_name:
        if config and "_error" in config:
            console.print(Panel(f"{config_name}: 加载 config 失败: {config['_error']}", title="GraspDataGen config", border_style="red"))
        else:
            assert config is not None
            rows: list[tuple[str, object]] = [("name", config_name)]
            for key in ("gripper_file", "base_frame", "finger_colliders", "bite", "pinch_width_resolution"):
                if key in config:
                    rows.append((key, config[key]))
            base_name, finger_names = config_values(config)
            if base_name:
                base = prim_by_name(summary, base_name)
                rows.append(("resolved base_frame", base.path if base else "MISSING"))
            for finger_name in finger_names:
                finger = prim_by_name(summary, finger_name)
                rows.append((f"resolved finger_collider {finger_name}", finger.path if finger else "MISSING"))
            console.print(Panel(key_value_table(rows), title="GraspDataGen 如何识别这个 gripper", border_style="cyan", box=box.ROUNDED))

    top_table = Table(box=box.SIMPLE_HEAVY, expand=True)
    top_table.add_column("root prim", style="bold")
    top_table.add_column("类型", width=18)
    top_table.add_column("APIs", ratio=1)
    top_table.add_column("children", ratio=2)
    for prim in roots:
        child_names = [summary.prims[child].name for child in prim.children]
        top_table.add_row(
            prim.path,
            f"{prim.specifier} {prim.type_name or 'untyped'}",
            compact_apis(prim.apis),
            ", ".join(child_names) if child_names else "-",
        )
    console.print(Panel(top_table, title="Top-level structure", border_style="white", box=box.ROUNDED))

    link_table = Table(box=box.SIMPLE_HEAVY, expand=True, show_lines=True)
    link_table.add_column("link", style="bold", no_wrap=True)
    link_table.add_column("摘要", ratio=3, overflow="fold")
    base_name, finger_names = config_values(config)
    for body in bodies:
        roles = []
        if body.name == base_name:
            roles.append("base")
        if body.name in finger_names:
            roles.append("finger")
        role = ",".join(roles) if roles else "-"
        mass = body.attrs.get("physics:mass", "-")
        transform = body.attrs.get("xformOp:translate", "-")
        link_table.add_row(
            f"{body.name} [dim]({role})[/dim]\n[dim]{body.path}[/dim]",
            "\n".join(
                [
                    f"APIs: {compact_api_tags(body)}",
                    f"mass: {mass}",
                    f"translate: {transform}",
                    *body_visual_lines(summary, body),
                    *body_collision_lines(summary, body),
                ]
            ),
        )
    console.print(Panel(link_table, title="Links, visuals, and collisions", border_style="green", box=box.ROUNDED))

    joint_table = Table(box=box.SIMPLE_HEAVY, expand=True, show_lines=True)
    joint_table.add_column("joint", style="bold", no_wrap=True)
    joint_table.add_column("摘要", ratio=3, overflow="fold")
    for joint in joints:
        body0 = short_path(joint.rels.get("physics:body0"))
        body1 = short_path(joint.rels.get("physics:body1"))
        axis = joint.attrs.get("physics:axis", "-").strip('"')
        lower = joint.attrs.get("physics:lowerLimit", "-")
        upper = joint.attrs.get("physics:upperLimit", "-")
        drive_bits = []
        for key in (
            "drive:linear:physics:stiffness",
            "drive:linear:physics:damping",
            "drive:linear:physics:maxForce",
            "drive:angular:physics:stiffness",
            "drive:angular:physics:damping",
            "drive:angular:physics:maxForce",
        ):
            if key in joint.attrs:
                drive_bits.append(f"{key.split(':')[-1]}={joint.attrs[key]}")
        mimic_bits = []
        for rel_name, rel_value in joint.rels.items():
            if "MimicJoint" in rel_name:
                mimic_bits.append(f"ref={short_path(rel_value)}")
        for key, value in joint.attrs.items():
            if "MimicJoint" in key and (key.endswith(":gearing") or key.endswith(":offset")):
                mimic_bits.append(f"{key.rsplit(':', 1)[-1]}={value}")
        joint_table.add_row(
            f"{joint.name}\n[dim]{joint.type_name}[/dim]",
            "\n".join(
                [
                    f"连接: {body0} -> {body1}",
                    f"axis: {axis}",
                    f"limits: [{lower}, {upper}]",
                    f"drive: {', '.join(drive_bits) if drive_bits else '-'}",
                    f"mimic: {', '.join(mimic_bits) if mimic_bits else '-'}",
                    f"APIs: {compact_apis(joint.apis)}",
                ]
            ),
        )
    console.print(Panel(joint_table, title="Joint 和运动模型", border_style="yellow", box=box.ROUNDED))

    materials = [prim for prim in summary.prims.values() if is_material(prim)]
    visual_materials = [prim for prim in materials if prim not in physics_materials]
    material_rows: list[tuple[str, object]] = []
    if physics_materials:
        material_rows.append(("Physics materials", len(physics_materials)))
        for material in physics_materials:
            attrs = ", ".join(format_attrs(material, ("physics:staticFriction", "physics:dynamicFriction", "physics:restitution"))) or "-"
            material_rows.append((material.path, attrs))
    else:
        material_rows.append(("Physics materials", "未找到"))
    if show_visual_materials:
        material_rows.append(("Visual materials", len(visual_materials)))
        for material in visual_materials:
            material_rows.append((material.path, "visual"))
    else:
        material_rows.append(("Visual materials", f"找到 {len(visual_materials)} 个（默认隐藏；使用 --show-visual-materials 显示）"))
    console.print(Panel(key_value_table(material_rows), title="Materials", border_style="blue", box=box.ROUNDED))

    warning_text = "\n".join(f"- {warning}" for warning in summary.warnings) if summary.warnings else "(none)"
    console.print(Panel(warning_text, title="原始 warnings", border_style="red" if summary.warnings else "green", box=box.ROUNDED))

    if details:
        detailed_collisions = [prim for prim in summary.prims.values() if is_collision(prim) and not is_body(prim)]
        detail_table = Table(box=box.SIMPLE_HEAVY, expand=True)
        detail_table.add_column("collision prim", style="bold", no_wrap=True)
        detail_table.add_column("摘要", ratio=3, overflow="fold")
        for collision in detailed_collisions:
            attrs = ", ".join(
                format_attrs(
                    collision,
                    (
                        "physics:approximation",
                        "physxConvexDecompositionCollision:shrinkWrap",
                        "material:binding",
                        "visibility",
                    ),
                )
            )
            detail_table.add_row(
                collision.path,
                "\n".join(
                    [
                        f"parent: {collision.parent or '-'}",
                        f"APIs: {compact_apis(collision.apis)}",
                        f"关键属性: {attrs or '-'}",
                    ]
                ),
            )
        console.print(Panel(detail_table if detailed_collisions else "(none)", title="详细 collision prims", border_style="white", box=box.ROUNDED))


def to_jsonable(summary: UsdSummary) -> dict[str, object]:
    return {
        "usd_path": summary.usd_path,
        "metadata": summary.metadata,
        "roots": summary.root_paths,
        "warnings": summary.warnings,
        "prims": {
            path: {
                "name": prim.name,
                "type": prim.type_name,
                "specifier": prim.specifier,
                "parent": prim.parent,
                "apis": prim.apis,
                "attrs": prim.attrs,
                "rels": prim.rels,
                "references": prim.references,
                "children": prim.children,
                "line": prim.line,
            }
            for path, prim in summary.prims.items()
            if any((is_body(prim), is_joint(prim), is_collision(prim), is_physics_material(prim), has_api(prim, "Articulation")))
        },
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="输出 gripper 相关的 USD 内部信息摘要，并避开 mesh data 噪声。")
    parser.add_argument("usd_file", help="USD/USDA/USDC 文件路径。")
    parser.add_argument("--gripper-config", help="可选：用于对照的 GRIPPER_CONFIGS key。")
    parser.add_argument("--include-prototypes", action="store_true", help="在 top-level structure 中显示 Flattened_Prototype roots。")
    parser.add_argument("--show-visual-materials", action="store_true", help="列出 visual material paths。")
    parser.add_argument("--details", action="store_true", help="在主报告后追加更底层的 collision prim details。")
    parser.add_argument("--json", action="store_true", help="输出 machine-readable JSON，而不是文本报告。")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    usd_path = os.path.abspath(args.usd_file)
    summary = parse_usd(usd_path)
    if args.json:
        print(json.dumps(to_jsonable(summary), indent=2, sort_keys=True))
    else:
        print_summary(summary, args.gripper_config, args.include_prototypes, args.show_visual_materials, args.details)


if __name__ == "__main__":
    main()
