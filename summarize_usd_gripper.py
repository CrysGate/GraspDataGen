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


def enrich_with_pxr(summary: UsdSummary) -> None:
    """用真正的 pxr.Usd.Stage 把每个 prim 的真实 type 和 applied APIs 合并回 summary。

    纯文本扫描看不到 reference / over 的 composition 结果。例如
    `/Robotiq_2F_86/right_inner_finger/visuals/...` 下的 mesh 在文本里只能看到
    `over "..."`（无 type），真正的 `def Mesh "..."` 在 prototype 段。Composition
    完成后才能拿到正确 type=Mesh + CollisionAPI。
    """
    try:
        from pxr import Usd  # type: ignore
    except ImportError:
        return
    try:
        stage = Usd.Stage.Open(summary.usd_path)
    except Exception:  # noqa: BLE001
        return
    if stage is None:
        return
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        s_prim = summary.prims.get(path)
        if s_prim is None:
            continue
        if not s_prim.type_name:
            tn = prim.GetTypeName()
            if tn:
                s_prim.type_name = str(tn)
        apis_set = set(s_prim.apis)
        for api in prim.GetAppliedSchemas():
            if api not in apis_set:
                s_prim.apis.append(api)
                apis_set.add(api)


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
            existing = prims.get(path)
            if existing is not None:
                # USD `over "..."` 不带 type，会覆盖前面 `def Mesh "..."` 的记录。
                # 合并：保留 type / apis / refs / 已收集 attrs，让后续 attrs 继续累加。
                prim = existing
                if maybe_type and not prim.type_name:
                    prim.type_name = maybe_type
                # specifier 优先保留 "def"
                if existing.specifier != "def" and specifier == "def":
                    prim.specifier = specifier
            else:
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
    enrich_with_pxr(summary)
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


def prims_by_name(summary: UsdSummary, name: str | object) -> list[Prim]:
    if not isinstance(name, str):
        return []
    return [prim for prim in summary.prims.values() if prim.name == name]


def prims_by_name_in_default(summary: UsdSummary, name: str | object) -> list[Prim]:
    """只返回 defaultPrim 子树里同名的 prim。

    扁平化的 Flattened_Prototype_* 段在 USD 文本里也会出现同名 prim，但 IsaacLab
    UsdFileCfg 只 spawn defaultPrim，所以这些 prototype 副本不应该参与"finger 实际
    解析到哪个 link"的判定。
    """
    if not isinstance(name, str):
        return []
    default = summary.metadata.get("defaultPrim", "").strip().strip('"')
    if not default:
        return prims_by_name(summary, name)
    root = f"/{default}"
    prefix = f"{root}/"
    return [
        prim for prim in summary.prims.values()
        if prim.name == name and (prim.path == root or prim.path.startswith(prefix))
    ]


def body_prims_by_name_in_default(summary: UsdSummary, name: str | object) -> list[Prim]:
    """defaultPrim 子树里同名 且 是 rigid body 的 prim。

    finger / base 在 USD 文本里可能被 collisions/visuals 子 prim 同名复用，但
    IsaacLab 的 body_names 只看 RigidBodyAPI，所以判定时也应该过滤一遍。
    """
    return [prim for prim in prims_by_name_in_default(summary, name) if is_body(prim)]


def prim_by_name(summary: UsdSummary, name: str | object) -> Prim | None:
    matches = prims_by_name(summary, name)
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


def is_driven_joint(joint: Prim) -> bool:
    return any("DriveAPI" in api for api in joint.apis)


def is_mesh_collider(prim: Prim) -> bool:
    """直接挂在 Mesh prim 上的 CollisionAPI。"""
    return prim.type_name == "Mesh" and has_api(prim, "CollisionAPI")


def is_real_mesh_collider(prim: Prim) -> bool:
    """认作有效的 collider：要么是 Mesh + CollisionAPI，要么是 Xform + CollisionAPI 且
    有 reference 指向 prototype（prototype 内部含 Mesh）。Xform + CollisionAPI 但既不
    是 Mesh 也无 reference 时，运行时 usd_tools 不会收集顶点（Piper 早期那个坑）。
    """
    if not has_api(prim, "CollisionAPI"):
        return False
    return prim.type_name == "Mesh" or bool(prim.references)


def collider_yields_mesh(summary: UsdSummary, prim: Prim) -> bool:
    """判一个挂 PhysicsCollisionAPI 的 prim 在运行时能否真的提供 Mesh 顶点。

    usd_tools.get_prim_collision_mesh 会从启用了 CollisionAPI 的 prim 起向下递归，
    然后只在 type_name == 'Mesh' 时收集顶点。所以合法的 collider 形态有三种：
      1) prim 自己就是 Mesh（且挂 CollisionAPI）
      2) prim 通过 reference 指向 prototype（prototype 里递归会展开出 Mesh）
      3) prim 是 Xform + CollisionAPI，但它的子孙里有 Mesh prim
         （父 collision_enabled 被子 Mesh 继承）
    """
    if prim.type_name == "Mesh" or prim.references:
        return True
    for child in descendants_with_self(summary, prim):
        if child.type_name == "Mesh":
            return True
    return False


def descendants_with_self(summary: UsdSummary, prim: Prim) -> list[Prim]:
    return [prim, *descendants(summary, prim.path)]


def find_world_fixed_joints(summary: UsdSummary) -> list[Prim]:
    """挑出 body0 指向 world (None / 空) 的 PhysicsFixedJoint。"""
    out: list[Prim] = []
    for joint in joint_prims(summary):
        if "FixedJoint" not in joint.type_name:
            continue
        body0 = joint.rels.get("physics:body0")
        if body0 is None or body0 in {"", "None"}:
            out.append(joint)
    return out


def driving_joint_drive_attrs(joint: Prim) -> dict[str, float]:
    out: dict[str, float] = {}
    for key, value in joint.attrs.items():
        if not key.startswith("drive:"):
            continue
        try:
            out[key] = float(str(value).strip())
        except (TypeError, ValueError):
            continue
    return out


def parse_translate(value: str) -> tuple[float, float, float] | None:
    nums = re.findall(r"[-+]?(?:\d+\.\d*|\.\d+|\d+)(?:e[-+]?\d+)?", value, flags=re.IGNORECASE)
    if len(nums) < 3:
        return None
    return float(nums[0]), float(nums[1]), float(nums[2])


def check_mesh_triangulation(usd_path: str) -> tuple[list[str], str | None]:
    """用 pxr 真正打开 USD，校验每个 Mesh 的 faceVertexCounts。

    返回 (errors, skip_reason)。skip_reason 非 None 表示由于环境原因（缺 pxr 等）
    没有跑成检查，调用方应当把它当 CHECK 而不是 OK/FAIL。
    """
    try:
        from pxr import Usd, UsdGeom  # type: ignore
    except ImportError as exc:
        return [], f"未安装 pxr (USD Python bindings)；跳过三角面检查（{exc}）。"

    errors: list[str] = []
    try:
        stage = Usd.Stage.Open(usd_path, Usd.Stage.LoadAll)
    except Exception as exc:  # noqa: BLE001
        return [], f"无法打开 USD stage 做三角面检查: {exc}"
    if stage is None:
        return [], "无法打开 USD stage 做三角面检查。"

    for prim in stage.Traverse():
        if prim.GetTypeName() != "Mesh":
            continue
        mesh = UsdGeom.Mesh(prim)
        counts_attr = mesh.GetFaceVertexCountsAttr()
        if not counts_attr or not counts_attr.HasValue():
            continue
        counts = counts_attr.Get()
        if counts is None:
            continue
        non_tri = sum(1 for c in counts if c != 3)
        if non_tri:
            errors.append(f"{prim.GetPath()}: {non_tri}/{len(counts)} 个非三角面")
    return errors, None


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

    # --- 新增的硬性 / 软性检查 ---

    # (a) defaultPrim 必须是 articulation root 的祖先（IsaacLab UsdFileCfg 只 spawn defaultPrim 子树）
    default_prim_raw = summary.metadata.get("defaultPrim", "").strip().strip('"')
    default_prim_name = default_prim_raw or None
    if default_prim_name:
        if articulations:
            articulation_paths = [prim.path for prim in articulations]
            on_default = any(
                path == f"/{default_prim_name}" or path.startswith(f"/{default_prim_name}/")
                for path in articulation_paths
            )
            if not on_default:
                checks.append(Check("sim", "FAIL",
                    "Articulation root 不在 defaultPrim 子树内",
                    f"defaultPrim='{default_prim_name}'，但 articulation 在 {', '.join(articulation_paths)}。"
                    "IsaacLab UsdFileCfg 只 spawn defaultPrim 及其子树。"))
            else:
                checks.append(Check("sim", "OK",
                    "Articulation root 在 defaultPrim 子树内",
                    f"defaultPrim='{default_prim_name}'。"))
    else:
        checks.append(Check("sim", "RISK",
            "USD 未声明 defaultPrim",
            "metadata 中找不到 defaultPrim；IsaacLab UsdFileCfg 加载行为未定义。"))

    # (b) 必须有 PhysicsFixedJoint 把 base_frame 锚到 world
    world_fixed_joints = find_world_fixed_joints(summary)
    if base_name:
        base_anchored = any(
            short_path(joint.rels.get("physics:body1")) == base_name
            for joint in world_fixed_joints
        )
        if not base_anchored:
            checks.append(Check("sim", "FAIL",
                "缺少把 base 锚到 world 的 PhysicsFixedJoint",
                f"未找到 body0=None 且 body1=/.../{base_name} 的 fixed joint；"
                "spawn 后整把夹爪会自由下落/漂走。"))
        else:
            checks.append(Check("sim", "OK",
                "base 已通过 fixed joint 锚到 world",
                f"base_frame='{base_name}'。"))

    # (c) 配置里指定的 finger / base 在 defaultPrim 子树里是否唯一（只看 rigid body）
    if base_name:
        bases = body_prims_by_name_in_default(summary, base_name)
        if len(bases) > 1:
            checks.append(Check("guess", "FAIL",
                f"base_frame '{base_name}' 在 defaultPrim 子树里有 {len(bases)} 个 rigid body 匹配",
                ", ".join(prim.path for prim in bases) +
                "；summarize 取第一个，但 create_gripper_lab 可能解析到另一个。"))

    if finger_names:
        for name in finger_names:
            matches = body_prims_by_name_in_default(summary, name)
            if len(matches) > 1:
                checks.append(Check("guess", "FAIL",
                    f"finger_collider '{name}' 在 defaultPrim 子树里有 {len(matches)} 个 rigid body 匹配",
                    ", ".join(prim.path for prim in matches) +
                    "；body_names.index 取第一个，可能与你预期不一致。"))

            # (d) finger 子树里必须有挂 PhysicsCollisionAPI 的 prim，且这条 collider
            # 链能展开出 Mesh（直接 Mesh / instanced reference / 子树含 Mesh）。
            for finger in matches:
                sub = descendants_with_self(summary, finger)
                collider_prims = [prim for prim in sub if has_api(prim, "CollisionAPI")]
                working = [prim for prim in collider_prims if collider_yields_mesh(summary, prim)]
                if not collider_prims:
                    checks.append(Check("sim", "FAIL",
                        f"{name} ({finger.path}) 子树没有 PhysicsCollisionAPI",
                        "create_gripper_lab 在这条 link 上收集不到顶点；PhysX 也不会产生 contacts。"))
                elif not working:
                    paths = ", ".join(prim.path for prim in collider_prims)
                    checks.append(Check("sim", "FAIL",
                        f"{name} ({finger.path}) 的 CollisionAPI 没能展开到 Mesh",
                        f"找到的 collider: {paths}。usd_tools.get_prim_collision_mesh 只在"
                        " type=='Mesh' 时收顶点；prim 既不是 Mesh，也没有 reference 到含 Mesh"
                        " 的 prototype，子树里也没有 Mesh prim（Piper 早期把 API 挂在父 Xform"
                        " 上导致 finger contacts=0 就是这种）。"))
                else:
                    convex_hull = [
                        prim.name for prim in working
                        if prim.attrs.get("physics:approximation", "").strip('"') == "convexHull"
                    ]
                    if convex_hull:
                        checks.append(Check("guess", "RISK",
                            f"{name} 的 collider 使用 convexHull",
                            f"{', '.join(convex_hull)} approximation=convexHull；finger pad 凹面会被填平，"
                            "建议改为 convexDecomposition + shrinkWrap。"))

    # (e) base_frame 应放在原点（不含 translate / scale）
    if base_name:
        bases = body_prims_by_name_in_default(summary, base_name)
        base = bases[0] if bases else None
        if base is not None:
            translate = base.attrs.get("xformOp:translate")
            if translate:
                parsed = parse_translate(translate)
                if parsed and any(abs(v) > 1e-6 for v in parsed):
                    checks.append(Check("guess", "RISK",
                        "base_frame 不在原点",
                        f"xformOp:translate={translate}。所有 grasp pose 都相对 base，"
                        "base 上叠 translate 会让输出多一段隐藏 offset。"))

    # (f) mimic referenceJoint 必须真实存在且本身是 driven joint
    joints_by_name = {joint.name: joint for joint in joints}
    for joint in joints:
        for rel_name, rel_value in joint.rels.items():
            if "MimicJoint" not in rel_name:
                continue
            ref_name = short_path(rel_value)
            if ref_name in {"-", "None", ""}:
                checks.append(Check("sim", "FAIL",
                    f"{joint.name} mimic referenceJoint 为空",
                    f"{rel_name}={rel_value}；多连杆耦合无效。"))
                continue
            ref_joint = joints_by_name.get(ref_name)
            if ref_joint is None:
                checks.append(Check("sim", "FAIL",
                    f"{joint.name} mimic referenceJoint 找不到",
                    f"{rel_name}={rel_value}，本 USD 里没有 name='{ref_name}' 的 joint。"))
            elif not is_driven_joint(ref_joint):
                checks.append(Check("sim", "RISK",
                    f"{joint.name} mimic 引用的 joint 没有 DriveAPI",
                    f"referenceJoint='{ref_name}' 本身不是 driven joint；mimic 跟随的目标无人推动。"))

    # (g) driving joint 的 drive 强度阈值（防止 IK 不收敛 / 闭合无力）
    DRIVE_STIFFNESS_REVOLUTE_MIN = 1.0
    DRIVE_STIFFNESS_PRISMATIC_MIN = 100.0
    DRIVE_MAXFORCE_MIN = 1.0
    for joint in moving_joints:
        if not is_driven_joint(joint):
            continue
        is_prismatic = "Prismatic" in joint.type_name
        drives = driving_joint_drive_attrs(joint)
        stiff_keys = [k for k in drives if k.endswith(":stiffness")]
        force_keys = [k for k in drives if k.endswith(":maxForce")]
        stiff_min = DRIVE_STIFFNESS_PRISMATIC_MIN if is_prismatic else DRIVE_STIFFNESS_REVOLUTE_MIN
        if stiff_keys:
            stiff = max(drives[k] for k in stiff_keys)
            if stiff < stiff_min:
                checks.append(Check("sim", "RISK",
                    f"{joint.name} drive stiffness 偏低",
                    f"stiffness={stiff} < 经验阈值 {stiff_min}（"
                    f"{'prismatic' if is_prismatic else 'revolute'}）；IK 可能不收敛或闭合无力。"))
        if force_keys:
            max_force = max(drives[k] for k in force_keys)
            if max_force < DRIVE_MAXFORCE_MIN:
                checks.append(Check("sim", "RISK",
                    f"{joint.name} drive maxForce 偏低",
                    f"maxForce={max_force} < {DRIVE_MAXFORCE_MIN}；闭合力不足，物体在 tug 阶段会被扯出。"))

    # (h) Mesh 必须是三角面（运行时 usd_tools.get_prim_collision_mesh 会 raise）
    tri_errors, tri_skip = check_mesh_triangulation(summary.usd_path)
    if tri_skip:
        checks.append(Check("guess", "CHECK", "跳过三角面检查", tri_skip))
    elif tri_errors:
        for err in tri_errors[:5]:
            checks.append(Check("guess", "FAIL", "Mesh 含非三角面",
                f"{err}。usd_tools.get_prim_collision_mesh 在加载时会 raise ValueError。"))
        if len(tri_errors) > 5:
            checks.append(Check("guess", "FAIL", "更多非三角面 mesh",
                f"另外还有 {len(tri_errors) - 5} 个 Mesh 含非三角面。"))
    else:
        checks.append(Check("guess", "OK", "所有 Mesh 都是三角面",
            "通过 pxr.UsdGeom.Mesh.GetFaceVertexCountsAttr 校验。"))

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


def print_npz_cache_status(summary: UsdSummary) -> None:
    """提示 bots/<gripper>.npz 是否与 USD 时间戳一致。"""
    usd_path = summary.usd_path
    npz_path = os.path.splitext(usd_path)[0] + ".npz"
    if not os.path.exists(npz_path):
        console.print(Panel(
            f"{npz_path} 不存在；首次跑 create_gripper_lab 时会重新生成。",
            title="缓存状态 (bots/*.npz)",
            border_style="dim",
            box=box.ROUNDED,
        ))
        return
    try:
        usd_mtime = os.path.getmtime(usd_path)
        npz_mtime = os.path.getmtime(npz_path)
    except OSError as exc:  # noqa: BLE001
        console.print(Panel(f"无法读取时间戳: {exc}", title="缓存状态 (bots/*.npz)", border_style="yellow", box=box.ROUNDED))
        return
    delta = npz_mtime - usd_mtime
    if delta < 0:
        msg = (
            f"[bold red]缓存比 USD 旧[/bold red]（USD 比 .npz 新 {-delta:.0f} 秒）。\n"
            f"建议先 [bold]rm {npz_path}[/bold] 再跑 datagen，"
            "否则 Gripper.load 在 interpenetration 路径下可能用旧 npz（skip_config_validation=True）。"
        )
        style = "red"
    else:
        msg = (
            f"缓存比 USD 新（差 {delta:.0f} 秒），Gripper.load 会再做 config 比对。\n"
            f"[dim]{npz_path}[/dim]"
        )
        style = "green"
    console.print(Panel(msg, title="缓存状态 (bots/*.npz)", border_style=style, box=box.ROUNDED))


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

    print_npz_cache_status(summary)

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
