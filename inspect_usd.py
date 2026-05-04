#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
inspect_usd_zh.py

中文友好版 USD / Isaac Sim 机械夹爪结构查看器。
重点不是把所有属性暴力打印出来，而是把机械臂夹爪常见信息按“人能看懂”的方式重新组织：

1. 文件总体信息
2. 对象数量总览
3. Prim 层级树
4. 路径编号表
5. 刚体 / Link 表
6. 关节 / Joint 表
7. 碰撞体 / Collider 表
8. Mesh 几何表
9. 材质表
10. 诊断提示

用法：
    python inspect_usd_zh.py bots/franka_panda.usd
    python inspect_usd_zh.py bots/franka_panda.usd --root /panda_gripper
    python inspect_usd_zh.py bots/franka_panda.usd --detail /panda_gripper/joints/panda_finger_joint1
    python inspect_usd_zh.py bots/franka_panda.usd --show-render
    python inspect_usd_zh.py bots/franka_panda.usd --width 180

如果在 Isaac Sim 环境中：
    /path/to/isaac-sim/python.sh inspect_usd_zh.py bots/franka_panda.usd
"""

import argparse
import shutil
from collections import Counter
from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Optional, Tuple

from pxr import Usd, UsdGeom, UsdPhysics, Sdf, Gf

try:
    from pxr import UsdShade
except Exception:
    UsdShade = None

try:
    from pxr import PhysxSchema
except Exception:
    PhysxSchema = None

try:
    from rich.console import Console
    from rich.panel import Panel
    from rich.table import Table
    from rich.tree import Tree
    from rich import box
except ImportError:
    raise SystemExit(
        "缺少 rich，请先安装：\n"
        "  pip install rich\n"
        "如果你用 Isaac Sim 自带 Python：\n"
        "  /path/to/isaac-sim/python.sh -m pip install rich"
    )


# =============================================================================
# 基础工具函数
# =============================================================================

console: Console


def safe_get(fn, default=None):
    try:
        return fn()
    except Exception:
        return default


def fmt_value(v: Any, max_len: int = 80) -> str:
    """把 USD / Gf / Vt 类型转成适合表格显示的字符串。"""
    if v is None:
        return "—"
    if isinstance(v, Sdf.Path):
        return str(v)
    if isinstance(v, bool):
        return "是" if v else "否"
    if isinstance(v, float):
        if v == float("inf"):
            return "inf"
        if v == float("-inf"):
            return "-inf"
        return f"{v:.6g}"
    if isinstance(v, (int, str)):
        return str(v)
    if isinstance(v, (Gf.Vec2f, Gf.Vec2d, Gf.Vec2i)):
        return f"({fmt_value(v[0])}, {fmt_value(v[1])})"
    if isinstance(v, (Gf.Vec3f, Gf.Vec3d, Gf.Vec3i)):
        return f"({fmt_value(v[0])}, {fmt_value(v[1])}, {fmt_value(v[2])})"
    if isinstance(v, (Gf.Vec4f, Gf.Vec4d, Gf.Vec4i)):
        return f"({fmt_value(v[0])}, {fmt_value(v[1])}, {fmt_value(v[2])}, {fmt_value(v[3])})"
    if isinstance(v, (Gf.Quatf, Gf.Quatd)):
        imag = v.GetImaginary()
        return f"real={fmt_value(v.GetReal())}, imag=({fmt_value(imag[0])}, {fmt_value(imag[1])}, {fmt_value(imag[2])})"

    # Vt array 等序列类型，避免把大数组全部打印出来。
    try:
        if hasattr(v, "__len__") and not isinstance(v, dict):
            n = len(v)
            preview = []
            for i, item in enumerate(v):
                if i >= 4:
                    break
                preview.append(fmt_value(item, max_len=30))
            s = f"{type(v).__name__}(长度={n}, 预览={preview})"
            return s if len(s) <= max_len else s[:max_len] + "…"
    except Exception:
        pass

    s = str(v)
    return s if len(s) <= max_len else s[:max_len] + "…"


def yes_no(v: Any) -> str:
    if v is None:
        return "—"
    return "[green]是[/green]" if bool(v) else "[dim]否[/dim]"


def type_name(prim: Usd.Prim) -> str:
    t = str(prim.GetTypeName())
    return t if t else "Untyped"


def type_zh(t: str) -> str:
    mapping = {
        "Xform": "坐标变换节点 / 也常当作 link 容器",
        "Scope": "分组文件夹，不表示实体几何",
        "Mesh": "真实三角网格几何",
        "Material": "材质",
        "Shader": "材质着色器",
        "PhysicsPrismaticJoint": "移动关节 / 直线滑动关节",
        "PhysicsRevoluteJoint": "转动关节 / 旋转关节",
        "PhysicsFixedJoint": "固定关节 / 焊死连接",
        "PhysicsJoint": "物理关节",
        "RenderProduct": "渲染输出配置，可忽略",
        "RenderSettings": "渲染设置，可忽略",
        "RenderVar": "渲染变量，可忽略",
        "Untyped": "无类型 Prim / 通常是容器或配置节点",
    }
    return mapping.get(t, "USD 类型")


def schema_zh(schema: str) -> str:
    mapping = {
        "PhysicsRigidBodyAPI": "刚体属性：此 Prim 参与物理仿真",
        "PhysicsMassAPI": "质量属性：质量、密度、质心、惯量",
        "PhysicsCollisionAPI": "碰撞属性：此 Prim 可参与碰撞检测",
        "PhysicsMeshCollisionAPI": "网格碰撞属性：碰撞几何按网格/凸包等方式近似",
        "PhysicsDriveAPI:linear": "线性驱动：用于驱动移动关节",
        "PhysicsDriveAPI:angular": "角度驱动：用于驱动转动关节",
        "PhysicsArticulationRootAPI": "关节系统根：机器人 articulation 的根节点",
        "MaterialBindingAPI": "材质绑定：这个 Prim 绑定了材质",
        "NodeDefAPI": "着色器节点定义",
    }
    return mapping.get(schema, schema)


def get_schemas(prim: Usd.Prim) -> List[str]:
    return [str(s) for s in safe_get(lambda: prim.GetAppliedSchemas(), [])]


def get_schemas_zh(prim: Usd.Prim) -> str:
    schemas = get_schemas(prim)
    if not schemas:
        return "—"
    return "\n".join(f"{s}：{schema_zh(s)}" for s in schemas)


def is_joint(prim: Usd.Prim) -> bool:
    joint_type_names = {
        "PhysicsJoint",
        "PhysicsFixedJoint",
        "PhysicsRevoluteJoint",
        "PhysicsPrismaticJoint",
        "PhysicsSphericalJoint",
        "PhysicsDistanceJoint",
    }
    return type_name(prim) in joint_type_names or bool(safe_get(lambda: prim.IsA(UsdPhysics.Joint), False))


def has_api(prim: Usd.Prim, api) -> bool:
    return bool(safe_get(lambda: prim.HasAPI(api), False))


def is_rigid_body(prim: Usd.Prim) -> bool:
    return has_api(prim, UsdPhysics.RigidBodyAPI)


def is_collider(prim: Usd.Prim) -> bool:
    return has_api(prim, UsdPhysics.CollisionAPI)


def is_mesh(prim: Usd.Prim) -> bool:
    return bool(safe_get(lambda: prim.IsA(UsdGeom.Mesh), False))


def is_material(prim: Usd.Prim) -> bool:
    return type_name(prim) == "Material"


def world_xyz(prim: Usd.Prim) -> str:
    if not prim.IsA(UsdGeom.Xformable):
        return "—"
    try:
        cache = UsdGeom.XformCache(Usd.TimeCode.Default())
        m = cache.GetLocalToWorldTransform(prim)
        t = m.ExtractTranslation()
        return f"({t[0]:.5g}, {t[1]:.5g}, {t[2]:.5g})"
    except Exception as e:
        return f"读取失败：{e}"


def xform_ops(prim: Usd.Prim) -> str:
    if not prim.IsA(UsdGeom.Xformable):
        return "—"
    xf = UsdGeom.Xformable(prim)
    ops = xf.GetOrderedXformOps()
    if not ops:
        return "—"
    lines = []
    for op in ops:
        value = safe_get(lambda op=op: op.Get(), None)
        lines.append(f"{op.GetOpName()} = {fmt_value(value, max_len=120)}")
    return "\n".join(lines)


def relationship_targets(rel) -> List[str]:
    return [str(x) for x in safe_get(lambda: rel.GetTargets(), [])]


def short_name(path: str) -> str:
    return path.rstrip("/").split("/")[-1] if path != "/" else "/"


def should_hide_render_prim(prim: Usd.Prim) -> bool:
    p = str(prim.GetPath())
    return p == "/Render" or p.startswith("/Render/")


def collider_parent_body(prim: Usd.Prim) -> Optional[str]:
    p = prim.GetParent()
    while p and p.IsValid() and str(p.GetPath()) != "/":
        if is_rigid_body(p):
            return str(p.GetPath())
        p = p.GetParent()
    return None


def collider_approximation(prim: Usd.Prim) -> str:
    if has_api(prim, UsdPhysics.MeshCollisionAPI):
        api = UsdPhysics.MeshCollisionAPI(prim)
        return fmt_value(safe_get(lambda: api.GetApproximationAttr().Get(), None))
    return "—"


def approx_zh(approx: str) -> str:
    mapping = {
        "convexHull": "单个凸包；速度快，但形状较粗略",
        "convexDecomposition": "凸分解；用多个凸体近似，通常比单凸包更贴合",
        "meshSimplification": "简化三角网格；动态刚体上可能不合适",
        "none": "无近似 / 原始三角网格；动态刚体上常有问题",
        "sdf": "SDF 碰撞；适合较复杂形状，但生成/计算更重",
        "—": "没有读到网格碰撞近似属性",
    }
    return mapping.get(approx, "碰撞几何近似方式")


def joint_type_zh(t: str) -> str:
    mapping = {
        "PhysicsPrismaticJoint": "移动关节：body1 相对 body0 沿指定轴直线滑动",
        "PhysicsRevoluteJoint": "转动关节：body1 相对 body0 绕指定轴旋转",
        "PhysicsFixedJoint": "固定关节：两个刚体之间没有相对运动",
        "PhysicsSphericalJoint": "球关节：类似万向连接",
        "PhysicsDistanceJoint": "距离约束关节",
    }
    return mapping.get(t, "物理关节")


def limit_unit_for_joint(t: str, meters_per_unit: float) -> str:
    if t == "PhysicsPrismaticJoint":
        if meters_per_unit == 1.0:
            return "米"
        return f"stage单位；1单位={meters_per_unit:g}米"
    if t == "PhysicsRevoluteJoint":
        return "度或角度单位，取决于 USD/PhysX 属性约定"
    return "—"


@dataclass
class IdIndex:
    path_to_id: Dict[str, str]
    id_to_path: Dict[str, str]

    def get(self, path_or_none: Optional[str]) -> str:
        if not path_or_none:
            return "—"
        return self.path_to_id.get(path_or_none, path_or_none)

    def label(self, path_or_none: Optional[str]) -> str:
        if not path_or_none:
            return "—"
        sid = self.get(path_or_none)
        return f"{sid} {short_name(path_or_none)}" if sid != path_or_none else path_or_none


def build_index(prims: Iterable[Usd.Prim]) -> IdIndex:
    path_to_id: Dict[str, str] = {}
    id_to_path: Dict[str, str] = {}
    counters = Counter()

    def add(path: str, prefix: str):
        if path in path_to_id:
            return
        counters[prefix] += 1
        sid = f"{prefix}{counters[prefix]:02d}"
        path_to_id[path] = sid
        id_to_path[sid] = path

    for p in prims:
        path = str(p.GetPath())
        if is_rigid_body(p):
            add(path, "B")  # Body
    for p in prims:
        path = str(p.GetPath())
        if is_joint(p):
            add(path, "J")  # Joint
    for p in prims:
        path = str(p.GetPath())
        if is_collider(p):
            add(path, "C")  # Collider
    for p in prims:
        path = str(p.GetPath())
        if is_mesh(p):
            add(path, "M")  # Mesh
    for p in prims:
        path = str(p.GetPath())
        if is_material(p):
            add(path, "T")  # Material
    for p in prims:
        add(str(p.GetPath()), "P")

    return IdIndex(path_to_id, id_to_path)


# =============================================================================
# Prim 收集与解析
# =============================================================================


def collect_prims(stage: Usd.Stage, root_path: str, show_render: bool, all_prims: bool) -> List[Usd.Prim]:
    root = stage.GetPseudoRoot() if root_path == "/" else stage.GetPrimAtPath(root_path)
    if not root or not root.IsValid():
        raise RuntimeError(f"找不到 root prim：{root_path}")

    if all_prims:
        prims = list(Usd.PrimRange.AllPrims(root))
    elif root_path == "/":
        prims = list(stage.Traverse())
    else:
        prims = list(Usd.PrimRange(root))

    if not show_render:
        prims = [p for p in prims if not should_hide_render_prim(p)]

    return prims


def get_joint_data(prim: Usd.Prim, index: IdIndex, meters_per_unit: float) -> Dict[str, str]:
    t = type_name(prim)
    joint = UsdPhysics.Joint(prim)
    body0_targets = relationship_targets(joint.GetBody0Rel())
    body1_targets = relationship_targets(joint.GetBody1Rel())
    body0 = body0_targets[0] if body0_targets else None
    body1 = body1_targets[0] if body1_targets else None

    axis = "—"
    lower = "—"
    upper = "—"

    if prim.IsA(UsdPhysics.PrismaticJoint):
        j = UsdPhysics.PrismaticJoint(prim)
        axis = fmt_value(safe_get(lambda: j.GetAxisAttr().Get(), None))
        lower = fmt_value(safe_get(lambda: j.GetLowerLimitAttr().Get(), None))
        upper = fmt_value(safe_get(lambda: j.GetUpperLimitAttr().Get(), None))
    elif prim.IsA(UsdPhysics.RevoluteJoint):
        j = UsdPhysics.RevoluteJoint(prim)
        axis = fmt_value(safe_get(lambda: j.GetAxisAttr().Get(), None))
        lower = fmt_value(safe_get(lambda: j.GetLowerLimitAttr().Get(), None))
        upper = fmt_value(safe_get(lambda: j.GetUpperLimitAttr().Get(), None))

    drives = []
    for drive_name in ["linear", "angular", "rotX", "rotY", "rotZ", "transX", "transY", "transZ"]:
        d = safe_get(lambda name=drive_name: UsdPhysics.DriveAPI.Get(prim, name), None)
        if d and d.GetPrim().IsValid():
            drive_type = fmt_value(safe_get(lambda: d.GetTypeAttr().Get(), None))
            max_force = fmt_value(safe_get(lambda: d.GetMaxForceAttr().Get(), None))
            target_pos = fmt_value(safe_get(lambda: d.GetTargetPositionAttr().Get(), None))
            target_vel = fmt_value(safe_get(lambda: d.GetTargetVelocityAttr().Get(), None))
            stiffness = fmt_value(safe_get(lambda: d.GetStiffnessAttr().Get(), None))
            damping = fmt_value(safe_get(lambda: d.GetDampingAttr().Get(), None))
            drives.append(
                f"{drive_name} 驱动\n"
                f"  类型={drive_type}\n"
                f"  最大力/力矩={max_force}\n"
                f"  目标位置={target_pos}\n"
                f"  目标速度={target_vel}\n"
                f"  刚度k={stiffness}\n"
                f"  阻尼d={damping}"
            )

    return {
        "id": index.get(str(prim.GetPath())),
        "path": str(prim.GetPath()),
        "name": prim.GetName(),
        "type": t,
        "type_zh": joint_type_zh(t),
        "body0": index.label(body0),
        "body1": index.label(body1),
        "axis": axis,
        "lower": lower,
        "upper": upper,
        "unit": limit_unit_for_joint(t, meters_per_unit),
        "drive": "\n".join(drives) if drives else "无驱动 DriveAPI",
        "schemas": get_schemas_zh(prim),
    }


def get_body_data(prim: Usd.Prim, index: IdIndex) -> Dict[str, str]:
    rb = UsdPhysics.RigidBodyAPI(prim)
    mass = UsdPhysics.MassAPI(prim)
    enabled = safe_get(lambda: rb.GetRigidBodyEnabledAttr().Get(), None)
    kinematic = safe_get(lambda: rb.GetKinematicEnabledAttr().Get(), None)
    mass_v = safe_get(lambda: mass.GetMassAttr().Get(), None)
    density_v = safe_get(lambda: mass.GetDensityAttr().Get(), None)
    com_v = safe_get(lambda: mass.GetCenterOfMassAttr().Get(), None)
    inertia_v = safe_get(lambda: mass.GetDiagonalInertiaAttr().Get(), None)

    mass_note = ""
    if mass_v in (None, 0, 0.0):
        mass_note = "未设置有效质量或读到0，仿真前建议确认"
    else:
        mass_note = "显式质量"

    com_note = ""
    com_s = fmt_value(com_v)
    if "-inf" in com_s or "inf" in com_s:
        com_note = "质心没有有效设置"
    elif com_s == "—":
        com_note = "未显式设置质心"
    else:
        com_note = "显式质心"

    return {
        "id": index.get(str(prim.GetPath())),
        "path": str(prim.GetPath()),
        "name": prim.GetName(),
        "type": type_name(prim),
        "enabled": yes_no(enabled),
        "kinematic": yes_no(kinematic),
        "motion_mode": "运动学刚体：由程序/动画直接控制" if bool(kinematic) else "动力学刚体：由物理引擎积分运动",
        "mass": fmt_value(mass_v),
        "density": fmt_value(density_v),
        "com": com_s,
        "inertia": fmt_value(inertia_v),
        "mass_note": mass_note,
        "com_note": com_note,
        "world": world_xyz(prim),
        "schemas": get_schemas_zh(prim),
    }


def get_collider_data(prim: Usd.Prim, index: IdIndex) -> Dict[str, str]:
    col = UsdPhysics.CollisionAPI(prim)
    enabled = safe_get(lambda: col.GetCollisionEnabledAttr().Get(), None)
    parent = collider_parent_body(prim)
    approx = collider_approximation(prim)
    return {
        "id": index.get(str(prim.GetPath())),
        "path": str(prim.GetPath()),
        "name": prim.GetName(),
        "type": type_name(prim),
        "parent": index.label(parent),
        "enabled": yes_no(enabled),
        "approx": approx,
        "approx_zh": approx_zh(approx),
        "world": world_xyz(prim),
        "schemas": get_schemas_zh(prim),
    }


def get_mesh_data(prim: Usd.Prim, index: IdIndex) -> Dict[str, str]:
    mesh = UsdGeom.Mesh(prim)
    points = safe_get(lambda: mesh.GetPointsAttr().Get(), None)
    counts = safe_get(lambda: mesh.GetFaceVertexCountsAttr().Get(), None)
    indices = safe_get(lambda: mesh.GetFaceVertexIndicesAttr().Get(), None)
    normals = safe_get(lambda: mesh.GetNormalsAttr().Get(), None)
    return {
        "id": index.get(str(prim.GetPath())),
        "path": str(prim.GetPath()),
        "name": prim.GetName(),
        "points": str(len(points)) if points is not None else "0",
        "faces": str(len(counts)) if counts is not None else "0",
        "indices": str(len(indices)) if indices is not None else "0",
        "normals": str(len(normals)) if normals is not None else "0",
        "subdivision": fmt_value(safe_get(lambda: mesh.GetSubdivisionSchemeAttr().Get(), None)),
        "double_sided": yes_no(safe_get(lambda: mesh.GetDoubleSidedAttr().Get(), None)),
        "world": world_xyz(prim),
    }


# =============================================================================
# 打印模块
# =============================================================================


def print_stage_info(stage: Usd.Stage, usd_file: str, root_path: str):
    default = stage.GetDefaultPrim()
    default_path = str(default.GetPath()) if default and default.IsValid() else "—"
    data = [
        ("文件", usd_file, "当前打开的 USD 文件路径"),
        ("查看根节点", root_path, "只分析这个 Prim 下面的内容；/ 表示整份文件"),
        ("默认 Prim", default_path, "别人引用这个 USD 时默认使用的根对象，机器人文件一般就是机器人根节点"),
        ("上方向 Up Axis", UsdGeom.GetStageUpAxis(stage), "世界坐标里哪个轴是“上”。Z 表示 Z 轴向上，Isaac Sim 常用 Z-up"),
        ("单位 Meters Per Unit", fmt_value(UsdGeom.GetStageMetersPerUnit(stage)), "1 个 stage 单位等于多少米；1.0 表示坐标/长度单位就是米"),
        ("FPS", fmt_value(stage.GetFramesPerSecond()), "动画播放帧率；静态机器人结构通常不用太关心"),
        ("TCPS", fmt_value(stage.GetTimeCodesPerSecond()), "timeCode 到秒的换算；静态结构通常不用太关心"),
    ]
    table = Table(title="① 文件总体信息", box=box.ROUNDED, expand=True)
    table.add_column("项目", style="cyan", ratio=2)
    table.add_column("值", style="bold", ratio=3)
    table.add_column("这代表什么", ratio=6)
    for row in data:
        table.add_row(*map(str, row))
    console.print(table)


def print_summary(prims: List[Usd.Prim]):
    bodies = [p for p in prims if is_rigid_body(p)]
    joints = [p for p in prims if is_joint(p)]
    colliders = [p for p in prims if is_collider(p)]
    meshes = [p for p in prims if is_mesh(p)]
    materials = [p for p in prims if is_material(p)]

    table = Table(title="② 夹爪结构总览", box=box.ROUNDED, expand=True)
    table.add_column("类别", style="cyan", ratio=2)
    table.add_column("数量", justify="right", style="bold", ratio=1)
    table.add_column("中文解释", ratio=5)
    table.add_column("怎么看", ratio=5)

    table.add_row("Prim 节点", str(len(prims)), "USD 里的节点总数。Prim 可以是几何、关节、材质、容器、渲染配置等。", "数量多不一定复杂；要看里面有多少刚体、关节、Mesh。")
    table.add_row("刚体 RigidBody", str(len(bodies)), "参与物理仿真的实体。机械夹爪通常 hand、left finger、right finger 都是刚体。", "夹爪能不能动，主要看刚体和关节是否设置正确。")
    table.add_row("关节 Joint", str(len(joints)), "连接两个刚体的约束，比如移动关节、旋转关节、固定关节。", "夹爪开合通常靠 PrismaticJoint 或 RevoluteJoint。")
    table.add_row("碰撞体 Collider", str(len(colliders)), "用于物理碰撞检测的形状，不一定等同于视觉模型。", "抓取仿真是否靠谱，很依赖 collider 是否存在且形状合理。")
    table.add_row("网格 Mesh", str(len(meshes)), "真实三角网格几何，通常用于显示或碰撞。", "如果为 0，说明这个文件里没有直接写入几何网格。")
    table.add_row("材质 Material", str(len(materials)), "颜色、贴图、shader 等外观信息。", "对物理运动影响通常不大。")
    console.print(table)

    counts = Counter(type_name(p) for p in prims)
    type_table = Table(title="③ Prim 类型统计", box=box.SIMPLE_HEAVY, expand=True)
    type_table.add_column("USD 类型", style="cyan", ratio=2)
    type_table.add_column("数量", justify="right", ratio=1)
    type_table.add_column("中文含义", ratio=5)
    for t, c in sorted(counts.items(), key=lambda x: x[0]):
        type_table.add_row(t, str(c), type_zh(t))
    console.print(type_table)


def print_tree(prims: List[Usd.Prim]):
    root_tree = Tree("[bold]/[/bold]")
    nodes = {"/": root_tree}
    for prim in prims:
        path = str(prim.GetPath())
        if path == "/":
            continue
        parent_path = str(prim.GetParent().GetPath())
        parent_node = nodes.get(parent_path, root_tree)

        tags = []
        if is_rigid_body(prim):
            tags.append("[green]刚体[/green]")
        if is_collider(prim):
            tags.append("[yellow]碰撞体[/yellow]")
        if is_joint(prim):
            tags.append("[magenta]关节[/magenta]")
        if is_mesh(prim):
            tags.append("[blue]网格[/blue]")
        if is_material(prim):
            tags.append("[cyan]材质[/cyan]")
        tag = " ".join(tags)
        label = f"{prim.GetName()} [dim]<{type_name(prim)}>[/dim] {tag}"
        nodes[path] = parent_node.add(label)

    console.print(Panel(root_tree, title="④ Prim 层级树：看夹爪由哪些节点组成", box=box.ROUNDED))
    console.print(Panel(
        "读法：缩进表示父子关系。比如 leftfinger 下面的 collisions 表示碰撞体挂在左手指下面。\n"
        "标签含义：刚体=参与物理仿真；关节=连接两个刚体；碰撞体=用于碰撞检测；网格=真实几何；材质=外观。",
        title="层级树说明",
        box=box.SIMPLE,
    ))


def print_path_index(index: IdIndex):
    table = Table(title="⑤ 路径编号表：后面的表用短编号，避免路径太长被挤成省略号", box=box.ROUNDED, expand=True)
    table.add_column("编号", style="bold cyan", ratio=1)
    table.add_column("类别", ratio=2)
    table.add_column("完整 USD 路径", ratio=8, overflow="fold")
    for sid, path in sorted(index.id_to_path.items(), key=lambda x: (x[0][0], int(x[0][1:]) if x[0][1:].isdigit() else 999)):
        prefix = sid[0]
        category = {
            "B": "刚体 Body/Link",
            "J": "关节 Joint",
            "C": "碰撞体 Collider",
            "M": "网格 Mesh",
            "T": "材质 Material",
            "P": "普通 Prim",
        }.get(prefix, "Prim")
        table.add_row(sid, category, path)
    console.print(table)


def print_bodies(prims: List[Usd.Prim], index: IdIndex):
    bodies = [p for p in prims if is_rigid_body(p)]
    table = Table(title="⑥ 刚体 / Link 信息：哪些部件会参与物理仿真", box=box.ROUNDED, expand=True)
    table.add_column("编号", style="bold green", ratio=1)
    table.add_column("名称", ratio=2)
    table.add_column("是否启用", ratio=1)
    table.add_column("控制方式", ratio=3)
    table.add_column("质量", ratio=1)
    table.add_column("质心", ratio=2)
    table.add_column("世界坐标", ratio=2)
    table.add_column("解释/提醒", ratio=5, overflow="fold")

    if not bodies:
        table.add_row("—", "没有刚体", "—", "—", "—", "—", "—", "这个文件里没有 PhysicsRigidBodyAPI。")
    else:
        for p in bodies:
            d = get_body_data(p, index)
            note = f"{d['mass_note']}；{d['com_note']}。Schema: {d['schemas']}"
            table.add_row(d["id"], d["name"], d["enabled"], d["motion_mode"], d["mass"], d["com"], d["world"], note)

    console.print(table)
    console.print(Panel(
        "字段说明：\n"
        "• 刚体/Link：机器人里能被物理引擎当作一个整体运动的部件。\n"
        "• 是否启用：否的话这个刚体不会参与正常刚体仿真。\n"
        "• 控制方式：kinematic=是 表示主要由程序直接移动；kinematic=否 表示由物理引擎根据力、关节、碰撞来算。\n"
        "• 质量/质心：抓取仿真里很重要；如果读到 0 或 -inf，说明质量属性可能没有有效设置。",
        title="刚体表怎么看",
        box=box.SIMPLE,
    ))


def print_joints(prims: List[Usd.Prim], index: IdIndex, meters_per_unit: float):
    joints = [p for p in prims if is_joint(p)]
    table = Table(title="⑦ 关节 / Joint 信息：夹爪怎样开合", box=box.ROUNDED, expand=True)
    table.add_column("编号", style="bold magenta", ratio=1)
    table.add_column("名称", ratio=2)
    table.add_column("关节类型", ratio=3)
    table.add_column("连接关系", ratio=4, overflow="fold")
    table.add_column("运动轴", ratio=1)
    table.add_column("运动范围", ratio=3)
    table.add_column("驱动", ratio=4, overflow="fold")
    table.add_column("新手解释", ratio=5, overflow="fold")

    if not joints:
        table.add_row("—", "没有关节", "—", "—", "—", "—", "—", "这个文件没有物理关节。")
    else:
        for p in joints:
            d = get_joint_data(p, index, meters_per_unit)
            relation = f"body0: {d['body0']}\nbody1: {d['body1']}"
            limit = f"下限 {d['lower']}\n上限 {d['upper']}\n单位 {d['unit']}"
            drive = d["drive"]
            if drive == "无驱动 DriveAPI" and d["type"] in {"PhysicsPrismaticJoint", "PhysicsRevoluteJoint"}:
                drive = "[yellow]无驱动 DriveAPI[/yellow]"
            explain = d["type_zh"]
            if d["type"] == "PhysicsPrismaticJoint":
                explain += "。对 Panda 夹爪来说，finger joint 通常控制手指沿某个方向平移开合。"
            if d["drive"].startswith("无驱动") and d["type"] in {"PhysicsPrismaticJoint", "PhysicsRevoluteJoint"}:
                explain += " 当前没有 drive，除非有 mimic/coupling 或外部控制，否则它可能不会主动跟随。"
            table.add_row(d["id"], d["name"], f"{d['type']}\n{d['type_zh']}", relation, d["axis"], limit, drive, explain)

    console.print(table)
    console.print(Panel(
        "字段说明：\n"
        "• body0/body1：关节连接的两个刚体。一般可以理解为 body1 相对 body0 运动。\n"
        "• 运动轴 Axis：PrismaticJoint 沿这个轴平移；RevoluteJoint 绕这个轴旋转。\n"
        "• 下限/上限：关节能运动的范围。移动关节通常是长度；metersPerUnit=1 时就是米。\n"
        "• Drive：驱动器，相当于关节电机/控制器。targetPosition 是目标位置，maxForce 是最大输出力或力矩，stiffness/damping 是类似 PD 控制的刚度和阻尼。",
        title="关节表怎么看",
        box=box.SIMPLE,
    ))


def print_colliders(prims: List[Usd.Prim], index: IdIndex):
    cols = [p for p in prims if is_collider(p)]
    table = Table(title="⑧ 碰撞体 / Collider 信息：仿真抓取时真正用于接触检测的形状", box=box.ROUNDED, expand=True)
    table.add_column("编号", style="bold yellow", ratio=1)
    table.add_column("名称", ratio=2)
    table.add_column("挂在哪个刚体下", ratio=3)
    table.add_column("是否启用", ratio=1)
    table.add_column("碰撞近似", ratio=3)
    table.add_column("世界坐标", ratio=2)
    table.add_column("解释/提醒", ratio=6, overflow="fold")

    if not cols:
        table.add_row("—", "没有碰撞体", "—", "—", "—", "—", "没有 CollisionAPI；仿真时可能无法和物体接触。")
    else:
        for p in cols:
            d = get_collider_data(p, index)
            note = f"{d['approx_zh']}。Schema: {d['schemas']}"
            if d["type"] != "Mesh":
                note += " 注意：CollisionAPI 挂在 Xform/非 Mesh 上不一定错，但要确认它下面或引用里确实有几何。"
            table.add_row(d["id"], d["name"], d["parent"], d["enabled"], d["approx"], d["world"], note)

    console.print(table)
    console.print(Panel(
        "字段说明：\n"
        "• Collider 不一定等于你看到的外观模型；它是物理引擎用来判断接触/碰撞的形状。\n"
        "• convexHull：用一个凸壳包住物体，快但粗。\n"
        "• convexDecomposition：把物体拆成多个凸体，更贴近真实形状。\n"
        "• 动态刚体通常不建议直接用复杂三角网格作为 collider，容易慢或报 PhysX 警告。",
        title="碰撞体表怎么看",
        box=box.SIMPLE,
    ))


def print_meshes(prims: List[Usd.Prim], index: IdIndex):
    meshes = [p for p in prims if is_mesh(p)]
    table = Table(title="⑨ Mesh 几何信息：真实网格是否在这个 USD 里", box=box.ROUNDED, expand=True)
    table.add_column("编号", style="bold blue", ratio=1)
    table.add_column("名称", ratio=3)
    table.add_column("顶点数", justify="right", ratio=1)
    table.add_column("面数", justify="right", ratio=1)
    table.add_column("索引数", justify="right", ratio=1)
    table.add_column("法线数", justify="right", ratio=1)
    table.add_column("细分模式", ratio=2)
    table.add_column("解释", ratio=5, overflow="fold")

    if not meshes:
        table.add_row("—", "[yellow]没有 Mesh prim[/yellow]", "—", "—", "—", "—", "—", "这个 USD 文件里没有直接写入三角网格。visuals/collisions 可能只是空 Xform，或者几何在外部引用/payload 里。")
    else:
        for p in meshes:
            d = get_mesh_data(p, index)
            explain = "顶点/面越多，显示越精细；如果这是碰撞网格，太复杂会拖慢物理仿真。"
            table.add_row(d["id"], d["name"], d["points"], d["faces"], d["indices"], d["normals"], d["subdivision"], explain)
    console.print(table)


def print_materials(prims: List[Usd.Prim], index: IdIndex):
    mats = [p for p in prims if is_material(p)]
    table = Table(title="⑩ 材质信息：外观颜色/贴图/shader", box=box.ROUNDED, expand=True)
    table.add_column("编号", style="bold cyan", ratio=1)
    table.add_column("材质名", ratio=3)
    table.add_column("Shader 子节点数", justify="right", ratio=1)
    table.add_column("说明", ratio=5, overflow="fold")
    if not mats:
        table.add_row("—", "没有材质", "—", "这个 USD 没有 Material prim。")
    else:
        for p in mats:
            children = list(p.GetChildren())
            table.add_row(index.get(str(p.GetPath())), p.GetName(), str(len(children)), "材质主要影响显示外观，一般不影响夹爪的物理运动。")
    console.print(table)


def print_diagnostics(prims: List[Usd.Prim], index: IdIndex):
    meshes = [p for p in prims if is_mesh(p)]
    joints = [p for p in prims if is_joint(p)]
    cols = [p for p in prims if is_collider(p)]
    bodies = [p for p in prims if is_rigid_body(p)]

    lines = []

    if not meshes:
        lines.append(
            "[yellow]1. 没有发现 Mesh prim。[/yellow]\n"
            "这表示当前文件里没有直接存放真实三角网格。你的 visuals/collisions 节点可能只是 Xform 容器。\n"
            "建议下一步：用 `usdcat --flatten 文件.usd > flat.usda` 后搜索 Mesh，或者检查 reference/payload 是否指向外部几何文件。"
        )

    for j in joints:
        d = get_joint_data(j, index, 1.0)
        if d["drive"] == "无驱动 DriveAPI" and d["type"] in {"PhysicsPrismaticJoint", "PhysicsRevoluteJoint"}:
            lines.append(
                f"[yellow]2. 关节 {d['id']} {d['name']} 没有 DriveAPI。[/yellow]\n"
                "如果它是被动跟随关节，这可能没问题；但如果你希望它主动运动，需要添加 drive 或 mimic/coupling 关系。"
            )

    for c in cols:
        if type_name(c) != "Mesh":
            lines.append(
                f"[yellow]3. 碰撞体 {index.label(str(c.GetPath()))} 是 {type_name(c)}，不是 Mesh。[/yellow]\n"
                "这不一定错，但需要确认碰撞几何是否真的存在。否则 Isaac Sim 里可能看起来有 collision API，但没有实际碰撞形状。"
            )

    for b in bodies:
        child_cols = [c for c in cols if collider_parent_body(c) == str(b.GetPath())]
        if not child_cols:
            lines.append(
                f"[yellow]4. 刚体 {index.label(str(b.GetPath()))} 下面没有找到碰撞体。[/yellow]\n"
                "这个刚体可能无法与环境/物体发生接触碰撞。"
            )

    if not lines:
        lines.append("[green]没有发现明显结构问题。[/green]")

    console.print(Panel("\n\n".join(lines), title="⑪ 自动诊断：这个夹爪 USD 可能需要注意什么", box=box.ROUNDED))


def print_detail(stage: Usd.Stage, path: str, index: IdIndex):
    prim = stage.GetPrimAtPath(path)
    if not prim or not prim.IsValid():
        console.print(Panel(f"找不到 Prim：{path}", title="Prim 详情", style="red"))
        return

    title = f"Prim 详情：{index.label(path)}"
    basic = Table(title=title, box=box.ROUNDED, expand=True)
    basic.add_column("项目", style="cyan", ratio=2)
    basic.add_column("值", ratio=4, overflow="fold")
    basic.add_column("中文解释", ratio=5, overflow="fold")

    basic.add_row("完整路径", str(prim.GetPath()), "这个节点在 USD 层级树中的唯一地址。")
    basic.add_row("名称", prim.GetName(), "路径最后一段。")
    basic.add_row("USD 类型", type_name(prim), type_zh(type_name(prim)))
    basic.add_row("是否 active", yes_no(prim.IsActive()), "inactive 的 Prim 通常不会出现在最终 stage 中。")
    basic.add_row("是否 defined", yes_no(prim.IsDefined()), "defined 表示这个 Prim 在某个 layer 中有定义。")
    basic.add_row("是否 loaded", yes_no(prim.IsLoaded()), "payload 相关；未 loaded 时外部资源可能没展开。")
    basic.add_row("世界坐标", world_xyz(prim), "该 Prim 当前相对世界坐标系的位置。")
    basic.add_row("应用的 API/Schemas", get_schemas_zh(prim), "这些 API 决定它是否是刚体、碰撞体、关节驱动等。")
    console.print(basic)

    xf = Table(title="局部 Transform Ops", box=box.SIMPLE_HEAVY, expand=True)
    xf.add_column("xformOp", ratio=4, overflow="fold")
    xf.add_column("含义", ratio=5, overflow="fold")
    xf.add_row(xform_ops(prim), "这个 Prim 自己相对父节点的平移/旋转/缩放等变换。")
    console.print(xf)

    attrs = Table(title="属性 Attributes", box=box.SIMPLE_HEAVY, expand=True)
    attrs.add_column("属性名", style="cyan", ratio=3)
    attrs.add_column("值", ratio=5, overflow="fold")
    attrs.add_column("解释", ratio=4, overflow="fold")
    attr_list = prim.GetAttributes()
    if not attr_list:
        attrs.add_row("—", "—", "这个 Prim 没有可读属性。")
    else:
        for attr in attr_list:
            name = attr.GetName()
            val = fmt_value(safe_get(lambda a=attr: a.Get(), None), max_len=200)
            explain = explain_attr(name)
            attrs.add_row(name, val, explain)
    console.print(attrs)

    rels = Table(title="关系 Relationships", box=box.SIMPLE_HEAVY, expand=True)
    rels.add_column("关系名", style="cyan", ratio=3)
    rels.add_column("目标", ratio=5, overflow="fold")
    rels.add_column("解释", ratio=4, overflow="fold")
    rel_list = prim.GetRelationships()
    if not rel_list:
        rels.add_row("—", "—", "这个 Prim 没有关系目标。")
    else:
        for rel in rel_list:
            rels.add_row(rel.GetName(), "\n".join(relationship_targets(rel)) or "—", explain_rel(rel.GetName()))
    console.print(rels)


def explain_attr(name: str) -> str:
    n = name.lower()
    if "xformop" in n:
        return "局部坐标变换，比如平移、旋转、缩放。"
    if "physics:axis" in n or n.endswith("axis"):
        return "关节运动轴；移动关节沿它滑动，转动关节绕它旋转。"
    if "lowerlimit" in n:
        return "关节下限，限制最小运动位置/角度。"
    if "upperlimit" in n:
        return "关节上限，限制最大运动位置/角度。"
    if "drivetype" in n or "drive:type" in n:
        return "驱动类型，例如 force 表示力/力矩驱动。"
    if "targetposition" in n:
        return "驱动器希望关节到达的目标位置。"
    if "targetvelocity" in n:
        return "驱动器希望关节达到的目标速度。"
    if "maxforce" in n:
        return "驱动器最大能输出的力或力矩。"
    if "stiffness" in n:
        return "刚度，类似 PD 控制里的 P；越大越强烈拉向目标位置。"
    if "damping" in n:
        return "阻尼，类似 PD 控制里的 D；用于抑制振荡。"
    if "collisionenabled" in n:
        return "是否启用碰撞检测。"
    if "approximation" in n:
        return "碰撞几何近似方式，例如 convexHull 或 convexDecomposition。"
    if "mass" in n:
        return "质量相关属性。"
    if "density" in n:
        return "密度相关属性。"
    if "centerofmass" in n:
        return "质心位置。"
    if "inertia" in n:
        return "转动惯量相关属性。"
    return "普通 USD 属性。"


def explain_rel(name: str) -> str:
    n = name.lower()
    if "body0" in n:
        return "关节连接的第一个刚体，通常可理解为参考/父刚体。"
    if "body1" in n:
        return "关节连接的第二个刚体，通常可理解为相对 body0 运动的刚体。"
    if "material" in n:
        return "材质绑定关系。"
    return "USD relationship，表示这个 Prim 指向其他 Prim 或资源。"


def print_find(prims: List[Usd.Prim], keyword: str, index: IdIndex):
    k = keyword.lower()
    table = Table(title=f"搜索结果：{keyword}", box=box.ROUNDED, expand=True)
    table.add_column("编号", ratio=1)
    table.add_column("路径", ratio=5, overflow="fold")
    table.add_column("类型", ratio=2)
    table.add_column("匹配原因", ratio=4, overflow="fold")
    count = 0
    for p in prims:
        reasons = []
        path = str(p.GetPath())
        if k in path.lower():
            reasons.append("路径包含关键词")
        for attr in p.GetAttributes():
            if k in attr.GetName().lower():
                reasons.append(f"属性名包含：{attr.GetName()}")
        for rel in p.GetRelationships():
            if k in rel.GetName().lower():
                reasons.append(f"关系名包含：{rel.GetName()}")
        if reasons:
            count += 1
            table.add_row(index.get(path), path, type_name(p), "\n".join(reasons))
    if count == 0:
        table.add_row("—", "没有匹配", "—", "—")
    console.print(table)


# =============================================================================
# 主程序
# =============================================================================


def main():
    parser = argparse.ArgumentParser(description="中文友好的 USD 机械夹爪结构查看器")
    parser.add_argument("usd_file", help="USD/USDA/USDC 文件路径")
    parser.add_argument("--root", default="/", help="只查看某个根路径下的内容，例如 /panda_gripper")
    parser.add_argument("--show-render", action="store_true", help="显示 Render 渲染配置节点；默认隐藏，减少噪声")
    parser.add_argument("--all-prims", action="store_true", help="遍历所有 Prim，包括默认 Traverse 可能跳过的 inactive/abstract 等")
    parser.add_argument("--no-tree", action="store_true", help="不打印层级树")
    parser.add_argument("--detail", default=None, help="查看某个 Prim 的完整属性/关系详情")
    parser.add_argument("--find", default=None, help="搜索包含关键词的 Prim/属性/关系")
    parser.add_argument("--width", type=int, default=None, help="输出宽度；建议 150~200，终端越宽越好看")
    args = parser.parse_args()

    global console
    term_width = shutil.get_terminal_size((160, 30)).columns
    width = args.width or max(150, term_width)
    console = Console(width=width)

    stage = Usd.Stage.Open(args.usd_file)
    if stage is None:
        raise RuntimeError(f"无法打开 USD 文件：{args.usd_file}")

    prims = collect_prims(stage, args.root, args.show_render, args.all_prims)
    index = build_index(prims)
    meters_per_unit = UsdGeom.GetStageMetersPerUnit(stage)

    console.rule("[bold cyan]USD 机械夹爪中文检查报告[/bold cyan]")

    print_stage_info(stage, args.usd_file, args.root)
    print_summary(prims)
    if not args.no_tree:
        print_tree(prims)
    print_path_index(index)
    print_bodies(prims, index)
    print_joints(prims, index, meters_per_unit)
    print_colliders(prims, index)
    print_meshes(prims, index)
    print_materials(prims, index)

    if args.find:
        print_find(prims, args.find, index)

    if args.detail:
        print_detail(stage, args.detail, index)

    print_diagnostics(prims, index)

    console.rule("[bold cyan]报告结束[/bold cyan]")


if __name__ == "__main__":
    main()
