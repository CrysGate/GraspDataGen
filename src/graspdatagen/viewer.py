"""Static USD inspection of compact YAML grasps without advancing physics."""

from __future__ import annotations

import json
from pathlib import Path
from typing import TYPE_CHECKING

import numpy as np

from graspdatagen.config import read_mapping
from graspdatagen.geometry import FloatArray, pose_matrices, set_transform
from graspdatagen.records import PreparedPair
from graspdatagen.runtime import PhysxRuntime, RuntimeConfig

if TYPE_CHECKING:
    from pxr import Usd


def closure_transforms(
    root: Usd.Prim, base_body: str, joints: dict[str, float]
) -> dict[str, FloatArray]:
    """Place supported fixed/prismatic links using USD joint frames and recorded DOFs."""
    from pxr import Usd, UsdPhysics

    transforms = {str(root.GetPath()) + "/" + base_body: np.eye(4)}
    pending = []
    for prim in Usd.PrimRange(root):
        if not prim.IsA(UsdPhysics.Joint):
            continue
        joint = UsdPhysics.Joint(prim)
        parents, children = joint.GetBody0Rel().GetTargets(), joint.GetBody1Rel().GetTargets()
        if not parents:
            continue  # The prepared root joint anchors the base to the world.
        if len(parents) != 1 or len(children) != 1:
            raise ValueError(f"Unsupported joint body relationship: {prim.GetPath()}")
        frames = []
        for side in (0, 1):
            position = prim.GetAttribute(f"physics:localPos{side}").Get()
            rotation = prim.GetAttribute(f"physics:localRot{side}").Get()
            frames.append(
                pose_matrices(np.array([*position, *rotation.GetImaginary(), rotation.GetReal()]))
            )
        motion = np.eye(4)
        if prim.IsA(UsdPhysics.PrismaticJoint):
            axis = UsdPhysics.PrismaticJoint(prim).GetAxisAttr().Get()
            motion["XYZ".index(axis), 3] = joints[prim.GetName()]
        elif not prim.IsA(UsdPhysics.FixedJoint):
            raise ValueError(f"Static viewer requires fixed/prismatic joints: {prim.GetPath()}")
        pending.append(
            (str(parents[0]), str(children[0]), frames[0] @ motion @ np.linalg.inv(frames[1]))
        )
    while pending:
        remaining = []
        for parent, child, relative in pending:
            if parent in transforms:
                transforms[child] = transforms[parent] @ relative
            else:
                remaining.append((parent, child, relative))
        if len(remaining) == len(pending):
            raise ValueError("Disconnected static gripper joint chain")
        pending = remaining
    return transforms


class GraspViewer:
    def __init__(self, runtime: PhysxRuntime, source: Path, candidate: int) -> None:
        import omni.ui as ui
        from pxr import Usd, UsdGeom, UsdPhysics

        self.runtime = runtime
        self.data = read_mapping(source)
        if self.data["format"] != "graspdatagen.grasps.compact.v1":
            raise ValueError("Unsupported compact grasp format")
        if self.data["position_unit"] != "m" or self.data["pose_layout"] != [
            "x",
            "y",
            "z",
            "qx",
            "qy",
            "qz",
            "qw",
        ]:
            raise ValueError("Viewer requires metre xyz/xyzw poses")
        manifest = json.loads(source.with_name("manifest.json").read_text())
        self.pair = PreparedPair.load(
            Path(manifest["gripper_cache"]), Path(manifest["object_cache"])
        )
        robot = json.loads((self.pair.gripper / "robot_snapshot.json").read_text())["name"]
        self.candidates = self.data["candidates"]
        if not self.candidates or self.data["object"] != self.pair.object_manifest["name"]:
            raise ValueError("Empty grasps or object does not match the asset manifest")
        names = self.pair.definition["joint_names"]
        for index, item in enumerate(self.candidates):
            if item["candidate_id"] != index or item["robot"] != robot:
                raise ValueError("Expected sequential YAML IDs and the matching robot")
            if "closed_joint_positions_m" not in item:
                raise ValueError(
                    f"YAML lacks closure joints; run graspdatagen export --run {source.parent}"
                )
            if set(item["closed_joint_positions_m"]) != set(names):
                raise ValueError("YAML closure joints differ from the prepared gripper")
            if not np.isfinite(list(item["closed_joint_positions_m"].values())).all():
                raise ValueError("Nonfinite closure joints")
            pose_matrices(np.asarray(item["pose_object_tcp_xyz_xyzw"], dtype=np.float64))
        if not 0 <= candidate < len(self.candidates):
            raise ValueError(f"Candidate must be between 0 and {len(self.candidates) - 1}")
        runtime.new_scene()
        self.object = runtime.reference(self.pair.object / "replay.usdc", "/World/Object")
        self.gripper = runtime.reference(self.pair.gripper / "gripper.usdc", "/World/Gripper")
        for prim in Usd.PrimRange(self.gripper):
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                UsdPhysics.RigidBodyAPI(prim).CreateRigidBodyEnabledAttr(False)
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr(False)
            if prim.IsA(UsdPhysics.Joint):
                UsdPhysics.Joint(prim).CreateJointEnabledAttr(False)
            if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)
            if prim.IsA(UsdGeom.Imageable):
                UsdGeom.Imageable(prim).CreateVisibilityAttr(UsdGeom.Tokens.inherited)
        self.current = -1
        self.needs_frame = True
        self.model = ui.SimpleIntModel(candidate)
        self.window = ui.Window("Grasp Inspector", width=330, height=160)
        self.window.position_x = 70
        self.window.position_y = 90
        with self.window.frame:
            with ui.VStack(spacing=8, width=310):
                ui.Label(f"{robot} / {self.data['object']}", height=24)
                with ui.HStack(height=28, spacing=6):
                    self.previous = ui.Button(
                        "<", width=32, tooltip="Previous grasp", clicked_fn=lambda: self.select(-1)
                    )
                    ui.IntDrag(
                        self.model,
                        width=120,
                        min=0,
                        max=len(self.candidates) - 1,
                        step=1,
                        tooltip="YAML candidate ID",
                    )
                    ui.Label(f"/ {len(self.candidates) - 1}", width=65)
                    self.next = ui.Button(
                        ">", width=32, tooltip="Next grasp", clicked_fn=lambda: self.select(1)
                    )
                ui.Button("Frame Grasp", height=28, clicked_fn=self.request_frame)
        self.update()

    def select(self, offset: int) -> None:
        self.model.set_value(max(0, min(len(self.candidates) - 1, self.model.as_int + offset)))

    def request_frame(self) -> None:
        self.needs_frame = True

    def update(self) -> None:
        from pxr import Usd, UsdGeom

        index = max(0, min(len(self.candidates) - 1, self.model.as_int))
        if self.model.as_int != index:
            self.model.set_value(index)
        if self.current != index:
            item = self.candidates[index]
            transforms = closure_transforms(
                self.gripper,
                self.pair.definition["extraction"]["base_body"],
                item["closed_joint_positions_m"],
            )
            for path, transform in transforms.items():
                set_transform(self.runtime.stage.GetPrimAtPath(path), transform)
            tcp = pose_matrices(np.asarray(item["pose_object_tcp_xyz_xyzw"], dtype=np.float64))
            set_transform(self.gripper, tcp @ np.linalg.inv(self.pair.arrays["T_B_tcp"]))
            self.current = index
            self.previous.enabled = index > 0
            self.next.enabled = index + 1 < len(self.candidates)
        if self.needs_frame:
            cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
            bounds = [
                cache.ComputeWorldBound(prim).ComputeAlignedRange()
                for prim in (self.object, self.gripper)
            ]
            points = np.array(
                [list(point) for bound in bounds for point in (bound.GetMin(), bound.GetMax())]
            )
            self.runtime.frame_validation(
                points, float(np.linalg.norm(np.ptp(points, axis=0)) * 0.05)
            )
            self.needs_frame = False


def view_grasps(source: Path, device: int, candidate: int) -> None:
    runtime = PhysxRuntime(RuntimeConfig(device, 120), gui=True)
    try:
        viewer = GraspViewer(runtime, source, candidate)
        while runtime.app.is_running():
            viewer.update()
            runtime.render_elapsed = 1 / 30
            runtime.render()
    except Exception:
        from traceback import print_exc

        print_exc()
        runtime.close(exit_code=1)
        raise
    runtime.close()
