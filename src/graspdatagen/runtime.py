"""Isaac Sim 6.0.1 lifecycle and the single PhysX/Warp boundary.

Importing this module does not initialize Kit or CUDA. Create one runtime per
process, then reuse it across scenes. Physics imports must follow SimulationApp.
"""

from __future__ import annotations

import gc
import time
from dataclasses import dataclass
from importlib.metadata import distribution, version
from pathlib import Path
from traceback import print_exc
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    import numpy as np
    import trimesh
    import warp as wp
    from isaacsim import SimulationApp
    from isaacsim.core.simulation_manager import SimulationManager
    from omni.physics.tensors.api import SimulationView
    from pxr import Usd

    from graspdatagen.config import CalibrationConfig, GripperConfig, ValidationProfile
    from graspdatagen.geometry import FloatArray
    from graspdatagen.records import PreparedPair


@dataclass(frozen=True)
class RuntimeConfig:
    device_index: int
    steps_per_second: int

    def __post_init__(self) -> None:
        if self.device_index < 0 or self.steps_per_second <= 0:
            raise ValueError("Device index must be nonnegative and step rate positive")


class PhysxRuntime:
    """Own Kit and recreate scenes only after stopping and invalidating views."""

    def __init__(self, config: RuntimeConfig, gui: bool = False) -> None:
        """Enable GUI for watching validation; preparation and batch jobs omit it."""
        if version("isaacsim") != "6.0.1.0":
            raise RuntimeError("The native runtime requires isaacsim==6.0.1.0")
        from isaacsim import SimulationApp

        self.config: RuntimeConfig = config
        self.gui: bool = gui
        self.visualizing: bool = False
        self.frame_time: float = time.monotonic()
        self.render_elapsed: float = 0.0
        self.app: SimulationApp = SimulationApp(
            {
                "headless": not gui,
                "active_gpu": config.device_index,
                "physics_gpu": config.device_index,
                "multi_gpu": False,
                "fast_shutdown": True,
                "create_new_stage": False,
                "extra_args": [
                    "--/exts/isaacsim.core.simulation_manager/default_engine=physx",
                    "--/physics/cudaDevice=" + str(config.device_index),
                    "--/persistent/renderer/startupMessageDisplayed=true",
                    "--ext-folder",
                    str(distribution("isaacsim").locate_file("isaacsim/extscache")),
                ],
            },
            experience=str(
                distribution("isaacsim").locate_file("isaacsim/apps/isaacsim.exp.base.python.kit")
                if gui
                else Path(__file__).with_name("physx.kit")
            ),
        )
        self.closed: bool = False
        self.started: bool = False
        from isaacsim.core.simulation_manager import SimulationManager

        self.manager: type[SimulationManager] = SimulationManager
        try:
            if not self.manager.switch_physics_engine("physx"):
                raise RuntimeError("PhysX could not be activated")
            self.manager.set_device(f"cuda:{config.device_index}")
            self.new_scene()
        except BaseException:
            print_exc()
            self.close(exit_code=1)
            raise

    @property
    def stage(self) -> Usd.Stage:
        """Resolve the current stage without retaining it across Kit shutdown."""
        import omni.usd

        if self.closed:
            raise RuntimeError("Runtime is closed")
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError("No active USD stage")
        return stage

    def new_scene(self) -> None:
        import omni.timeline
        import omni.usd
        from isaacsim.core.simulation_manager import PhysxScene
        from pxr import UsdGeom

        if self.closed:
            raise RuntimeError("Runtime is closed")
        omni.timeline.get_timeline_interface().stop()
        self.app.update()
        self.manager.invalidate_physics()
        self.started = False
        self.visualizing = False
        omni.usd.get_context().new_stage()
        UsdGeom.SetStageMetersPerUnit(self.stage, 1.0)
        UsdGeom.SetStageUpAxis(self.stage, UsdGeom.Tokens.z)
        UsdGeom.Xform.Define(self.stage, "/World")
        scene = PhysxScene("/World/physicsScene")
        scene.set_steps_per_second(self.config.steps_per_second)
        scene.set_gravity((0.0, 0.0, -9.81))
        scene.set_solver_type("TGS")
        scene.set_enabled_gpu_dynamics(True)
        scene.set_broadphase_type("GPU")

    def reference(self, source: Path, prim_path: str) -> Usd.Prim:
        """Reference a complete asset into the current stage without editing it."""
        if self.started:
            raise RuntimeError("Asset references must be added before physics starts")
        if not source.is_file():
            raise FileNotFoundError(source)
        prim = self.stage.DefinePrim(prim_path, "Xform")
        if not prim.GetReferences().AddReference(str(source.resolve())):
            raise ValueError(f"Could not reference {source}")
        return prim

    def start(self) -> SimulationView:
        import omni.timeline

        if self.closed or self.started:
            raise RuntimeError("Physics must start exactly once per scene")
        if self.gui:
            # Flush cloned USD geometry to the viewport before physics owns it.
            self.app.update()
        omni.timeline.get_timeline_interface().play()
        self.app.update()
        view = self.manager.get_physics_simulation_view()
        if view is None or not view.is_valid:
            raise RuntimeError("PhysX did not create a valid Warp simulation view")
        if self.manager.get_active_physics_engine() != "physx":
            raise RuntimeError("Unexpected physics engine")
        if not self.manager.get_device().is_cuda:
            raise RuntimeError("GPU physics is required; CPU fallback is invalid")
        self.started = True
        return view

    def step(self, count: int = 1) -> None:
        """Advance one control tick, or an explicit count for a hold interval."""
        if not self.started or count < 1:
            raise RuntimeError("Stepping requires a running scene and a positive count")
        if not self.visualizing:
            self.manager.step(steps=count)
            return
        import omni.timeline

        timeline = omni.timeline.get_timeline_interface()
        for _ in range(count):
            while not timeline.is_playing():
                if timeline.is_stopped():
                    raise RuntimeError("Visualization stopped before validation completed")
                self.render_elapsed = 1 / 30
                self.render()
            self.render_elapsed += 1 / self.config.steps_per_second
            refresh = self.render_elapsed >= 1 / 30
            self.manager.step(update_fabric=refresh)
            if refresh:
                self.render()

    def render(self) -> None:
        """Refresh the viewport without adding an uncontrolled physics step."""
        import carb

        if not self.app.is_running():
            raise RuntimeError("Visualization window closed before validation completed")
        settings = carb.settings.get_settings()
        playing = settings.get("/app/player/playSimulations")
        settings.set_bool("/app/player/playSimulations", False)
        try:
            self.app.update()
        finally:
            settings.set_bool("/app/player/playSimulations", playing)
        delay = self.render_elapsed - (time.monotonic() - self.frame_time)
        if delay > 0:
            time.sleep(delay)
        self.frame_time = time.monotonic()
        self.render_elapsed = 0.0

    def frame_validation(self, positions: FloatArray, radius: float) -> None:
        """Frame every trial cell; headless validation does not need a camera."""
        if not self.gui:
            return
        import numpy as np
        import omni.ui as ui
        from omni.kit.viewport.utility import get_active_viewport
        from pxr import Gf, UsdGeom, UsdLux

        lower, upper = positions.min(axis=0) - radius, positions.max(axis=0) + radius
        center = (lower + upper) / 2
        viewport = get_active_viewport()
        if viewport is None:
            raise RuntimeError("GUI did not create a viewport")
        for name in ("Content", "Console", "Stage", "Property", "Render Settings"):
            window = ui.Workspace.get_window(name)
            if window:
                window.visible = False
        camera = UsdGeom.Camera.Define(self.stage, "/World/Overview")
        camera.CreateClippingRangeAttr(Gf.Vec2f(0.01, 10000))
        camera.CreateFocalLengthAttr(24)
        direction = np.array([0.35, -1.0, 1.25])
        direction /= np.linalg.norm(direction)
        width, height = viewport.resolution
        aperture = camera.GetHorizontalApertureAttr().Get()
        half_angle = np.arctan(min(aperture, aperture * height / width) / 48)
        distance = np.linalg.norm(upper - lower) / (2 * np.sin(half_angle))
        eye = center + direction * distance
        transform = (
            Gf.Matrix4d()
            .SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*center), Gf.Vec3d(0, 0, 1))
            .GetInverse()
        )
        UsdGeom.Xformable(camera).MakeMatrixXform().Set(transform)
        viewport.set_active_camera(str(camera.GetPath()))
        light = UsdLux.DomeLight.Define(self.stage, "/World/OverviewLight")
        light.CreateIntensityAttr(1000)
        self.visualizing = True
        self.frame_time = time.monotonic()
        self.render()

    def cook_convexes(self, prim_path: str) -> list[trimesh.Trimesh]:
        """Return PhysX's actual cooked hulls in the collision mesh local frame."""
        import numpy as np
        import trimesh
        from omni.physx import get_physx_cooking_interface
        from omni.physx.bindings._physx import (
            PhysxCollisionRepresentationResult,
            PhysxConvexMeshData,
        )
        from pxr import PhysicsSchemaTools, UsdUtils

        hulls: list[trimesh.Trimesh] = []
        errors: list[str] = []

        def receive(
            result: PhysxCollisionRepresentationResult, convexes: list[PhysxConvexMeshData]
        ) -> None:
            if result != PhysxCollisionRepresentationResult.RESULT_VALID:
                errors.append(str(result))
                return
            for convex in convexes:
                vertices = np.array([(v.x, v.y, v.z) for v in convex.vertices])
                polygons = [
                    convex.indices[p.index_base : p.index_base + p.num_vertices]
                    for p in convex.polygons
                ]
                # PhysX polygons are convex, so trimesh's fan triangulation is valid.
                faces = trimesh.geometry.triangulate_quads(polygons)
                hulls.append(trimesh.Trimesh(vertices, faces, process=True))

        get_physx_cooking_interface().request_convex_collision_representation(
            stage_id=UsdUtils.StageCache.Get().GetId(self.stage).ToLongInt(),
            collision_prim_id=PhysicsSchemaTools.sdfPathToInt(prim_path),
            run_asynchronously=False,
            on_result=receive,
        )
        if errors or not hulls:
            raise RuntimeError(f"Collision cooking failed for {prim_path}: {errors}")
        # PhysX does not guarantee decomposition callback ordering.
        hulls.sort(key=lambda mesh: tuple(mesh.bounds.ravel()) + (len(mesh.vertices),))
        return hulls

    def measure_object(self, source: Path) -> dict[str, Any]:
        """Cook a prepared dynamic body and read final GPU PhysX mass properties."""
        import numpy as np
        from pxr import PhysxSchema

        self.new_scene()
        prim = self.reference(source, "/World/object")
        PhysxSchema.PhysxRigidBodyAPI.Apply(prim).CreateDisableGravityAttr(True)
        view = self.start()
        body = view.create_rigid_body_view("/World/object")
        if body.count != 1:
            raise RuntimeError("Expected one prepared rigid body")
        mass = body.get_masses().numpy().copy()
        com = body.get_coms().numpy().copy()
        inertia = body.get_inertias().numpy().copy().reshape(-1, 3, 3)
        materials = body.get_material_properties().numpy().copy()
        if not all(np.isfinite(a).all() for a in (mass, com, inertia, materials)):
            raise RuntimeError("Nonfinite cooked object properties")
        if (mass <= 0).any() or (np.linalg.eigvalsh(inertia) <= 0).any():
            raise RuntimeError("Invalid cooked mass/inertia")
        return {
            "mass_kg": float(mass[0, 0]),
            "com_pose_xyzw": com[0].tolist(),
            "inertia_kg_m2": inertia[0].tolist(),
            "materials": materials[0].tolist(),
            "shapes": body.max_shapes,
            "gpu_device": view.device_ordinal,
        }

    def measure_gripper(
        self,
        source: Path,
        active_name: str,
        follower_name: str,
        calibration: CalibrationConfig,
    ) -> dict[str, Any]:
        """Measure travel from identical initial states, commanding only the active DOF."""
        import numpy as np
        import warp as wp
        from pxr import PhysxSchema, Usd, UsdPhysics

        self.new_scene()
        root = self.reference(source, "/World/gripper")
        for prim in Usd.PrimRange(root):
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                PhysxSchema.PhysxRigidBodyAPI.Apply(prim).CreateDisableGravityAttr(True)
        view = self.start()
        articulation = view.create_articulation_view("/World/gripper/root_joint")
        names = list(articulation.shared_metatype.dof_names)
        if articulation.count != 1 or set(names) != {active_name, follower_name}:
            raise RuntimeError(f"Unexpected pure-gripper DOFs: {names}")
        active, follower = names.index(active_name), names.index(follower_name)
        indices: wp.array = wp.array([0], dtype=wp.int32, device=view.device)
        limits = articulation.get_dof_limits().numpy().copy()[0]
        stiffness = articulation.get_dof_stiffnesses().numpy().copy()[0]
        damping = articulation.get_dof_dampings().numpy().copy()[0]
        max_forces = articulation.get_dof_max_forces().numpy().copy()[0]
        max_velocities = articulation.get_dof_max_velocities().numpy().copy()[0]
        if stiffness[follower] != 0 or damping[follower] != 0:
            raise RuntimeError("Mimic follower has a conflicting independent drive")
        mimic_prims = [p for p in Usd.PrimRange(root) if p.GetName() == follower_name]
        if len(mimic_prims) != 1:
            raise RuntimeError("Cannot resolve follower joint")
        mimic_schemas = [
            s for s in mimic_prims[0].GetAppliedSchemas() if s.startswith("PhysxMimicJointAPI:")
        ]
        if len(mimic_schemas) != 1:
            raise RuntimeError("Expected one effective mimic schema")
        mimic = PhysxSchema.PhysxMimicJointAPI(mimic_prims[0], mimic_schemas[0].split(":")[1])
        gearing = float(mimic.GetGearingAttr().Get())
        offset = float(mimic.GetOffsetAttr().Get())
        if gearing == 0 or not np.isfinite([gearing, offset]).all():
            raise RuntimeError("Invalid mimic parameters")
        lo, hi = limits[active]
        # PhysX mimic equation: follower + gearing * reference + offset = 0.
        follower_range = sorted(
            ((-limits[follower, 0] - offset) / gearing, (-limits[follower, 1] - offset) / gearing)
        )
        lo, hi = max(lo, follower_range[0]), min(hi, follower_range[1])
        if not np.isfinite([lo, hi]).all() or hi <= lo:
            raise RuntimeError("Mimic joints have no shared finite legal travel")
        commands = np.linspace(lo, hi, calibration.samples)
        initial = np.zeros((1, len(names)), dtype=np.float32)
        initial[0, active] = lo
        initial[0, follower] = -gearing * lo - offset
        zero = wp.zeros(initial.shape, dtype=wp.float32, device=view.device)
        positions, velocities, link_poses, peaks = [], [], [], []
        for command in commands:
            articulation.set_dof_positions(
                wp.array(initial, dtype=wp.float32, device=view.device), indices
            )
            articulation.set_dof_velocities(zero, indices)
            targets = initial.copy()
            targets[0, active] = command
            articulation.set_dof_position_targets(
                wp.array(targets, dtype=wp.float32, device=view.device), indices
            )
            peak = np.zeros(len(names))
            for _ in range(calibration.settle_steps):
                self.step()
                speed = articulation.get_dof_velocities().numpy()[0]
                peak = np.maximum(peak, np.abs(speed))
            actual = articulation.get_dof_positions().numpy().copy()[0]
            speed = articulation.get_dof_velocities().numpy().copy()[0]
            pose = articulation.get_link_transforms().numpy().copy()[0]
            if not all(np.isfinite(a).all() for a in (actual, speed, pose, peak)):
                raise RuntimeError("Nonfinite gripper calibration state")
            if abs(actual[active] - command) > calibration.position_tolerance_m:
                raise RuntimeError(f"Active joint failed to converge: {command} -> {actual}")
            if (
                abs(actual[follower] + gearing * actual[active] + offset)
                > calibration.position_tolerance_m
            ):
                raise RuntimeError("Mimic relation failed during calibration")
            if (actual < limits[:, 0] - calibration.position_tolerance_m).any() or (
                actual > limits[:, 1] + calibration.position_tolerance_m
            ).any():
                raise RuntimeError("Calibration exceeded a joint limit")
            if np.max(np.abs(speed)) > calibration.velocity_tolerance_m_s:
                raise RuntimeError("Gripper failed to settle")
            positions.append(actual)
            velocities.append(speed)
            link_poses.append(pose)
            peaks.append(peak)
        return {
            "joint_names": names,
            "link_names": list(articulation.shared_metatype.link_names),
            "active_index": active,
            "follower_index": follower,
            "commands_m": commands.tolist(),
            "positions_m": np.asarray(positions).tolist(),
            "velocities_m_s": np.asarray(velocities).tolist(),
            "peak_velocities_m_s": np.asarray(peaks).tolist(),
            "link_poses_xyzw": np.asarray(link_poses).tolist(),
            "limits_m": limits.tolist(),
            "stiffness": stiffness.tolist(),
            "damping": damping.tolist(),
            "max_forces_N": max_forces.tolist(),
            "max_velocities_m_s": max_velocities.tolist(),
            "mimic": {"schema": mimic_schemas[0], "gearing": gearing, "offset": offset},
            "gpu_device": view.device_ordinal,
        }

    def probe_gripper_contact(
        self,
        source: Path,
        config: GripperConfig,
        definition: dict[str, Any],
        friction: float,
    ) -> dict[str, Any]:
        """A 40 mm gauge fixture checks bilateral target contact and friction sensitivity.

        This is an asset/control check, not a P2 object-grasp success label.
        """
        import numpy as np
        import warp as wp
        from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics
        from scipy.spatial.transform import Rotation

        from graspdatagen.assets import bind_material
        from graspdatagen.config import MaterialConfig

        self.new_scene()
        root = self.reference(source, "/World/gripper")
        self.stage.SetDefaultPrim(root)
        colliders = []
        for prim in Usd.PrimRange(root):
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                PhysxSchema.PhysxRigidBodyAPI.Apply(prim).CreateDisableGravityAttr(True)
                PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.0)
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                colliders.append(prim)
        bind_material(
            self.stage, colliders, MaterialConfig(friction, friction, 0.0, config.material.source)
        )
        c = config.calibration
        probe = UsdGeom.Cube.Define(self.stage, "/World/probe")
        probe.CreateSizeAttr(1.0)
        UsdGeom.Xformable(probe).AddTranslateOp().Set(Gf.Vec3d(10.0, 0.0, 0.0))
        UsdGeom.Xformable(probe).AddScaleOp().Set(
            Gf.Vec3d(c.contact_probe_depth_m, c.contact_probe_width_m, c.contact_probe_height_m)
        )
        UsdPhysics.CollisionAPI.Apply(probe.GetPrim())
        UsdPhysics.RigidBodyAPI.Apply(probe.GetPrim())
        UsdPhysics.MassAPI.Apply(probe.GetPrim()).CreateMassAttr(c.contact_probe_mass_kg)
        PhysxSchema.PhysxRigidBodyAPI.Apply(probe.GetPrim()).CreateDisableGravityAttr(True)
        bind_material(
            self.stage,
            [probe.GetPrim()],
            MaterialConfig(friction, friction, 0.0, config.material.source),
        )
        view = self.start()
        articulation = view.create_articulation_view("/World/gripper/root_joint")
        body = view.create_rigid_body_view("/World/probe")
        sensors = view.create_rigid_contact_view(
            ["/World/gripper/" + name for name in definition["finger_body_names"]],
            filter_patterns=[["/World/probe"], ["/World/probe"]],
            max_contact_data_count=256,
        )
        gpu: wp.array = wp.array([0], dtype=wp.int32, device=view.device)
        cpu: wp.array = wp.array([0], dtype=wp.int32, device="cpu")
        active = definition["calibration"]["active_index"]
        command_index = int(
            np.argmax(
                np.asarray(definition["calibration"]["commands_m"]) == definition["open_command_m"]
            )
        )
        targets = np.asarray(
            definition["calibration"]["positions_m"][command_index], dtype=np.float32
        )[None, :]
        articulation.set_dof_positions(wp.array(targets, dtype=wp.float32, device=view.device), gpu)
        articulation.set_dof_position_targets(
            wp.array(targets, dtype=wp.float32, device=view.device), gpu
        )
        articulation.set_dof_velocities(
            wp.zeros(targets.shape, dtype=wp.float32, device=view.device), gpu
        )
        self.step(c.settle_steps)
        T_B_tcp = np.asarray(definition["tcp"]["T_B_tcp"])
        approach = np.asarray(config.approach_axis_base)
        opening = np.asarray(config.opening_axis_base)
        rotation = np.column_stack((approach, opening, np.cross(approach, opening)))
        pose = np.concatenate((T_B_tcp[:3, 3], Rotation.from_matrix(rotation).as_quat()))[None, :]
        body.set_transforms(wp.array(pose, dtype=wp.float32, device=view.device), gpu)
        body.set_velocities(wp.zeros((1, 6), dtype=wp.float32, device=view.device), gpu)
        targets[0, active] = definition["closed_command_m"]
        articulation.set_dof_position_targets(
            wp.array(targets, dtype=wp.float32, device=view.device), gpu
        )
        self.step(c.settle_steps)
        closed_pose = body.get_transforms().numpy().copy()[0]
        forces = sensors.get_contact_force_matrix(1.0 / self.config.steps_per_second).numpy().copy()
        force_norms = np.linalg.norm(forces.reshape(2, -1, 3).sum(axis=1), axis=1)
        positions = articulation.get_dof_positions().numpy().copy()[0]
        if not np.isfinite(force_norms).all() or np.min(force_norms) < c.minimum_contact_force_N:
            raise RuntimeError(f"Gauge fixture has no bilateral contact: {force_norms}")
        body.set_disable_gravities(wp.zeros((1, 1), dtype=wp.uint8, device="cpu"), cpu)
        body.wake_up(cpu)
        self.step(c.hold_steps)
        final_pose = body.get_transforms().numpy().copy()[0]
        final_forces = (
            sensors.get_contact_force_matrix(1.0 / self.config.steps_per_second).numpy().copy()
        )
        hold_norms = np.linalg.norm(final_forces.reshape(2, -1, 3).sum(axis=1), axis=1)
        displacement = float(np.linalg.norm(final_pose[:3] - closed_pose[:3]))
        return {
            "friction": friction,
            "fixture_mass_kg": c.contact_probe_mass_kg,
            "closing_target_contact_forces_N": force_norms.tolist(),
            "holding_target_contact_forces_N": hold_norms.tolist(),
            "actual_joint_positions_m": positions.tolist(),
            "hold_translation_m": displacement,
            "held": bool(
                displacement < c.max_probe_translation_m
                and np.min(hold_norms) >= c.minimum_contact_force_N
            ),
            "scope": "gauge fixture, no grasp dataset label",
        }

    def close(self, exit_code: int = 0) -> None:
        """Stop physics, then terminate the worker using Kit's fast shutdown.

        Persist results before calling. The supervising process verifies exit;
        Kit's supported fast-shutdown path does not return to the caller.
        Omit exit_code for success; failed qualification passes 1 explicitly.
        """
        if not self.closed:
            import omni.timeline

            omni.timeline.get_timeline_interface().stop()
            self.app.update()
            self.manager.invalidate_physics()
            gc.collect()
            self.closed = True
            self.started = False
            self.app.close(wait_for_replicator=False, exit_code=exit_code)


class GraspScene:
    """Homogeneous GPU environments; all USD and tensor access stays at this boundary."""

    def __init__(
        self, runtime: PhysxRuntime, pair: PreparedPair, count: int, profile: ValidationProfile
    ) -> None:
        self.runtime: PhysxRuntime = runtime
        self.count: int = count
        self.profile: ValidationProfile = profile
        self.pair: PreparedPair = pair
        self.mass_scale: float = 1.0
        self.friction_scale: float = 1.0
        self.has_trial_state: bool = False
        self._create_scene()

    def _create_scene(self) -> None:
        import numpy as np
        import warp as wp
        from isaacsim.core.cloner import Cloner
        from pxr import PhysxSchema, Usd, UsdGeom, UsdPhysics

        runtime, pair, count, profile = self.runtime, self.pair, self.count, self.profile
        runtime.config = RuntimeConfig(runtime.config.device_index, profile.steps_per_second)
        runtime.new_scene()
        physics = PhysxSchema.PhysxSceneAPI(runtime.stage.GetPrimAtPath("/World/physicsScene"))
        physics.CreateSolverTypeAttr(profile.solver_type)
        physics.CreateEnableExternalForcesEveryIterationAttr(
            profile.external_forces_every_iteration
        )
        physics.CreateSolveArticulationContactLastAttr(profile.solve_articulation_contact_last)
        physics.CreateGpuFoundLostPairsCapacityAttr(profile.gpu_found_lost_pairs_capacity)
        physics.CreateGpuFoundLostAggregatePairsCapacityAttr(
            profile.gpu_found_lost_aggregate_pairs_capacity
        )
        physics.CreateGpuTotalAggregatePairsCapacityAttr(profile.gpu_total_aggregate_pairs_capacity)
        physics.CreateGpuMaxRigidContactCountAttr(profile.gpu_max_rigid_contact_count)
        cloner = Cloner(stage=runtime.stage)
        cloner.define_base_env("/World/envs")
        paths = cloner.generate_paths("/World/envs/env", count)
        UsdGeom.Xform.Define(runtime.stage, paths[0])
        runtime.reference(pair.gripper / "gripper.usdc", paths[0] + "/gripper")
        runtime.reference(pair.object / "object.usdc", paths[0] + "/object")
        for prim in Usd.PrimRange(runtime.stage.GetPrimAtPath(paths[0])):
            if runtime.gui and prim.IsA(UsdGeom.Imageable):
                imageable = UsdGeom.Imageable(prim)
                imageable.CreateVisibilityAttr(UsdGeom.Tokens.inherited)
                if "/object/" in str(prim.GetPath()):
                    imageable.CreatePurposeAttr(UsdGeom.Tokens.default_)
                    if prim.IsA(UsdGeom.Mesh):
                        UsdGeom.Mesh(prim).CreateDisplayColorAttr([(0.65, 0.24, 0.16)])
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                api = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
                api.CreateDisableGravityAttr(True)
                api.CreateSleepThresholdAttr(0.0)
                api.CreateSolverPositionIterationCountAttr(profile.solver_position_iterations)
                api.CreateSolverVelocityIterationCountAttr(profile.solver_velocity_iterations)
                PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.0)
            if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                api = PhysxSchema.PhysxArticulationAPI.Apply(prim)
                api.CreateSolverPositionIterationCountAttr(profile.solver_position_iterations)
                api.CreateSolverVelocityIterationCountAttr(profile.solver_velocity_iterations)
                api.CreateSleepThresholdAttr(0.0)
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                collision = PhysxSchema.PhysxCollisionAPI.Apply(prim)
                collision.CreateContactOffsetAttr(profile.contact_offset_m)
                collision.CreateRestOffsetAttr(0.0)
        # P0 qualified colocated isolation. P2 also spaces cells by swept asset bounds.
        with np.load(pair.object / "geometry.npz", allow_pickle=False) as geometry:
            radius = np.linalg.norm(geometry["surface_vertices_m"], axis=1).max()
        with np.load(pair.gripper / "collision_geometry.npz", allow_pickle=False) as geometry:
            radius += max(
                np.linalg.norm(geometry[k], axis=1).max()
                for k in geometry.files
                if k.endswith("vertices_B_m")
            )
        spacing = 4 * radius
        self.visual_radius: float = float(radius)
        from graspdatagen.geometry import set_transform

        parked = np.eye(4)
        parked[1, 3] = spacing
        set_transform(runtime.stage.GetPrimAtPath(paths[0] + "/object"), parked)
        self.origins: FloatArray = np.zeros((count, 3))
        # A compact grid avoids losing world-position precision at large counts.
        columns = int(np.ceil(np.sqrt(count)))
        self.origins[:, 0] = np.arange(count) % columns * spacing
        self.origins[:, 1] = np.arange(count) // columns * spacing
        cloner.clone(
            source_prim_path=paths[0],
            prim_paths=paths,
            positions=self.origins,
            replicate_physics=True,
            enable_env_ids=True,
        )
        self.view = runtime.start()
        self.device: str = str(self.view.device)
        self.articulation = self.view.create_articulation_view(
            "/World/envs/env_*/gripper/root_joint"
        )
        self.body = self.view.create_rigid_body_view("/World/envs/env_*/object")
        if self.articulation.count != count or self.body.count != count:
            raise RuntimeError("Cloned view count mismatch")
        if list(self.body.prim_paths) != [p + "/object" for p in paths]:
            raise RuntimeError("Unexpected PhysX environment ordering")
        if list(self.articulation.prim_paths) != [p + "/gripper/root_joint" for p in paths]:
            raise RuntimeError("Unexpected PhysX articulation ordering")
        if list(self.articulation.shared_metatype.dof_names) != pair.definition["joint_names"]:
            raise RuntimeError("Runtime joint order differs from calibration")
        fingers = pair.definition["finger_body_names"]
        links = list(self.articulation.shared_metatype.link_names)
        self.contact_links: list[str] = fingers + [link for link in links if link not in fingers]
        self.base_index: int = links.index(pair.definition["extraction"]["base_body"])
        sensor_paths = [path + "/gripper/" + link for path in paths for link in self.contact_links]
        filter_paths = [
            (
                [path + "/object", paths[(i + 1) % count] + "/object"]
                if count > 1
                else [path + "/object"]
            )
            for i, path in enumerate(paths)
            for _ in self.contact_links
        ]
        self.contacts = self.view.create_rigid_contact_view(
            sensor_paths,
            filter_paths,
            max_contact_data_count=count * profile.contact_buffer_per_env,
        )
        if (
            list(self.contacts.sensor_paths) != sensor_paths
            or self.contacts.filter_paths != filter_paths
        ):
            raise RuntimeError("Unexpected PhysX contact sensor/filter ordering")
        self.gpu_indices: wp.array = wp.array(np.arange(count), dtype=wp.int32, device=self.device)
        self.cpu_indices: wp.array = wp.array(np.arange(count), dtype=wp.int32, device="cpu")
        self.zero_velocity: wp.array = wp.zeros((count, 6), dtype=wp.float32, device=self.device)
        self.zero_joints: wp.array = wp.zeros(
            (count, len(pair.definition["joint_names"])), dtype=wp.float32, device=self.device
        )
        self.force_buffer: wp.array = wp.zeros((count, 3), dtype=wp.float32, device=self.device)
        self.dofs: int = len(pair.definition["joint_names"])
        self.state_buffer: wp.array = wp.empty(
            (count, 20 + 2 * self.dofs + 2 * len(self.contact_links)),
            dtype=wp.float32,
            device=self.device,
        )
        self.command_buffer: wp.array = wp.empty(
            (count, self.dofs), dtype=wp.float32, device=self.device
        )
        self.base_buffer: wp.array = wp.empty((count, 7), dtype=wp.float32, device=self.device)
        self.masses: FloatArray = self.body.get_masses().numpy().astype(np.float64)
        self.nominal_masses: FloatArray = self.masses.copy()
        self.nominal_inertias: FloatArray = self.body.get_inertias().numpy().astype(np.float64)
        self.nominal_materials: list[FloatArray] = [
            self.body.get_material_properties().numpy().astype(np.float64),
            self.articulation.get_material_properties().numpy().astype(np.float64),
        ]
        self.active_index: int = pair.definition["calibration"]["active_index"]
        if not np.allclose(self.masses, pair.object_manifest["physical"]["mass_kg"]):
            raise RuntimeError("Runtime mass differs from prepared physics")
        if not np.allclose(
            self.articulation.get_dof_max_forces().numpy(),
            pair.definition["calibration"]["max_forces_N"],
        ):
            raise RuntimeError("Drive force limits changed")
        if (self.mass_scale, self.friction_scale) != (1.0, 1.0):
            self.physical_scales(self.mass_scale, self.friction_scale)

    def gpu(self, data: FloatArray) -> wp.array:
        import warp as wp

        return wp.array(data, dtype=wp.float32, device=self.device)

    def gravity(self, enabled: bool) -> None:
        import numpy as np
        import warp as wp

        self.body.set_disable_gravities(
            wp.array(np.full((self.count, 1), not enabled), dtype=wp.uint8, device="cpu"),
            self.cpu_indices,
        )
        self.body.wake_up(self.cpu_indices)

    def reset(self, object_poses: FloatArray, base_poses: FloatArray, joints: FloatArray) -> None:
        """Start an independent trial, including PhysX's internal contact/solver state."""
        import numpy as np

        # Tensor pose/velocity writes leave contact history behind. In real replay
        # this changed a later trial's outcome when the environment count changed.
        if self.has_trial_state:
            self._create_scene()
        self.gravity(False)
        self.force(np.zeros((self.count, 3)))
        self.body.set_transforms(self.gpu(object_poses), self.gpu_indices)
        self.body.set_velocities(self.zero_velocity, self.gpu_indices)
        self.articulation.set_root_transforms(self.gpu(base_poses), self.gpu_indices)
        self.articulation.set_root_velocities(self.zero_velocity, self.gpu_indices)
        self.articulation.set_dof_positions(self.gpu(joints), self.gpu_indices)
        self.articulation.set_dof_velocities(self.zero_joints, self.gpu_indices)
        self.command(joints)
        self.has_trial_state = True

    def command(self, joints: FloatArray) -> None:
        self.command_buffer.assign(joints.astype("float32"))
        self.articulation.set_dof_position_targets(self.command_buffer, self.gpu_indices)

    def move_base(self, poses: FloatArray) -> None:
        self.base_buffer.assign(poses.astype("float32"))
        self.articulation.set_root_transforms(self.base_buffer, self.gpu_indices)

    def force(self, accelerations: FloatArray) -> None:
        """Apply world-frame F=ma at the COM; None is the tensor API's zero-torque/COM mode."""
        self.force_buffer.assign((accelerations * self.masses).astype("float32"))
        self.body.apply_forces_and_torques_at_position(
            self.force_buffer, None, None, self.gpu_indices, True
        )

    def read(self) -> dict[str, np.ndarray]:
        import numpy as np
        import warp as wp

        from graspdatagen.geometry import pose_matrices
        from graspdatagen.kernels import pack_grasp_state

        wp.launch(
            pack_grasp_state,
            dim=self.count,
            inputs=[
                self.body.get_transforms(),
                self.articulation.get_link_transforms(),
                self.articulation.get_dof_positions(),
                self.articulation.get_dof_velocities(),
                self.body.get_velocities(),
                self.contacts.get_contact_force_matrix(1 / self.profile.steps_per_second),
                self.base_index,
                self.dofs,
                len(self.contact_links),
            ],
            outputs=[self.state_buffer],
            device=self.device,
        )
        values = self.state_buffer.numpy().astype(np.float64)
        if not np.isfinite(values).all():
            raise RuntimeError("Nonfinite PhysX state: solver_invalid")
        contact_offset = 20 + 2 * self.dofs
        if np.any(values[:, contact_offset + len(self.contact_links) :] > 0):
            raise RuntimeError("Cross-environment target contact: solver_invalid")
        result = {
            "object": pose_matrices(values[:, :7]),
            "base": pose_matrices(values[:, 7:14]),
            "joints": values[:, 14 : 14 + self.dofs],
            "joint_velocity": values[:, 14 + self.dofs : 14 + 2 * self.dofs],
            "velocity": values[:, 14 + 2 * self.dofs : contact_offset],
            "contact": values[:, contact_offset : contact_offset + len(self.contact_links)],
        }
        return result

    def physical_scales(self, mass_scale: float, friction_scale: float) -> None:
        """Explicit audit variants: scale density/inertia and both contacting materials.

        Nominal generation never calls this. The audit records both multipliers
        and always restores (1,1) before running a different experiment.
        """
        import numpy as np
        import warp as wp

        if (
            not np.isfinite([mass_scale, friction_scale]).all()
            or min(mass_scale, friction_scale) <= 0
        ):
            raise ValueError("Physical audit multipliers must be positive and finite")
        self.mass_scale = mass_scale
        self.friction_scale = friction_scale
        self.masses = self.nominal_masses * mass_scale
        self.body.set_masses(
            wp.array(self.masses, dtype=wp.float32, device="cpu"), self.cpu_indices
        )
        self.body.set_inertias(
            wp.array(self.nominal_inertias * mass_scale, dtype=wp.float32, device="cpu"),
            self.cpu_indices,
        )
        for view, nominal in zip(
            (self.body, self.articulation), self.nominal_materials, strict=True
        ):
            properties = nominal.copy()
            properties[..., :2] *= friction_scale
            view.set_material_properties(
                wp.array(properties, dtype=wp.float32, device="cpu"), self.cpu_indices
            )
