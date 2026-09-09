"""NiceGUI grasp inspection, with browser rendering and CPU-only asset loading."""

from __future__ import annotations

import argparse
import json
import logging
from functools import lru_cache
from pathlib import Path
from typing import Any

from fastapi import HTTPException
from nicegui import app, run, ui

from graspdatagen.config import load_objects
from graspdatagen.web_data import load_dataset

STATIC = Path(__file__).with_name("web_static")


def serve(
    sources: tuple[Path, ...],
    objects: Path,
    grippers: tuple[Path, ...],
    host: str,
    port: int,
    overview_faces: int,
) -> None:
    @lru_cache(maxsize=4)
    def dataset(index: int) -> dict[str, Any]:
        return load_dataset(sources[index], objects, grippers, overview_faces)

    @app.get("/grasp-data/{index}")
    async def grasp_data(index: int) -> dict[str, Any]:
        if not 0 <= index < len(sources):
            raise HTTPException(404, "Dataset not found")
        try:
            return await run.io_bound(dataset, index)
        except Exception as error:
            logging.exception("Cannot load grasp dataset %s", sources[index])
            raise HTTPException(422, str(error)) from error

    @ui.page("/")
    async def index() -> None:
        ui.colors(primary="#167567", secondary="#d18c36", accent="#167567")
        ui.add_css(STATIC.joinpath("viewer.css").read_text())
        ui.add_body_html("<script>" + STATIC.joinpath("viewer.js").read_text() + "</script>")
        state: dict[str, Any] = {"candidates": [], "index": 0, "playing": False, "ready": False}

        async def command(method: str, *args: Any) -> Any:
            return await ui.run_javascript(
                f"window.graspViewer.{method}(...{json.dumps(args)})", timeout=60
            )

        async def select(value: int) -> None:
            if not state["ready"]:
                return
            value = max(0, min(int(value), len(state["candidates"]) - 1))
            state["index"] = value
            candidate = state["candidates"][value]
            counter.set_text(f"{value + 1:03d} / {len(state['candidates']):03d}")
            candidate_input.set_value(value)
            scrubber.set_value(value)
            previous.set_enabled(value > 0)
            following.set_enabled(value < len(state["candidates"]) - 1)
            pose = candidate["pose_object_tcp_xyz_xyzw"]
            for label, number in zip(position_values, pose[:3], strict=True):
                label.set_text(f"{number * 1000:+.2f}")
            quaternion.set_text("  ".join(f"{v:+.4f}" for v in pose[3:]))
            joint_values.clear()
            with joint_values:
                for name, number in candidate["closed_joint_positions_m"].items():
                    with ui.row().classes("detail-row"):
                        ui.label(name).classes("muted")
                        ui.label(f"{number * 1000:.3f} mm").classes("mono")
            await command("select", value)

        async def mode_changed() -> None:
            if state["ready"]:
                await command("mode", mode.value)
                visible_count.set_text(str(len(state["candidates"]) if mode.value == "all" else 1))

        async def appearance() -> None:
            if state["ready"]:
                await command(
                    "appearance",
                    opacity.value,
                    object_visible.value,
                    axes.value,
                    grid.value,
                    wireframe.value,
                    color_mode.value,
                )

        def toggle_play() -> None:
            state["playing"] = not state["playing"]
            play.props(f"icon={'pause' if state['playing'] else 'play_arrow'}")

        async def tick() -> None:
            if state["ready"] and state["playing"]:
                await select((state["index"] + 1) % len(state["candidates"]))

        async def load() -> None:
            state.update(ready=False, playing=False)
            play.props("icon=play_arrow")
            controls.style("pointer-events: none; opacity: 0.5")
            dataset_select.set_enabled(False)
            loading.set_visibility(True)
            status.set_text("Loading assets")
            try:
                metadata = await command("load", int(dataset_select.value))
                state.update(candidates=metadata["candidates"], ready=True)
                title.set_text(metadata["object"])
                robot_label.set_text(metadata["robot"].upper())
                total_count.set_text(str(len(state["candidates"])))
                source_label.set_text(metadata["provenance"])
                source_label.tooltip(metadata["source"])
                dimensions.set_text(" x ".join(f"{v * 1000:.1f}" for v in metadata["size"]) + " mm")
                candidate_input._props["max"] = len(state["candidates"]) - 1
                candidate_input.update()
                scrubber._props["max"] = max(1, len(state["candidates"]) - 1)
                scrubber.update()
                await select(0)
                await mode_changed()
                await appearance()
                status.set_text("Scene ready")
                controls.style("pointer-events: auto; opacity: 1")
            except Exception as error:
                status.set_text("Load failed")
                ui.notify(str(error), type="negative", timeout=0, close_button=True)
            finally:
                loading.set_visibility(False)
                dataset_select.set_enabled(True)

        with ui.header().classes("app-header"):
            with ui.row().classes("brand"):
                ui.icon("view_in_ar", size="26px").classes("brand-icon")
                ui.label("GraspDataGen").classes("brand-name")
                ui.label("/  Grasp Studio").classes("header-subtitle")
            with ui.row().classes("header-status"):
                ui.element("span").classes("status-dot")
                status = ui.label("Connecting").classes("muted")
        with ui.element("main").classes("workspace"):
            with ui.column().classes("sidebar"):
                ui.label("DATASET").classes("eyebrow")
                dataset_select = (
                    ui.select(
                        {i: f"{p.parent.name} / {p.name}" for i, p in enumerate(sources)},
                        value=0,
                        on_change=load,
                    )
                    .props("outlined dense options-dense")
                    .classes("w-full dataset-select")
                )
                with ui.row().classes("object-heading"):
                    title = ui.label("Grasp scene").classes("object-title")
                    robot_label = ui.label("").classes("robot-badge")
                source_label = ui.label("").classes("muted small")
                with ui.row().classes("stats"):
                    with ui.column():
                        total_count = ui.label("--").classes("stat-value")
                        ui.label("Total grasps").classes("muted small")
                    with ui.column():
                        visible_count = ui.label("--").classes("stat-value")
                        ui.label("Visible").classes("muted small")
                ui.separator()
                with ui.column().classes("controls w-full") as controls:
                    ui.label("DISPLAY").classes("eyebrow")
                    mode = ui.toggle(
                        {"single": "Single grasp", "all": "All grasps"},
                        value="all",
                        on_change=mode_changed,
                    ).props("no-caps unelevated")
                    ui.label("Distribution opacity").classes("control-label")
                    opacity = ui.slider(
                        min=0.05, max=1, step=0.05, value=0.3, on_change=appearance
                    ).props("label")
                    color_mode = (
                        ui.select(
                            {"direction": "Approach direction", "uniform": "Uniform"},
                            value="direction",
                            label="Color by",
                            on_change=appearance,
                        )
                        .props("outlined dense")
                        .classes("w-full")
                    )
                    with ui.row().classes("direction-legend"):
                        for color, label in (
                            ("#e29448", "X"),
                            ("#42b6a0", "Y"),
                            ("#7785cc", "Z"),
                        ):
                            with ui.row().classes("legend-item"):
                                ui.element("span").style(f"background:{color}").classes("swatch")
                                ui.label(label)
                    object_visible = ui.switch("Object", value=True, on_change=appearance)
                    axes = ui.switch("TCP axes", value=False, on_change=appearance)
                    grid = ui.switch("Ground grid", value=True, on_change=appearance)
                    wireframe = ui.switch("Wireframe", value=False, on_change=appearance)
                    ui.separator()
                    with ui.row().classes("detail-row"):
                        ui.label("SELECTED GRASP").classes("eyebrow")
                        candidate_input = (
                            ui.number(value=0, min=0, step=1, precision=0)
                            .props("dense outlined prefix=#")
                            .classes("candidate-input")
                        )
                        candidate_input.on("change", lambda: select(candidate_input.value or 0))
                    ui.label("TCP position / mm").classes("control-label")
                    position_values = []
                    with ui.row().classes("position-grid"):
                        for axis in "XYZ":
                            with ui.column():
                                ui.label(axis).classes("muted small")
                                position_values.append(ui.label("--").classes("mono"))
                    ui.label("Quaternion / xyzw").classes("control-label")
                    quaternion = ui.label("--").classes("mono quaternion")
                    ui.label("Closed joints").classes("control-label")
                    joint_values = ui.column().classes("w-full joint-values")
                with ui.column().classes("asset-footer"):
                    ui.label("OBJECT DIMENSIONS").classes("eyebrow")
                    dimensions = ui.label("--").classes("mono small")
            with ui.column().classes("viewport-area"):
                with ui.row().classes("viewport-toolbar"):
                    with ui.row().classes("toolbar-title"):
                        ui.icon("view_in_ar", size="19px")
                        ui.label("3D workspace")
                    with ui.row().classes("tool-buttons"):
                        for icon, tooltip, view in (
                            ("view_in_ar", "Perspective view", "perspective"),
                            ("vertical_align_top", "Top view", "top"),
                            ("crop_landscape", "Front view", "front"),
                        ):
                            ui.button(icon=icon, on_click=lambda v=view: command("frame", v)).props(
                                "flat round dense"
                            ).tooltip(tooltip)
                        ui.separator().props("vertical")
                        ui.button(
                            icon="center_focus_strong",
                            on_click=lambda: command("frame", "perspective"),
                        ).props("flat round dense").tooltip("Fit scene")
                        ui.button(icon="photo_camera", on_click=lambda: command("snapshot")).props(
                            "flat round dense"
                        ).tooltip("Download image")
                with ui.element("div").classes("scene-wrap"):
                    scene = ui.scene(grid=False, background_color="#edf0f2").classes("main-scene")
                    with ui.column().classes("loading-overlay") as loading:
                        ui.spinner(size="32px")
                        ui.label("Loading scene")
                    with ui.row().classes("viewport-legend"):
                        ui.element("span").classes("swatch selected-swatch")
                        ui.label("Selected grasp")
                    ui.label("OBJECT FRAME  /  METRES").classes("frame-caption")
                with ui.row().classes("timeline"):
                    with ui.row().classes("transport"):
                        previous = (
                            ui.button(
                                icon="skip_previous", on_click=lambda: select(state["index"] - 1)
                            )
                            .props("flat round dense")
                            .tooltip("Previous grasp")
                        )
                        play = (
                            ui.button(icon="play_arrow", on_click=toggle_play)
                            .props("unelevated round")
                            .tooltip("Play / pause")
                        )
                        following = (
                            ui.button(icon="skip_next", on_click=lambda: select(state["index"] + 1))
                            .props("flat round dense")
                            .tooltip("Next grasp")
                        )
                    scrubber = ui.slider(min=0, max=1, step=1, value=0).classes("scrubber")
                    scrubber.on("change", lambda: select(scrubber.value))
                    counter = ui.label("000 / 000").classes("mono counter")
                with ui.row().classes("viewport-footer"):
                    ui.label("Grasp pose inspection")
                    ui.label("XYZ / XYZW  ·  Z-UP")
        scene.on("grasp_pick", lambda event: select(int(event.args)))
        await ui.context.client.connected(timeout=30)
        await ui.run_javascript(
            f"window.graspViewer = await createGraspViewer({scene.id})", timeout=30
        )
        await load()
        ui.timer(0.9, tick)

    ui.run(
        host=host,
        port=port,
        title="GraspDataGen | Grasp Studio",
        reload=False,
        show=False,
        reconnect_timeout=60,
        favicon="◈",
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--grasps",
        type=Path,
        nargs="*",
        default=[],
        help="Compact YAML files; omitted: discover outputs and configured objects",
    )
    parser.add_argument(
        "--objects",
        type=Path,
        default=Path("configs/objects/production.yaml"),
        help="Object configuration for standalone YAML without a manifest",
    )
    parser.add_argument(
        "--grippers",
        type=Path,
        nargs="+",
        default=list(Path("configs/grippers").glob("*.yaml")),
        help="Gripper configurations for standalone YAML without a manifest",
    )
    parser.add_argument(
        "--host", default="127.0.0.1", help="Bind address; override for remote access"
    )
    parser.add_argument("--port", type=int, default=8080, help="HTTP port; override if occupied")
    parser.add_argument(
        "--overview-faces",
        type=int,
        default=3000,
        help="Per-mesh triangle budget in all-grasps mode; single mode is full detail",
    )
    args = parser.parse_args()
    if args.overview_faces < 100:
        parser.error("--overview-faces must be at least 100")
    sources = args.grasps
    if not sources:
        sources = sorted(Path("outputs").rglob("grasps.yaml"))
        sources += [
            o.source.with_name("grasps.yaml")
            for o in load_objects(args.objects)
            if o.source.with_name("grasps.yaml").is_file()
        ]
    sources = list(dict.fromkeys(p.resolve() for p in sources))
    if not sources or any(not p.is_file() for p in sources):
        parser.error("No grasp files found; provide existing compact YAML files with --grasps")
    serve(
        tuple(sources),
        args.objects,
        tuple(args.grippers),
        args.host,
        args.port,
        args.overview_faces,
    )


if __name__ == "__main__":
    main()
