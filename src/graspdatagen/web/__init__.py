"""NiceGUI grasp inspection, with browser rendering and CPU-only asset loading."""

from __future__ import annotations

import argparse
import gzip
import json
import logging
from functools import lru_cache
from pathlib import Path
from typing import Any

import numpy as np
from fastapi import HTTPException, Request, Response
from nicegui import app, run, ui

from graspdatagen.config import load_objects
from graspdatagen.web.data import load_dataset
from graspdatagen.web.file_picker import pick_dataset
from graspdatagen.web.transport import encode_payload

STATIC = Path(__file__).with_name("static")
LOAD_TIMEOUT_SECONDS = 60


def serve(
    sources: tuple[Path, ...],
    objects: Path,
    grippers: tuple[Path, ...],
    host: str,
    port: int,
    overview_faces: int,
    grasp_regions: Path,
) -> None:
    available_sources = list(sources)

    @lru_cache(maxsize=4)
    def dataset(source: Path) -> dict[str, Any]:
        return load_dataset(source, objects, grippers, overview_faces)

    def dataset_response(index: int, annotation: bool, gzip_enabled: bool) -> Response:
        if not 0 <= index < len(available_sources):
            raise HTTPException(404, "Dataset not found")
        try:
            loaded = dataset(available_sources[index])
            if annotation:
                payload = {"annotation_mesh": loaded["annotation_mesh"]}
            else:
                payload = {key: value for key, value in loaded.items() if key != "annotation_mesh"}
                payload["has_annotation"] = loaded["annotation_mesh"] is not None
            content = encode_payload(payload)
            headers = {"Vary": "Accept-Encoding"}
            if gzip_enabled:
                content = gzip.compress(content, compresslevel=1, mtime=0)
                headers["Content-Encoding"] = "gzip"
            # An explicit encoding also bypasses NiceGUI's synchronous gzip
            # middleware, which would otherwise compress on the event loop.
            return Response(content, headers=headers, media_type="application/octet-stream")
        except Exception as error:
            logging.exception("Cannot load grasp dataset %s", available_sources[index])
            raise HTTPException(422, str(error)) from error

    @app.get("/grasp-data/{index}")
    async def grasp_data(index: int, request: Request) -> Response:
        # Both asset loading and serialization run off the event loop so the UI
        # can report progress, handle disconnects and show timeouts during loading.
        return await run.io_bound(
            dataset_response, index, False, "gzip" in request.headers.get("Accept-Encoding", "")
        )

    @app.get("/grasp-data/{index}/annotation")
    async def annotation_data(index: int, request: Request) -> Response:
        return await run.io_bound(
            dataset_response, index, True, "gzip" in request.headers.get("Accept-Encoding", "")
        )

    @ui.page("/")
    async def index() -> None:
        ui.colors(primary="#167567", secondary="#d18c36", accent="#167567")
        ui.add_css(STATIC.joinpath("viewer.css").read_text())
        ui.add_body_html("<script>" + STATIC.joinpath("viewer.js").read_text() + "</script>")
        state: dict[str, Any] = {"candidates": [], "index": 0, "playing": False, "ready": False}

        def dataset_options() -> dict[int, str]:
            return {i: str(path) for i, path in enumerate(available_sources)}

        async def open_dataset(source: Path) -> None:
            # Validate assets before changing the current selection or scene.
            await run.io_bound(dataset, source)
            if source not in available_sources:
                available_sources.append(source)
            selected = available_sources.index(source)
            dataset_select.set_options(dataset_options())
            if dataset_select.value == selected:
                await load()
            else:
                dataset_select.set_value(selected)

        async def browse_dataset() -> None:
            await pick_dataset(
                available_sources[int(dataset_select.value)].parent, open_dataset
            )

        async def javascript(expression: str) -> Any:
            # NiceGUI 2.x does not send rejected JS promises back to Python.
            # Return errors explicitly rather than waiting for its RPC timeout.
            reply = await ui.run_javascript(
                "(async () => { try { return {value: await (" + expression
                + ") ?? null}; } catch (error) { "
                "return {error: error.message || String(error)}; } })()",
                timeout=LOAD_TIMEOUT_SECONDS + 10,
            )
            if "error" in reply:
                raise ValueError(reply["error"])
            return reply["value"]

        async def command(method: str, *args: Any) -> Any:
            return await javascript(f"window.graspViewer.{method}(...{json.dumps(args)})")

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

        async def workspace_changed() -> None:
            if not state["ready"]:
                return
            dataset_select.set_enabled(False)
            dataset_open.set_enabled(False)
            workspace_mode.set_enabled(False)
            loading.set_visibility(True)
            try:
                await command("workspace", workspace_mode.value)
                status.set_text(
                    "Region annotation"
                    if workspace_mode.value == "annotate"
                    else "Scene ready"
                )
            except Exception as error:
                workspace_mode.set_value("preview")
                ui.notify(str(error), type="negative", timeout=0, close_button=True)
            finally:
                loading.set_visibility(False)
                dataset_select.set_enabled(True)
                dataset_open.set_enabled(True)
                workspace_mode.set_enabled(True)

        async def save_region() -> None:
            if not state["ready"]:
                return

            index = int(dataset_select.value)
            loaded = await run.io_bound(dataset, available_sources[index])
            annotation = loaded.get("annotation_mesh")
            if annotation is None:
                ui.notify(
                    "This dataset has no prepared annotation surface",
                    type="negative",
                )
                return

            faces = np.asarray(
                await command("annotationFaces"),
                dtype=np.int64,
            )

            vertices = np.asarray(
                annotation["vertices"], dtype=np.float64
            ).reshape(-1, 3)
            topology = np.asarray(
                annotation["faces"], dtype=np.int64
            ).reshape(-1, 3)

            if faces.size and (
                faces.min() < 0 or faces.max() >= len(topology)
            ):
                ui.notify("Invalid annotated face index", type="negative")
                return

            grasp_regions.mkdir(parents=True, exist_ok=True)
            output = grasp_regions / f"{loaded['object']}.npz"

            np.savez_compressed(
                output,
                allowed_faces=np.unique(faces),
                surface_vertices_m=vertices,
                surface_faces=topology,
            )

            ui.notify(
                f"Saved {len(np.unique(faces))} faces → {output}",
                type="positive",
            )

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
            retry.set_visibility(False)
            play.props("icon=play_arrow")
            controls.style("pointer-events: none; opacity: 0.5")
            dataset_select.set_enabled(False)
            dataset_open.set_enabled(False)
            loading.set_visibility(True)
            status.set_text("Loading assets")
            loading_message.set_text("Loading assets")
            try:
                dataset_index = int(dataset_select.value)
                metadata = await command("load", dataset_index)
                state.update(candidates=metadata["candidates"])

                loaded = await run.io_bound(dataset, available_sources[dataset_index])
                annotation = loaded.get("annotation_mesh")
                if annotation is None:
                    workspace_mode.set_value("preview")
                existing = grasp_regions / f"{loaded['object']}.npz"

                allowed_faces: list[int] = []
                if annotation is not None and existing.is_file():
                    vertices = np.asarray(
                        annotation["vertices"], dtype=np.float64
                    ).reshape(-1, 3)
                    faces = np.asarray(
                        annotation["faces"], dtype=np.int64
                    ).reshape(-1, 3)

                    with np.load(existing, allow_pickle=False) as region:
                        annotated_vertices = np.asarray(
                            region["surface_vertices_m"]
                        )
                        annotated_faces = np.asarray(
                            region["surface_faces"], dtype=np.int64
                        )

                        if not np.array_equal(
                            vertices, annotated_vertices
                        ) or not np.array_equal(
                            faces, annotated_faces
                        ):
                            raise ValueError(
                                f"Region topology mismatch: {existing}"
                            )

                        allowed_faces = (
                            np.asarray(
                                region["allowed_faces"], dtype=np.int64
                            )
                            .reshape(-1)
                            .tolist()
                        )

                await command("setAnnotation", allowed_faces)
                await command("workspace", workspace_mode.value)
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
                state["ready"] = True
                await select(0)
                await mode_changed()
                await appearance()
                status.set_text(
                    "Region annotation" if workspace_mode.value == "annotate" else "Scene ready"
                )
                controls.style("pointer-events: auto; opacity: 1")
            except Exception as error:
                state["ready"] = False
                status.set_text("Load failed")
                retry.set_visibility(True)
                ui.run_javascript("window.graspViewer.cancelLoad()")
                ui.notify(
                    str(error) or "The viewer timed out. Retry or select another file.",
                    type="negative", timeout=0, close_button=True,
                )
            finally:
                loading.set_visibility(False)
                dataset_select.set_enabled(True)
                dataset_open.set_enabled(True)

        async def initialize() -> None:
            loading.set_visibility(True)
            retry.set_visibility(False)
            loading_message.set_text("Initializing 3D viewer")
            try:
                await javascript(
                    "window.graspViewer ? true : "
                    f"!!(window.graspViewer = await createGraspViewer({scene.id}, "
                    f"{LOAD_TIMEOUT_SECONDS * 1000}))"
                )
            except Exception as error:
                status.set_text("Viewer initialization failed")
                loading.set_visibility(False)
                retry.set_visibility(True)
                ui.notify(str(error), type="negative", timeout=0, close_button=True)
                return
            await load()

        with ui.header().classes("app-header"):
            with ui.row().classes("brand"):
                ui.icon("view_in_ar", size="26px").classes("brand-icon")
                ui.label("GraspDataGen").classes("brand-name")
                ui.label("/  Grasp Studio").classes("header-subtitle")
            with ui.row().classes("header-status"):
                ui.element("span").classes("status-dot")
                status = ui.label("Connecting").classes("muted")
                retry = ui.button("Retry", icon="refresh", on_click=initialize).props(
                    "flat no-caps"
                )
                retry.set_visibility(False)
        with ui.element("main").classes("workspace"):
            with ui.column().classes("sidebar"):
                ui.label("DATASET").classes("eyebrow")
                with ui.row().classes("dataset-picker"):
                    dataset_select = (
                        ui.select(dataset_options(), value=0, on_change=load)
                        .props("outlined dense options-dense")
                        .classes("dataset-select")
                    )
                    dataset_select.add_slot("selected-item", '''
                        <span>{{ props.opt.label.split('/').slice(-2).join(' / ') }}
                            <q-tooltip>{{ props.opt.label }}</q-tooltip>
                        </span>
                    ''')
                    dataset_open = (
                        ui.button(icon="folder_open", on_click=browse_dataset)
                        .props("flat round dense")
                        .tooltip("Open grasps YAML")
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
                    ui.label("WORKSPACE").classes("eyebrow")
                    workspace_mode = ui.toggle(
                        {
                            "preview": "Preview",
                            "annotate": "Annotate",
                        },
                        value="preview",
                        on_change=workspace_changed,
                    ).props("no-caps unelevated")

                    (
                        ui.button(
                            "Save region",
                            icon="save",
                            on_click=save_region,
                        )
                        .props("outline no-caps")
                        .classes("w-full")
                    )

                    ui.label(
                        "Annotate: click = paint · Shift+click = erase"
                    ).classes("muted small")

                    ui.separator()
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
                        loading_message = ui.label("Initializing 3D viewer")
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
        scene.on("load_progress", lambda event: loading_message.set_text(event.args))
        await ui.context.client.connected(timeout=30)
        await initialize()
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
        help="Compact YAML files; omitted: discover exports beside configured objects",
    )
    parser.add_argument(
        "--objects",
        type=Path,
        default=Path("configs/objects/our_assets.yaml"),
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
    parser.add_argument(
        "--grasp-regions",
        type=Path,
        default=Path("annotations/grasp_regions"),
        help="Directory containing grasp-region NPZ annotations",
    )
    args = parser.parse_args()
    if args.overview_faces < 100:
        parser.error("--overview-faces must be at least 100")
    sources = args.grasps
    if not sources:
        sources = [
            path
            for o in load_objects(args.objects)
            for path in sorted(o.source.parent.glob("grasps*.yaml"))
            if path.is_file()
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
        args.grasp_regions,
    )


if __name__ == "__main__":
    main()
