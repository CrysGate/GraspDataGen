"""Inspect, prepare, generate, replay and audit USD grasp data with Isaac Sim 6."""

from __future__ import annotations

import argparse
import fcntl
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path
from traceback import print_exc
from typing import Any

from graspdatagen.assets import check_cache, file_hash, inspect_object, write_json
from graspdatagen.config import digest, load_gripper, load_objects, load_run
from graspdatagen.runtime import PhysxRuntime, RuntimeConfig
from graspdatagen.storage import acknowledge_checkpoints, durable_json, export_grasps_yaml


def physics_errors(text: str) -> list[str]:
    return [
        line
        for line in text.splitlines()
        if "[Error]" in line
        and any(name in line for name in ("[omni.physx", "[omni.physics", "[carb.cudainterop"))
    ]


def rendering_errors(text: str) -> list[str]:
    return [
        line
        for line in text.splitlines()
        if "[Error]" in line
        and any(name in line for name in ("[omni.rtx]", "[carb.graphics", "[carb.windowing"))
    ]


def publish_completed_pairs(root: Path) -> None:
    for directory in acknowledge_checkpoints(root):
        manifest_path = directory / "manifest.json"
        manifest = json.loads(manifest_path.read_text())
        manifest["worker_verified"] = True
        durable_json(manifest_path, manifest)
        export_grasps_yaml(directory)


def worker(args: argparse.Namespace) -> None:
    if args.command in ("generate", "replay", "audit"):
        data_worker(args)
        return
    objects = load_objects(args.manifest)
    report: dict[str, Any] = {
        "gate": "P1",
        "worker_pid": os.getpid(),
        "complete": False,
        "objects": [],
    }
    runtime = PhysxRuntime(RuntimeConfig(args.device, 240))
    try:
        from graspdatagen.assets import prepare_object
        from graspdatagen.grippers import prepare_gripper

        if args.command == "inspect":
            report["objects"] = [inspect_object(config) for config in objects]
        else:
            for config in objects:
                report["objects"].append(str(prepare_object(runtime, config, args.cache)))
                write_json(args.output, report)
            gripper = load_gripper(args.gripper)
            if gripper.calibration.steps_per_second != runtime.config.steps_per_second:
                raise ValueError("Calibration step rate must match the runtime")
            prepared = prepare_gripper(runtime, gripper, args.cache)
            report["gripper"] = str(prepared)
            report["cache_checks"] = [check_cache(Path(p))["complete"] for p in report["objects"]]
        report["checks_passed"] = True
        write_json(args.output, report)
    except Exception as error:
        report["error"] = {"type": type(error).__name__, "message": str(error)}
        write_json(args.output, report)
        print_exc()
        runtime.close(exit_code=1)
        raise
    runtime.close()


def data_worker(args: argparse.Namespace) -> None:
    report: dict[str, Any] = {"gate": "P3", "worker_pid": os.getpid(), "complete": False}
    if args.command == "generate":
        config = load_run(args.config)
        device, rate = config.device, config.validation.steps_per_second
    else:
        saved = json.loads((args.run / "manifest.json").read_text())
        device, rate = args.device, saved["protocol"]["steps_per_second"]
    runtime = PhysxRuntime(RuntimeConfig(device, rate), gui=args.gui)
    try:
        if args.command == "generate":
            from graspdatagen.pipeline import generate

            report["combinations"] = generate(runtime, config)
            report["checks_passed"] = all(
                p["status"] in ("target_reached", "insufficient_valid_grasps", "asset_invalid")
                for p in report["combinations"]
            )
            report["all_targets_reached"] = all(
                p["status"] == "target_reached" for p in report["combinations"]
            )
        elif args.command == "replay":
            from graspdatagen.replay import replay

            report.update(replay(runtime, args.run, args.output, args.grasp_id, args.environments))
        else:
            from graspdatagen.audit import audit

            report.update(audit(runtime, args.run, args.config, args.output, args.case))
        write_json(args.output, report)
    except Exception as error:
        report["error"] = {"type": type(error).__name__, "message": str(error)}
        write_json(args.output, report)
        print_exc()
        runtime.close(exit_code=1)
        raise
    runtime.close()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    export = commands.add_parser("export", help="Export verified grasps to compact grasps.yaml")
    export.add_argument("--run", type=Path, required=True)
    view = commands.add_parser(
        "view", help="Inspect static grasp poses from compact YAML in Isaac Sim"
    )
    view.add_argument("--grasps", type=Path, required=True)
    view.add_argument(
        "--candidate", type=int, default=0, help="Zero-based YAML candidate ID; default 0"
    )
    view.add_argument("--device", type=int, default=0, help="GPU ordinal; default 0")
    for name in ("inspect", "prepare"):
        command = commands.add_parser(name)
        command.add_argument("--manifest", type=Path, required=True)
        command.add_argument("--output", type=Path, required=True)
        command.add_argument(
            "--device", type=int, default=0, help="GPU ordinal; default is the P0-qualified GPU 0"
        )
        command.add_argument("--worker", action="store_true", help=argparse.SUPPRESS)
        if name == "prepare":
            command.add_argument("--gripper", type=Path, required=True)
            command.add_argument(
                "--cache",
                type=Path,
                default=Path("outputs/prepared"),
                help="Prepared assets; override to isolate preparation for another dataset",
            )
    generate = commands.add_parser("generate", help="Bounded full-protocol production")
    generate.add_argument("--config", type=Path, required=True)
    generate.add_argument(
        "--resume", action="store_true", help="Continue an identical interrupted run"
    )
    generate.add_argument("--worker", action="store_true", help=argparse.SUPPRESS)
    replay = commands.add_parser("replay", help="Execute saved pregrasp and every trial")
    replay.add_argument("--run", type=Path, required=True)
    replay.add_argument("--output", type=Path, required=True)
    replay.add_argument("--device", type=int, default=0, help="GPU ordinal, normally 0")
    replay.add_argument(
        "--grasp-id", type=int, default=0, help="Saved candidate ID; omit to replay all"
    )
    replay.add_argument(
        "--environments", type=int, default=1, help="Default 1 for independent serial replay"
    )
    replay.add_argument("--worker", action="store_true", help=argparse.SUPPRESS)
    audit = commands.add_parser("audit", help="Run real validator counterexamples and sensitivity")
    audit.add_argument("--run", type=Path, required=True)
    audit.add_argument("--config", type=Path, required=True)
    audit.add_argument("--output", type=Path, required=True)
    audit.add_argument("--device", type=int, default=0, help="GPU ordinal, normally 0")
    from graspdatagen.audit import CASES

    audit.add_argument(
        "--case",
        choices=("all", *CASES),
        default="all",
        help="Default all includes sensitivity; select one fixture to rerun a failed audit",
    )
    audit.add_argument("--worker", action="store_true", help=argparse.SUPPRESS)
    for command in (generate, replay, audit):
        command.add_argument(
            "--gui", action="store_true", help="Watch all parallel trials in a live viewport"
        )
    args = parser.parse_args()
    if args.command == "view":
        if not os.environ.get("DISPLAY"):
            parser.error("view requires DISPLAY; run from a local or remote graphical desktop")
        from graspdatagen.viewer import view_grasps

        view_grasps(args.grasps, args.device, args.candidate)
        return
    if args.command == "export":
        print(export_grasps_yaml(args.run))
        return
    if (
        args.command in ("generate", "replay", "audit")
        and args.gui
        and not os.environ.get("DISPLAY")
    ):
        parser.error(
            "--gui requires DISPLAY; run from a local or remote graphical desktop terminal"
        )
    lock_fds: tuple[int, ...] = ()
    if args.command == "generate":
        config = load_run(args.config)
        args.output = config.output / "report.json"
        if not args.worker:
            existed = config.output.exists()
            config.output.mkdir(parents=True, exist_ok=True)
            descriptor = os.open(config.output / ".writer.lock", os.O_CREAT | os.O_RDWR, 0o600)
            try:
                fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                os.close(descriptor)
                raise SystemExit(f"Output is locked by another writer: {config.output}") from None
            lock_fds = (descriptor,)
            if existed and not args.resume:
                raise SystemExit("Output exists; use --resume with the identical configuration")
            inputs = [config.manifest, *config.grippers]
            inputs.extend(load_gripper(path).robot_config for path in config.grippers)
            identity = {
                "config": config.snapshot,
                "inputs": {str(path): file_hash(path) for path in inputs},
                "implementation": {
                    p.name: file_hash(p) for p in Path(__file__).parent.glob("*.py")
                },
            }
            run_path = config.output / "run.json"
            if run_path.exists():
                if json.loads(run_path.read_text())["identity"] != digest(identity):
                    raise SystemExit("Resume rejected: configuration or implementation changed")
            else:
                if args.resume:
                    raise SystemExit("No initialized run exists to resume")
                durable_json(run_path, {"identity": digest(identity), **identity})
            # Pending transactions belong to the previous worker, whose final batch
            # did not receive log verification. Its committed manifest is authoritative.
            for transaction in config.output.glob("*/transaction.json"):
                transaction.unlink()
    if args.command == "replay" and (args.environments < 1 or args.grasp_id < 0):
        raise ValueError("Replay environments must be positive and grasp ID nonnegative")
    if args.worker:
        worker(args)
        return
    gate = "P3" if args.command in ("generate", "replay", "audit") else "P1"
    write_json(args.output, {"gate": gate, "complete": False, "status": "starting"})
    log = args.output.with_suffix(".worker.log")
    if log.exists():
        index = len(list(log.parent.glob(log.stem + ".previous-*.log")))
        log.rename(log.with_name(log.stem + f".previous-{index:03d}.log"))
    print(f"{gate}: {args.command}; log={log}", flush=True)
    started = time.monotonic()
    with log.open("w") as stream:
        process = subprocess.Popen(
            [
                sys.executable,
                "-u",
                "-X",
                "faulthandler",
                "-m",
                "graspdatagen.cli",
                *sys.argv[1:],
                "--worker",
            ],
            stdout=stream,
            stderr=subprocess.STDOUT,
            pass_fds=lock_fds,
            env={**os.environ, "GRASPDATAGEN_SUPERVISOR_PID": str(os.getpid())},
        )

        def stop_worker(signum: int, frame: object) -> None:
            process.send_signal(signum)

        signal.signal(signal.SIGTERM, stop_worker)
        signal.signal(signal.SIGINT, stop_worker)
        with log.open() as monitor:
            partial_line = ""
            while process.poll() is None:
                complete_lines, _, partial_line = (partial_line + monitor.read()).rpartition("\n")
                if physics_errors(complete_lines) or rendering_errors(complete_lines):
                    process.terminate()
                    process.wait()
                    break
                if args.command == "generate" and not partial_line:
                    publish_completed_pairs(args.output.parent)
                try:
                    process.wait(timeout=1)
                except subprocess.TimeoutExpired:
                    pass
    report = json.loads(args.output.read_text())
    report["worker_pid"] = process.pid
    report["worker_exit_code"] = process.returncode
    report["worker_elapsed_s"] = time.monotonic() - started
    report["worker_log"] = str(log)
    report["physics_errors"] = physics_errors(log.read_text(errors="replace"))
    report["rendering_errors"] = rendering_errors(log.read_text(errors="replace"))
    pids = subprocess.check_output(
        ["nvidia-smi", "--query-compute-apps=pid", "--format=csv,noheader,nounits"], text=True
    ).split()
    report["worker_gpu_released"] = str(report.get("worker_pid")) not in pids
    report["complete"] = (
        process.returncode == 0
        and report.get("checks_passed") is True
        and not report["physics_errors"]
        and not report["rendering_errors"]
        and report["worker_gpu_released"]
    )
    if args.command == "generate":
        verified = (
            process.returncode == 0
            and not report["physics_errors"]
            and not report["rendering_errors"]
            and report["worker_gpu_released"]
        )
        for pair in args.output.parent.glob("*/manifest.json"):
            manifest = json.loads(pair.read_text())
            if verified:
                manifest["worker_verified"] = True
                durable_json(pair, manifest)
                export_grasps_yaml(pair.parent)
    write_json(args.output, report)
    if args.command == "generate":
        durable_json(args.output.parent / "attempts" / f"worker-{process.pid}.json", report)
    print(f"{gate}: complete={report['complete']}; report={args.output}", flush=True)
    if not report["complete"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
