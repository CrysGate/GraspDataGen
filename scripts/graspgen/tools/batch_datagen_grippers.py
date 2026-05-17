#!/usr/bin/env python3
"""
Run datagen.py for multiple configured grippers and summarize the outputs.

Example:
    uv run --locked --no-sync python scripts/graspgen/tools/batch_datagen_grippers.py \
        --object_scales_json objects/datagen_example.json \
        --object_root objects \
        --num_grasps 1024 \
        --max_num_envs 512 \
        --force_magnitude 1.0 \
        --overwrite_existing \
        --min_success_grasps 16
"""

from __future__ import annotations

import argparse
import json
import os
import shlex
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any

import yaml


SCRIPT_DIR = Path(__file__).resolve().parent
GRASPGEN_DIR = SCRIPT_DIR.parent
REPO_ROOT = GRASPGEN_DIR.parent.parent
sys.path.insert(0, str(GRASPGEN_DIR))

from graspgen_utils import predict_grasp_data_filepath  # noqa: E402
from gripper_configurations import GRIPPER_CONFIGS, list_available_grippers  # noqa: E402


DEFAULT_SIM_OUTPUT_FOLDER = Path(os.environ.get("GRASP_DATASET_DIR", "")) / "datagen_sim_data"
DEFAULT_REPORT_DIR = Path("debug_output") / "batch_datagen"


def load_object_scales(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        data = json.load(f)
    if not isinstance(data, dict):
        raise ValueError(f"Expected object scale JSON to be an object: {path}")
    return data


def select_grippers(args: argparse.Namespace) -> list[str]:
    grippers = args.grippers or list_available_grippers()
    unknown = [name for name in grippers if name not in GRIPPER_CONFIGS]
    if unknown:
        raise ValueError(
            f"Unknown gripper config(s): {unknown}. "
            f"Available: {list_available_grippers()}"
        )

    excluded = set(args.exclude_grippers or [])
    unknown_excluded = [name for name in excluded if name not in GRIPPER_CONFIGS]
    if unknown_excluded:
        raise ValueError(
            f"Unknown excluded gripper config(s): {unknown_excluded}. "
            f"Available: {list_available_grippers()}"
        )
    return [name for name in grippers if name not in excluded]


def scale_file_extension_prefix(scale: Any) -> str:
    return "" if scale == 1.0 else f"{scale}"


def output_path_for(
    gripper_config_name: str,
    object_root: Path,
    object_path: str,
    object_scale: Any,
    sim_output_folder: Path,
    device: str,
) -> Path:
    gripper_file = GRIPPER_CONFIGS[gripper_config_name]["gripper_file"]
    gripper_name = Path(str(gripper_file)).stem
    file_name_prefix = "cpu" if device == "cpu" else ""
    full_object_path = object_root / object_path
    predicted = predict_grasp_data_filepath(
        gripper_name,
        str(full_object_path),
        str(sim_output_folder),
        file_name_prefix=file_name_prefix,
        file_extension_prefix=scale_file_extension_prefix(object_scale),
    )
    return Path(predicted)


def count_successes(grasp_file: Path) -> dict[str, Any]:
    if not grasp_file.exists():
        return {
            "status": "missing",
            "file": str(grasp_file),
            "total_grasps": 0,
            "successful_grasps": 0,
        }

    try:
        with grasp_file.open("r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
    except Exception as exc:
        return {
            "status": "read_error",
            "file": str(grasp_file),
            "error": str(exc),
            "total_grasps": 0,
            "successful_grasps": 0,
        }

    grasps = data.get("grasps", {})
    if not isinstance(grasps, dict):
        return {
            "status": "invalid",
            "file": str(grasp_file),
            "error": "YAML does not contain a dict-valued 'grasps' field",
            "total_grasps": 0,
            "successful_grasps": 0,
        }

    total = len(grasps)
    successful = 0
    invalid_confidence_count = 0
    for grasp in grasps.values():
        if not isinstance(grasp, dict):
            continue
        try:
            confidence = float(grasp.get("confidence", 0.0))
        except (TypeError, ValueError):
            invalid_confidence_count += 1
            continue
        if confidence > 0.0:
            successful += 1

    return {
        "status": "ok",
        "file": str(grasp_file),
        "total_grasps": total,
        "successful_grasps": successful,
        "invalid_confidence_count": invalid_confidence_count,
        "object_file": data.get("object_file"),
        "object_scale": data.get("object_scale"),
        "created_with": data.get("created_with"),
        "created_at": data.get("created_at"),
    }


def collect_gripper_stats(
    gripper_config_name: str,
    objects_and_scales: dict[str, Any],
    args: argparse.Namespace,
) -> dict[str, Any]:
    objects: dict[str, Any] = {}
    all_objects_meet_min_success = True
    total_successful_grasps = 0

    for object_path, scale in objects_and_scales.items():
        grasp_file = output_path_for(
            gripper_config_name,
            Path(args.object_root),
            object_path,
            scale,
            Path(args.sim_output_folder),
            args.device,
        )
        object_stats = count_successes(grasp_file)
        object_stats["meets_min_success"] = (
            object_stats["successful_grasps"] >= args.min_success_grasps
        )
        object_stats["requested_scale"] = scale
        objects[object_path] = object_stats
        total_successful_grasps += object_stats["successful_grasps"]
        if not object_stats["meets_min_success"]:
            all_objects_meet_min_success = False

    return {
        "gripper_config": gripper_config_name,
        "gripper_file": GRIPPER_CONFIGS[gripper_config_name].get("gripper_file"),
        "all_objects_meet_min_success": all_objects_meet_min_success,
        "min_success_grasps": args.min_success_grasps,
        "total_successful_grasps": total_successful_grasps,
        "objects": objects,
    }


def build_datagen_command(gripper_config_name: str, args: argparse.Namespace) -> list[str]:
    cmd = ["uv", "run"]
    if not args.allow_uv_lock_update:
        cmd.append("--locked")
    if not args.allow_uv_sync:
        cmd.append("--no-sync")
    cmd.extend([
        "python",
        "scripts/graspgen/datagen.py",
        "--gripper_config",
        gripper_config_name,
        "--object_scales_json",
        args.object_scales_json,
        "--object_root",
        args.object_root,
        "--num_grasps",
        str(args.num_grasps),
        "--max_num_envs",
        str(args.max_num_envs),
        "--force_magnitude",
        str(args.force_magnitude),
        "--sim_output_folder",
        args.sim_output_folder,
    ])
    if args.overwrite_existing:
        cmd.append("--overwrite_existing")
    if args.device:
        cmd.extend(["--device", args.device])
    if args.extra_datagen_args:
        cmd.extend(args.extra_datagen_args)
    return cmd


def run_datagen_for_gripper(
    gripper_config_name: str,
    command: list[str],
    log_dir: Path,
    dry_run: bool,
) -> dict[str, Any]:
    log_dir.mkdir(parents=True, exist_ok=True)
    log_file = log_dir / f"{gripper_config_name}.log"
    started_at = datetime.now().isoformat()
    start_time = time.time()

    if dry_run:
        return {
            "returncode": None,
            "status": "dry_run",
            "started_at": started_at,
            "finished_at": datetime.now().isoformat(),
            "duration_sec": 0.0,
            "command": command,
            "log_file": str(log_file),
        }

    with log_file.open("w", encoding="utf-8") as log:
        log.write("$ " + shlex.join(command) + "\n\n")
        log.flush()
        result = subprocess.run(
            command,
            cwd=REPO_ROOT,
            stdout=log,
            stderr=subprocess.STDOUT,
            check=False,
        )

    return {
        "returncode": result.returncode,
        "status": "completed" if result.returncode == 0 else "failed",
        "started_at": started_at,
        "finished_at": datetime.now().isoformat(),
        "duration_sec": round(time.time() - start_time, 3),
        "command": command,
        "log_file": str(log_file),
    }


def write_summary(summary: dict[str, Any], summary_file: Path) -> None:
    summary_file.parent.mkdir(parents=True, exist_ok=True)
    with summary_file.open("w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2, ensure_ascii=False)
        f.write("\n")


def print_console_summary(summary: dict[str, Any]) -> None:
    print("\nBatch datagen summary")
    print(f"  summary_file: {summary['summary_file']}")
    print(f"  grippers: {summary['totals']['num_grippers']}")
    print(f"  command_successes: {summary['totals']['command_successes']}")
    print(f"  command_failures: {summary['totals']['command_failures']}")
    print(f"  all_objects_ok: {summary['totals']['all_objects_ok']}")
    print("")
    for gripper in summary["grippers"]:
        command_status = gripper["run"]["status"]
        ok = gripper["stats"]["all_objects_meet_min_success"]
        counts = [
            f"{name}={stats['successful_grasps']}"
            for name, stats in gripper["stats"]["objects"].items()
        ]
        print(
            f"  {gripper['gripper_config']}: command={command_status}, "
            f"all_objects_ok={ok}, " + ", ".join(counts)
        )


def make_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Run scripts/graspgen/datagen.py for configured grippers and "
            "write a grasp-count summary."
        )
    )
    parser.add_argument(
        "--grippers",
        nargs="+",
        help="Gripper config names to run. Defaults to all configured grippers.",
    )
    parser.add_argument(
        "--exclude_grippers",
        nargs="+",
        help="Configured grippers to skip.",
    )
    parser.add_argument(
        "--object_scales_json",
        default="objects/datagen_example.json",
        help="Object scale JSON passed to datagen.py.",
    )
    parser.add_argument(
        "--object_root",
        default="objects",
        help="Object root passed to datagen.py.",
    )
    parser.add_argument(
        "--num_grasps",
        type=int,
        default=1024,
        help="Number of candidate grasps passed to datagen.py.",
    )
    parser.add_argument(
        "--max_num_envs",
        type=int,
        default=512,
        help="Maximum IsaacLab env count passed to datagen.py.",
    )
    parser.add_argument(
        "--force_magnitude",
        type=float,
        default=1.0,
        help="Disturbance force magnitude passed to datagen.py.",
    )
    parser.add_argument(
        "--overwrite_existing",
        action="store_true",
        default=False,
        help="Pass --overwrite_existing to datagen.py.",
    )
    parser.add_argument(
        "--sim_output_folder",
        default=str(DEFAULT_SIM_OUTPUT_FOLDER),
        help="Simulation output folder passed to datagen.py and inspected for stats.",
    )
    parser.add_argument(
        "--device",
        default="cuda",
        help="Device passed to datagen.py. Use cpu to match cpu-prefixed output files.",
    )
    parser.add_argument(
        "--min_success_grasps",
        type=int,
        default=16,
        help="Per-object minimum successful grasps required to mark an object as OK.",
    )
    parser.add_argument(
        "--report_dir",
        default=str(DEFAULT_REPORT_DIR),
        help="Directory for logs and the default summary JSON.",
    )
    parser.add_argument(
        "--summary_file",
        help="Summary JSON path. Defaults to report_dir/summary_<timestamp>.json.",
    )
    parser.add_argument(
        "--dry_run",
        action="store_true",
        help="Print/write planned commands and collect current stats without running datagen.py.",
    )
    parser.add_argument(
        "--fail_fast",
        action="store_true",
        help="Stop after the first failed gripper command.",
    )
    parser.add_argument(
        "--fail_on_gripper_failures",
        action="store_true",
        help="Return exit code 1 if any gripper command fails after writing the summary.",
    )
    parser.add_argument(
        "--allow_uv_sync",
        action="store_true",
        help="Allow inner 'uv run' datagen commands to sync/install the project environment.",
    )
    parser.add_argument(
        "--allow_uv_lock_update",
        action="store_true",
        help="Allow inner 'uv run' datagen commands to update uv.lock.",
    )
    parser.add_argument(
        "extra_datagen_args",
        nargs=argparse.REMAINDER,
        help="Extra args forwarded to datagen.py after a '--' separator.",
    )
    return parser


def main() -> int:
    args = make_parser().parse_args()
    if args.extra_datagen_args and args.extra_datagen_args[0] == "--":
        args.extra_datagen_args = args.extra_datagen_args[1:]

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    report_dir = Path(args.report_dir)
    log_dir = report_dir / f"logs_{timestamp}"
    summary_file = Path(args.summary_file or report_dir / f"summary_{timestamp}.json")
    args.object_scales_json = str(Path(args.object_scales_json))
    args.object_root = str(Path(args.object_root))
    args.sim_output_folder = str(Path(args.sim_output_folder))

    objects_and_scales = load_object_scales(Path(args.object_scales_json))
    grippers = select_grippers(args)

    results = []
    for index, gripper_config_name in enumerate(grippers, start=1):
        command = build_datagen_command(gripper_config_name, args)
        print(f"[{index}/{len(grippers)}] {gripper_config_name}")
        print(f"  command: {shlex.join(command)}")
        run_result = run_datagen_for_gripper(
            gripper_config_name,
            command,
            log_dir,
            dry_run=args.dry_run,
        )
        stats = collect_gripper_stats(gripper_config_name, objects_and_scales, args)
        print(f"  status: {run_result['status']}, log: {run_result['log_file']}")
        print(
            "  successful grasps: "
            + ", ".join(
                f"{name}={object_stats['successful_grasps']}"
                for name, object_stats in stats["objects"].items()
            )
        )
        results.append(
            {
                "gripper_config": gripper_config_name,
                "run": run_result,
                "stats": stats,
            }
        )
        if args.fail_fast and run_result["returncode"] not in (0, None):
            break

    command_successes = sum(1 for item in results if item["run"]["returncode"] == 0)
    command_failures = sum(
        1 for item in results if item["run"]["returncode"] not in (0, None)
    )
    all_objects_ok = sum(
        1 for item in results if item["stats"]["all_objects_meet_min_success"]
    )
    summary = {
        "created_at": datetime.now().isoformat(),
        "repo_root": str(REPO_ROOT),
        "summary_file": str(summary_file),
        "settings": {
            "object_scales_json": args.object_scales_json,
            "object_root": args.object_root,
            "num_grasps": args.num_grasps,
            "max_num_envs": args.max_num_envs,
            "force_magnitude": args.force_magnitude,
            "overwrite_existing": args.overwrite_existing,
            "sim_output_folder": args.sim_output_folder,
            "device": args.device,
            "min_success_grasps": args.min_success_grasps,
            "dry_run": args.dry_run,
            "allow_uv_sync": args.allow_uv_sync,
            "allow_uv_lock_update": args.allow_uv_lock_update,
            "extra_datagen_args": args.extra_datagen_args,
        },
        "objects": objects_and_scales,
        "totals": {
            "num_grippers": len(results),
            "command_successes": command_successes,
            "command_failures": command_failures,
            "all_objects_ok": all_objects_ok,
        },
        "grippers": results,
    }
    write_summary(summary, summary_file)
    print_console_summary(summary)
    return 1 if args.fail_on_gripper_failures and command_failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
