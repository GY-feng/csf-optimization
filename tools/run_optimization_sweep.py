#!/usr/bin/env python3
"""Run a no-YAML optimization sweep for CSF.

Edit the constants in the CONFIGURATION section below, then run:

    python tools/run_optimization_sweep.py

The script runs each experiment independently. If one experiment/file fails,
the error is written to the sweep reports and the next experiment continues.
"""

from __future__ import annotations

import copy
import csv
import json
import time
import traceback
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional

import laspy
import numpy as np

import CSF


# =============================================================================
# CONFIGURATION: edit these values before running.
# =============================================================================

DATA_DIR = Path("data")
OUTPUT_DIR = Path("output")
INPUT_PATTERNS = ("*.las",)

CSF_PARAMS = {
    "bSloopSmooth": False,
    "time_step": 0.5,
    "class_threshold": 0.05,
    "cloth_resolution": 0.1,
    "rigidness": 1,
    "iterations": 500,
}

# Writing LAS for every experiment can use a lot of disk. Keep True if you need
# ground.las/non_ground.las for every variant; set False for timing-only sweeps.
WRITE_LAS_OUTPUTS = True

EXPERIMENTS = [
    {
        "name": "00_legacy_baseline",
        "description": "All optimization switches off; legacy baseline.",
        "enabled": True,
        "memory_optimized": False,
        "deterministic_soa": False,
        "gpu_enabled": False,
        "gpu_simulation": False,
        "gpu_classification": False,
        "gpu_rasterization": False,
        "gpu_device_id": 0,
    },
    {
        "name": "01_memory_only",
        "description": "Stage 1 memory optimization only; still legacy backend.",
        "enabled": True,
        "memory_optimized": True,
        "deterministic_soa": False,
        "gpu_enabled": False,
        "gpu_simulation": False,
        "gpu_classification": False,
        "gpu_rasterization": False,
        "gpu_device_id": 0,
    },
    {
        "name": "02_deterministic_soa",
        "description": "Stage 2 CPU SoA backend; correctness-first legacy-equivalent constraint solver.",
        "enabled": True,
        "memory_optimized": True,
        "deterministic_soa": True,
        "gpu_enabled": False,
        "gpu_simulation": False,
        "gpu_classification": False,
        "gpu_rasterization": False,
        "gpu_device_id": 0,
    },
    {
        "name": "03_gpu_simulation",
        "description": "Stage 3 GPU simulation, CPU classification. Disabled until GPU solver matches fixed CPU SoA.",
        "enabled": False,
        "memory_optimized": True,
        "deterministic_soa": True,
        "gpu_enabled": True,
        "gpu_simulation": True,
        "gpu_classification": False,
        "gpu_rasterization": False,
        "gpu_device_id": 0,
    },
    {
        "name": "04_gpu_simulation_classification",
        "description": "Stage 3 GPU simulation and GPU classification. Disabled until GPU solver matches fixed CPU SoA.",
        "enabled": False,
        "memory_optimized": True,
        "deterministic_soa": True,
        "gpu_enabled": True,
        "gpu_simulation": True,
        "gpu_classification": True,
        "gpu_rasterization": False,
        "gpu_device_id": 0,
    },
]

CUDA_REBUILD_HINT = (
    "GPU mode needs a CUDA-built _CSF extension. Rebuild in WSL with: "
    "CSF_CUDA_ARCH=sm_89 CSF_ENABLE_CUDA=1 pip install -e . --no-build-isolation --force-reinstall"
)

CPU_ONLY_CUDA_ERROR_MARKERS = (
    b"gpu_enabled requires building with CSF_ENABLE_CUDA=1",
    b"gpu_enabled requires a CUDA build",
)


TIMING_FIELDS = [
    "read_las_ms",
    "set_point_cloud_ms",
    "bounding_box_ms",
    "cloth_init_ms",
    "rasterization_ms",
    "simulation_ms",
    "timestep_ms",
    "verlet_ms",
    "constraint_ms",
    "maxdiff_ms",
    "collision_ms",
    "postprocess_ms",
    "classification_ms",
    "write_las_ms",
    "total_filtering_ms",
    "total_wall_ms",
]

SUMMARY_FIELDS = [
    "status",
    "experiment",
    "description",
    "input_file",
    "backend",
    "memory_optimized",
    "deterministic_soa",
    "gpu_enabled",
    "gpu_simulation",
    "gpu_classification",
    "gpu_rasterization",
    "gpu_device_id",
    "backend_error",
    "error_type",
    "error_message",
    "point_count",
    "cloth_width",
    "cloth_height",
    "particle_count",
    "iterations_configured",
    "iterations_run",
    "ground_count",
    "non_ground_count",
    "ground_ratio",
    "cloth_resolution",
    "rigidness",
    "time_step",
    "class_threshold",
    "bSloopSmooth",
    *TIMING_FIELDS,
]


def ms_since(start: float) -> float:
    return (time.perf_counter() - start) * 1000.0


def get_csf_extension_path() -> Optional[Path]:
    extension = getattr(CSF, "_CSF", None)
    filename = getattr(extension, "__file__", None)
    if not filename:
        return None
    return Path(filename)


def installed_extension_is_known_cpu_only() -> bool:
    extension_path = get_csf_extension_path()
    if extension_path is None or not extension_path.is_file():
        return False
    try:
        binary = extension_path.read_bytes()
    except OSError:
        return False
    return any(marker in binary for marker in CPU_ONLY_CUDA_ERROR_MARKERS)


def preflight_experiment_runtime(experiment: Dict[str, Any]) -> None:
    if not bool(experiment.get("gpu_enabled", False)):
        return
    if installed_extension_is_known_cpu_only():
        extension_path = get_csf_extension_path()
        raise RuntimeError(
            "gpu_enabled=true, but the installed CSF extension is CPU-only. "
            f"Current extension: {extension_path}. {CUDA_REBUILD_HINT}"
        )


def format_ms(value: Any) -> str:
    if isinstance(value, (int, float)):
        return f"{float(value):.3f}"
    return ""


def format_pct(numerator: Any, denominator: Any) -> str:
    if not isinstance(numerator, (int, float)) or not isinstance(denominator, (int, float)):
        return ""
    if float(denominator) == 0.0:
        return ""
    return f"{float(numerator) / float(denominator) * 100.0:.2f}"


def vecint_to_numpy(indexes: Iterable[int]) -> np.ndarray:
    return np.asarray(list(indexes), dtype=np.int64)


def write_subset_las(source: laspy.LasData, indexes: np.ndarray, path: Path) -> None:
    subset = laspy.LasData(copy.deepcopy(source.header))
    subset.points = source.points[indexes]
    subset.write(path)


def discover_las_files(data_dir: Path) -> List[Path]:
    files: List[Path] = []
    for pattern in INPUT_PATTERNS:
        files.extend(data_dir.glob(pattern))
    return sorted(set(files))


def apply_csf_params(csf: CSF.CSF) -> None:
    csf.params.bSloopSmooth = bool(CSF_PARAMS["bSloopSmooth"])
    csf.params.time_step = float(CSF_PARAMS["time_step"])
    csf.params.class_threshold = float(CSF_PARAMS["class_threshold"])
    csf.params.cloth_resolution = float(CSF_PARAMS["cloth_resolution"])
    csf.params.rigidness = int(CSF_PARAMS["rigidness"])
    csf.params.interations = int(CSF_PARAMS["iterations"])


def configure_optimization(csf: CSF.CSF, experiment: Dict[str, Any]) -> None:
    csf.configureOptimization(
        bool(experiment["memory_optimized"]),
        bool(experiment["deterministic_soa"]),
        bool(experiment["gpu_enabled"]),
        bool(experiment["gpu_simulation"]),
        bool(experiment["gpu_classification"]),
        bool(experiment["gpu_rasterization"]),
        int(experiment["gpu_device_id"]),
    )


def experiment_metadata(experiment: Dict[str, Any]) -> Dict[str, Any]:
    return {
        "experiment": experiment["name"],
        "description": experiment.get("description", ""),
        "memory_optimized": bool(experiment["memory_optimized"]),
        "deterministic_soa": bool(experiment["deterministic_soa"]),
        "gpu_enabled": bool(experiment["gpu_enabled"]),
        "gpu_simulation": bool(experiment["gpu_simulation"]),
        "gpu_classification": bool(experiment["gpu_classification"]),
        "gpu_rasterization": bool(experiment["gpu_rasterization"]),
        "gpu_device_id": int(experiment["gpu_device_id"]),
    }


def write_profile_markdown(profile: Dict[str, Any], path: Path) -> None:
    lines = [
        f"# CSF Sweep Profile: {profile.get('experiment', '')} / {profile.get('input_file', '')}",
        "",
        f"- Status: `{profile.get('status', '')}`",
        f"- Description: {profile.get('description', '')}",
    ]
    if profile.get("status") == "error":
        lines.extend(
            [
                "",
                "## Error",
                "",
                f"- Type: `{profile.get('error_type', '')}`",
                f"- Message: `{profile.get('error_message', '')}`",
            ]
        )

    lines.extend(["", "## Scale", "", "| Metric | Value |", "|---|---:|"])
    for key in [
        "backend",
        "memory_optimized",
        "deterministic_soa",
        "gpu_enabled",
        "gpu_simulation",
        "gpu_classification",
        "gpu_rasterization",
        "backend_error",
        "point_count",
        "cloth_width",
        "cloth_height",
        "particle_count",
        "iterations_configured",
        "iterations_run",
        "ground_count",
        "non_ground_count",
        "ground_ratio",
    ]:
        lines.append(f"| `{key}` | {profile.get(key, '')} |")

    lines.extend(["", "## Parameters", "", "| Parameter | Value |", "|---|---:|"])
    for key in [
        "cloth_resolution",
        "rigidness",
        "time_step",
        "class_threshold",
        "bSloopSmooth",
    ]:
        lines.append(f"| `{key}` | {profile.get(key, '')} |")

    lines.extend(["", "## Timings", "", "| Stage | ms |", "|---|---:|"])
    for key in TIMING_FIELDS:
        lines.append(f"| `{key}` | {format_ms(profile.get(key))} |")

    total_wall = profile.get("total_wall_ms", 0.0)
    lines.extend(["", "## Wall-Time Share", "", "| Stage | % of total wall |", "|---|---:|"])
    for key in TIMING_FIELDS:
        if key == "total_wall_ms":
            continue
        lines.append(f"| `{key}` | {format_pct(profile.get(key), total_wall)} |")
    lines.append("")
    path.write_text("\n".join(lines), encoding="utf-8")


def error_profile(
    experiment: Dict[str, Any],
    las_path: Path,
    output_dir: Path,
    file_start: float,
    exc: BaseException,
    partial_profile: Dict[str, Any] | None = None,
) -> Dict[str, Any]:
    profile = dict(partial_profile or {})
    profile.update(experiment_metadata(experiment))
    profile.update(
        {
            "status": "error",
            "input_file": las_path.name,
            "input_path": str(las_path),
            "output_dir": str(output_dir),
            "error_type": type(exc).__name__,
            "error_message": str(exc),
            "traceback": traceback.format_exc(),
            "total_wall_ms": ms_since(file_start),
        }
    )
    profile.setdefault("backend_error", str(exc))
    return profile


def process_las(experiment: Dict[str, Any], las_path: Path, experiment_dir: Path) -> Dict[str, Any]:
    file_start = time.perf_counter()
    las_output_dir = experiment_dir / las_path.stem
    las_output_dir.mkdir(parents=True, exist_ok=True)

    partial_profile: Dict[str, Any] = {}
    try:
        preflight_experiment_runtime(experiment)

        read_start = time.perf_counter()
        las = laspy.read(las_path)
        read_las_ms = ms_since(read_start)
        partial_profile["read_las_ms"] = read_las_ms

        points = np.column_stack(
            (
                np.asarray(las.x, dtype=np.float64),
                np.asarray(las.y, dtype=np.float64),
                np.asarray(las.z, dtype=np.float64),
            )
        )
        partial_profile["point_count"] = int(points.shape[0])

        csf = CSF.CSF()
        apply_csf_params(csf)
        configure_optimization(csf, experiment)

        set_start = time.perf_counter()
        csf.setPointCloud(points)
        set_point_cloud_ms = ms_since(set_start)
        partial_profile["set_point_cloud_ms"] = set_point_cloud_ms

        ground = CSF.VecInt()
        non_ground = CSF.VecInt()
        csf.do_filtering(ground, non_ground, False)

        profile_raw = csf.getLastProfileJson()
        profile: Dict[str, Any] = json.loads(profile_raw) if profile_raw else {}

        ground_idx = vecint_to_numpy(ground)
        non_ground_idx = vecint_to_numpy(non_ground)
        if int(ground_idx.size + non_ground_idx.size) != int(points.shape[0]):
            raise RuntimeError(
                f"{las_path.name}: ground + non_ground does not equal point_count "
                f"({ground_idx.size} + {non_ground_idx.size} != {points.shape[0]})"
            )

        write_las_ms = 0.0
        if WRITE_LAS_OUTPUTS:
            write_start = time.perf_counter()
            write_subset_las(las, ground_idx, las_output_dir / "ground.las")
            write_subset_las(las, non_ground_idx, las_output_dir / "non_ground.las")
            write_las_ms = ms_since(write_start)

        profile.update(experiment_metadata(experiment))
        profile.update(
            {
                "status": "ok",
                "input_file": las_path.name,
                "input_path": str(las_path),
                "output_dir": str(las_output_dir),
                "read_las_ms": read_las_ms,
                "set_point_cloud_ms": set_point_cloud_ms,
                "write_las_ms": write_las_ms,
                "total_wall_ms": ms_since(file_start),
                "ground_count": int(ground_idx.size),
                "non_ground_count": int(non_ground_idx.size),
                "point_count": int(points.shape[0]),
                "ground_ratio": float(ground_idx.size) / float(points.shape[0])
                if points.shape[0]
                else 0.0,
            }
        )
    except BaseException as exc:
        profile = error_profile(experiment, las_path, las_output_dir, file_start, exc, partial_profile)

    (las_output_dir / "profile.json").write_text(
        json.dumps(profile, indent=2, ensure_ascii=False), encoding="utf-8"
    )
    write_profile_markdown(profile, las_output_dir / "profile.md")
    if profile.get("status") == "error":
        (las_output_dir / "error.txt").write_text(profile.get("traceback", ""), encoding="utf-8")
    return profile


def write_summary_csv(rows: List[Dict[str, Any]], path: Path) -> None:
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=SUMMARY_FIELDS, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def write_summary_markdown(rows: List[Dict[str, Any]], path: Path, title: str) -> None:
    ok_count = sum(1 for row in rows if row.get("status") == "ok")
    error_count = sum(1 for row in rows if row.get("status") == "error")
    lines = [
        f"# {title}",
        "",
        f"- Rows: {len(rows)}",
        f"- OK: {ok_count}",
        f"- Errors: {error_count}",
        f"- WRITE_LAS_OUTPUTS: {WRITE_LAS_OUTPUTS}",
        "",
        "## Compact Summary",
        "",
        "| Experiment | File | Status | Backend | Points | Ground | Non-ground | Total ms | Constraint ms | Error |",
        "|---|---|---|---|---:|---:|---:|---:|---:|---|",
    ]
    for row in rows:
        lines.append(
            "| {experiment} | {input_file} | {status} | {backend} | {point_count} | "
            "{ground_count} | {non_ground_count} | {total_wall_ms} | {constraint_ms} | {error} |".format(
                experiment=row.get("experiment", ""),
                input_file=row.get("input_file", ""),
                status=row.get("status", ""),
                backend=row.get("backend", ""),
                point_count=row.get("point_count", ""),
                ground_count=row.get("ground_count", ""),
                non_ground_count=row.get("non_ground_count", ""),
                total_wall_ms=format_ms(row.get("total_wall_ms")),
                constraint_ms=format_ms(row.get("constraint_ms")),
                error=row.get("error_message", ""),
            )
        )

    lines.extend(
        [
            "",
            "## Timing Breakdown",
            "",
            "| Experiment | File | Status | Read | Set PC | BBox | Cloth Init | Raster | Simulation | Verlet | Constraint | MaxDiff | Collision | Classify | Write | Total Wall |",
            "|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
        ]
    )
    for row in rows:
        lines.append(
            "| {experiment} | {input_file} | {status} | {read_las_ms} | {set_point_cloud_ms} | "
            "{bounding_box_ms} | {cloth_init_ms} | {rasterization_ms} | {simulation_ms} | "
            "{verlet_ms} | {constraint_ms} | {maxdiff_ms} | {collision_ms} | "
            "{classification_ms} | {write_las_ms} | {total_wall_ms} |".format(
                experiment=row.get("experiment", ""),
                input_file=row.get("input_file", ""),
                status=row.get("status", ""),
                read_las_ms=format_ms(row.get("read_las_ms")),
                set_point_cloud_ms=format_ms(row.get("set_point_cloud_ms")),
                bounding_box_ms=format_ms(row.get("bounding_box_ms")),
                cloth_init_ms=format_ms(row.get("cloth_init_ms")),
                rasterization_ms=format_ms(row.get("rasterization_ms")),
                simulation_ms=format_ms(row.get("simulation_ms")),
                verlet_ms=format_ms(row.get("verlet_ms")),
                constraint_ms=format_ms(row.get("constraint_ms")),
                maxdiff_ms=format_ms(row.get("maxdiff_ms")),
                collision_ms=format_ms(row.get("collision_ms")),
                classification_ms=format_ms(row.get("classification_ms")),
                write_las_ms=format_ms(row.get("write_las_ms")),
                total_wall_ms=format_ms(row.get("total_wall_ms")),
            )
        )
    lines.append("")
    path.write_text("\n".join(lines), encoding="utf-8")


def write_outputs(rows: List[Dict[str, Any]], output_root: Path) -> None:
    (output_root / "sweep_summary.json").write_text(
        json.dumps(rows, indent=2, ensure_ascii=False), encoding="utf-8"
    )
    write_summary_csv(rows, output_root / "sweep_summary.csv")
    write_summary_markdown(rows, output_root / "sweep_summary.md", "CSF Optimization Sweep Summary")

    error_rows = [row for row in rows if row.get("status") == "error"]
    (output_root / "error_log.json").write_text(
        json.dumps(error_rows, indent=2, ensure_ascii=False), encoding="utf-8"
    )
    if error_rows:
        write_summary_markdown(error_rows, output_root / "error_log.md", "CSF Optimization Sweep Errors")


def main() -> int:
    data_dir = DATA_DIR.resolve()
    output_base = OUTPUT_DIR.resolve()
    las_files = discover_las_files(data_dir)
    if not las_files:
        raise FileNotFoundError(f"No LAS files found in {data_dir} with patterns {INPUT_PATTERNS}")

    run_name = datetime.now().strftime("sweep_%Y%m%d_%H%M")
    output_root = output_base / run_name
    suffix = 1
    while output_root.exists():
        output_root = output_base / f"{run_name}_{suffix:02d}"
        suffix += 1
    output_root.mkdir(parents=True, exist_ok=False)

    manifest = {
        "data_dir": str(data_dir),
        "output_root": str(output_root),
        "input_patterns": INPUT_PATTERNS,
        "csf_params": CSF_PARAMS,
        "write_las_outputs": WRITE_LAS_OUTPUTS,
        "experiments": EXPERIMENTS,
    }
    (output_root / "sweep_manifest.json").write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False), encoding="utf-8"
    )

    rows: List[Dict[str, Any]] = []
    enabled_experiments = [experiment for experiment in EXPERIMENTS if experiment.get("enabled", True)]
    for experiment in enabled_experiments:
        experiment_dir = output_root / experiment["name"]
        experiment_dir.mkdir(parents=True, exist_ok=True)
        experiment_rows: List[Dict[str, Any]] = []
        print(f"[CSF sweep] experiment {experiment['name']}")

        for las_path in las_files:
            print(f"[CSF sweep]   processing {las_path.name}")
            row = process_las(experiment, las_path, experiment_dir)
            rows.append(row)
            experiment_rows.append(row)
            if row.get("status") == "error":
                print(f"[CSF sweep]   ERROR: {row.get('error_type')}: {row.get('error_message')}")

        (experiment_dir / "summary.json").write_text(
            json.dumps(experiment_rows, indent=2, ensure_ascii=False), encoding="utf-8"
        )
        write_summary_csv(experiment_rows, experiment_dir / "summary.csv")
        write_summary_markdown(experiment_rows, experiment_dir / "summary.md", experiment["name"])
        write_outputs(rows, output_root)

    write_outputs(rows, output_root)
    print(f"[CSF sweep] wrote reports to {output_root}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
