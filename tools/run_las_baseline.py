#!/usr/bin/env python3
"""Run CSF baseline profiling on LAS files.

The script reads data/*.las, runs the existing CSF Python binding, writes
ground/non-ground LAS outputs, and emits per-file plus per-run reports.
"""

from __future__ import annotations

import argparse
import copy
import csv
import json
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional

import laspy
import numpy as np

import CSF


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

OPTIMIZATION_FIELDS = [
    "memory_optimized",
    "deterministic_soa",
    "gpu_enabled",
    "gpu_simulation",
    "gpu_classification",
    "gpu_rasterization",
    "gpu_device_id",
    "backend_error",
]

SUMMARY_FIELDS = [
    "input_file",
    "backend",
    "fallback_used",
    "backend_fallback_reason",
    *OPTIMIZATION_FIELDS,
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

DEFAULT_OPTIMIZATION_CONFIG: Dict[str, Dict[str, Any]] = {
    "optimization": {
        "memory_optimized": False,
        "deterministic_soa": False,
        "gpu_enabled": False,
    },
    "gpu": {
        "device_id": 0,
        "simulation": True,
        "classification": True,
        "rasterization": False,
    },
    "validation": {
        "strict_prerequisites": True,
    },
}

CUDA_REBUILD_HINT = (
    "GPU mode needs a CUDA-built _CSF extension. Rebuild in WSL with: "
    "CSF_CUDA_ARCH=sm_89 CSF_ENABLE_CUDA=1 pip install -e . --no-build-isolation --force-reinstall"
)

CPU_ONLY_CUDA_ERROR_MARKERS = (
    b"gpu_enabled requires building with CSF_ENABLE_CUDA=1",
    b"gpu_enabled requires a CUDA build",
)


def ms_since(start: float) -> float:
    return (time.perf_counter() - start) * 1000.0


def parse_scalar(value: str) -> Any:
    value = value.split("#", 1)[0].strip()
    if value.lower() == "true":
        return True
    if value.lower() == "false":
        return False
    if value.lower() in {"null", "none", ""}:
        return None
    try:
        return int(value)
    except ValueError:
        pass
    try:
        return float(value)
    except ValueError:
        pass
    return value.strip("\"'")


def load_simple_yaml(path: Path) -> Dict[str, Dict[str, Any]]:
    data: Dict[str, Dict[str, Any]] = {}
    section = ""
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        if not raw_line.strip() or raw_line.lstrip().startswith("#"):
            continue
        if not raw_line.startswith((" ", "\t")):
            key = raw_line.split("#", 1)[0].strip()
            if not key.endswith(":"):
                raise ValueError(f"Unsupported YAML line in {path}: {raw_line}")
            section = key[:-1].strip()
            data.setdefault(section, {})
            continue
        if not section:
            raise ValueError(f"YAML key without section in {path}: {raw_line}")
        key, sep, value = raw_line.strip().partition(":")
        if not sep:
            raise ValueError(f"Unsupported YAML line in {path}: {raw_line}")
        data.setdefault(section, {})[key.strip()] = parse_scalar(value)
    return data


def load_optimization_config(path: Path) -> Dict[str, Dict[str, Any]]:
    config = copy.deepcopy(DEFAULT_OPTIMIZATION_CONFIG)
    loaded = load_simple_yaml(path)
    for section, values in loaded.items():
        if section not in config:
            config[section] = {}
        config[section].update(values)

    if config["validation"].get("strict_prerequisites", True):
        memory = bool(config["optimization"].get("memory_optimized", False))
        deterministic = bool(config["optimization"].get("deterministic_soa", False))
        gpu_enabled = bool(config["optimization"].get("gpu_enabled", False))
        gpu_simulation = bool(config["gpu"].get("simulation", False))
        gpu_classification = bool(config["gpu"].get("classification", False))
        gpu_rasterization = bool(config["gpu"].get("rasterization", False))

        if deterministic and not memory:
            raise ValueError("optimization config error: deterministic_soa requires memory_optimized=true")
        if gpu_enabled and not deterministic:
            raise ValueError("optimization config error: gpu_enabled requires deterministic_soa=true")
        if (gpu_simulation or gpu_classification or gpu_rasterization) and not gpu_enabled:
            raise ValueError("optimization config error: gpu module flags require gpu_enabled=true")

    return config


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


def preflight_optimization_runtime(config: Dict[str, Dict[str, Any]]) -> None:
    if not bool(config["optimization"].get("gpu_enabled", False)):
        return

    if installed_extension_is_known_cpu_only():
        extension_path = get_csf_extension_path()
        raise RuntimeError(
            "optimization config error: gpu_enabled=true, but the installed CSF extension is CPU-only.\n"
            f"Current extension: {extension_path}\n"
            f"{CUDA_REBUILD_HINT}"
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run CSF on data/*.las and generate baseline performance reports."
    )
    parser.add_argument("--data-dir", type=Path, default=Path("data"))
    parser.add_argument("--output-dir", type=Path, default=Path("output"))
    parser.add_argument("--cloth-resolution", type=float, default=None)
    parser.add_argument("--rigidness", type=int, default=None)
    parser.add_argument("--time-step", type=float, default=None)
    parser.add_argument("--class-threshold", type=float, default=None)
    parser.add_argument("--iterations", type=int, default=None)
    parser.add_argument("--no-slope-smooth", action="store_true")
    parser.add_argument("--backend", choices=("legacy", "soa"), default="legacy")
    parser.add_argument("--optimization-config", type=Path, default=None)
    return parser.parse_args()


def configure_csf(csf: CSF.CSF, args: argparse.Namespace) -> None:
    if args.cloth_resolution is not None:
        csf.params.cloth_resolution = args.cloth_resolution
    if args.rigidness is not None:
        csf.params.rigidness = args.rigidness
    if args.time_step is not None:
        csf.params.time_step = args.time_step
    if args.class_threshold is not None:
        csf.params.class_threshold = args.class_threshold
    if args.iterations is not None:
        csf.params.interations = args.iterations
    if args.no_slope_smooth:
        csf.params.bSloopSmooth = False
    if args.optimization_config is not None:
        config = load_optimization_config(args.optimization_config)
        csf.configureOptimization(
            bool(config["optimization"].get("memory_optimized", False)),
            bool(config["optimization"].get("deterministic_soa", False)),
            bool(config["optimization"].get("gpu_enabled", False)),
            bool(config["gpu"].get("simulation", False)),
            bool(config["gpu"].get("classification", False)),
            bool(config["gpu"].get("rasterization", False)),
            int(config["gpu"].get("device_id", 0)),
        )
    else:
        csf.params.useSoA = args.backend == "soa"


def vecint_to_numpy(indexes: Iterable[int]) -> np.ndarray:
    return np.asarray(list(indexes), dtype=np.int64)


def write_subset_las(source: laspy.LasData, indexes: np.ndarray, path: Path) -> None:
    subset = laspy.LasData(copy.deepcopy(source.header))
    subset.points = source.points[indexes]
    subset.write(path)


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


def write_profile_markdown(profile: Dict[str, Any], path: Path) -> None:
    lines = [
        f"# CSF LAS Profile: {profile['input_file']}",
        "",
        "## Scale",
        "",
        "| Metric | Value |",
        "|---|---:|",
    ]
    for key in [
        "backend",
        "fallback_used",
        "backend_fallback_reason",
        *OPTIMIZATION_FIELDS,
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


def write_summary_markdown(rows: List[Dict[str, Any]], path: Path) -> None:
    total_points = sum(int(row.get("point_count", 0)) for row in rows)
    total_wall_ms = sum(float(row.get("total_wall_ms", 0.0)) for row in rows)
    lines = [
        "# CSF Baseline Summary",
        "",
        f"- Files: {len(rows)}",
        f"- Total points: {total_points}",
        f"- Sum wall time ms: {total_wall_ms:.3f}",
        "",
        "| File | Backend | Points | Particles | Iterations | Ground % | Total ms | Simulation ms | Raster ms | Classify ms |",
        "|---|---|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for row in rows:
        ground_pct = float(row.get("ground_ratio", 0.0)) * 100.0
        values = {
            "input_file": row.get("input_file", ""),
            "backend": row.get("backend", ""),
            "point_count": row.get("point_count", ""),
            "particle_count": row.get("particle_count", ""),
            "iterations_run": row.get("iterations_run", ""),
            "total_wall_ms": float(row.get("total_wall_ms", 0.0)),
            "simulation_ms": float(row.get("simulation_ms", 0.0)),
            "rasterization_ms": float(row.get("rasterization_ms", 0.0)),
            "classification_ms": float(row.get("classification_ms", 0.0)),
        }
        lines.append(
            "| {input_file} | {backend} | {point_count} | {particle_count} | {iterations_run} | "
            "{ground_pct:.2f} | {total_wall_ms:.3f} | {simulation_ms:.3f} | "
            "{rasterization_ms:.3f} | {classification_ms:.3f} |".format(
                ground_pct=ground_pct,
                **values,
            )
        )
    lines.append("")
    lines.extend([
        "## Timing Breakdown",
        "",
        "| File | Backend | Read | Set PC | BBox | Cloth Init | Raster | Simulation | Timestep | Verlet | Constraint | MaxDiff | Collision | Post | Classify | Write | Total Filter | Total Wall |",
        "|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ])
    for row in rows:
        lines.append(
            "| {input_file} | {backend} | {read_las_ms} | {set_point_cloud_ms} | {bounding_box_ms} | "
            "{cloth_init_ms} | {rasterization_ms} | {simulation_ms} | {timestep_ms} | "
            "{verlet_ms} | {constraint_ms} | {maxdiff_ms} | {collision_ms} | {postprocess_ms} | "
            "{classification_ms} | {write_las_ms} | {total_filtering_ms} | {total_wall_ms} |".format(
                input_file=row.get("input_file", ""),
                backend=row.get("backend", ""),
                read_las_ms=format_ms(row.get("read_las_ms")),
                set_point_cloud_ms=format_ms(row.get("set_point_cloud_ms")),
                bounding_box_ms=format_ms(row.get("bounding_box_ms")),
                cloth_init_ms=format_ms(row.get("cloth_init_ms")),
                rasterization_ms=format_ms(row.get("rasterization_ms")),
                simulation_ms=format_ms(row.get("simulation_ms")),
                timestep_ms=format_ms(row.get("timestep_ms")),
                verlet_ms=format_ms(row.get("verlet_ms")),
                constraint_ms=format_ms(row.get("constraint_ms")),
                maxdiff_ms=format_ms(row.get("maxdiff_ms")),
                collision_ms=format_ms(row.get("collision_ms")),
                postprocess_ms=format_ms(row.get("postprocess_ms")),
                classification_ms=format_ms(row.get("classification_ms")),
                write_las_ms=format_ms(row.get("write_las_ms")),
                total_filtering_ms=format_ms(row.get("total_filtering_ms")),
                total_wall_ms=format_ms(row.get("total_wall_ms")),
            )
        )
    lines.append("")
    lines.extend([
        "## Wall-Time Share",
        "",
        "| File | Backend | Read % | Set PC % | BBox % | Cloth Init % | Raster % | Simulation % | Verlet % | Constraint % | MaxDiff % | Collision % | Classify % | Write % |",
        "|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ])
    for row in rows:
        total_wall = row.get("total_wall_ms", 0.0)
        lines.append(
            "| {input_file} | {backend} | {read_las_ms} | {set_point_cloud_ms} | {bounding_box_ms} | "
            "{cloth_init_ms} | {rasterization_ms} | {simulation_ms} | {verlet_ms} | "
            "{constraint_ms} | {maxdiff_ms} | {collision_ms} | {classification_ms} | {write_las_ms} |".format(
                input_file=row.get("input_file", ""),
                backend=row.get("backend", ""),
                read_las_ms=format_pct(row.get("read_las_ms"), total_wall),
                set_point_cloud_ms=format_pct(row.get("set_point_cloud_ms"), total_wall),
                bounding_box_ms=format_pct(row.get("bounding_box_ms"), total_wall),
                cloth_init_ms=format_pct(row.get("cloth_init_ms"), total_wall),
                rasterization_ms=format_pct(row.get("rasterization_ms"), total_wall),
                simulation_ms=format_pct(row.get("simulation_ms"), total_wall),
                verlet_ms=format_pct(row.get("verlet_ms"), total_wall),
                constraint_ms=format_pct(row.get("constraint_ms"), total_wall),
                maxdiff_ms=format_pct(row.get("maxdiff_ms"), total_wall),
                collision_ms=format_pct(row.get("collision_ms"), total_wall),
                classification_ms=format_pct(row.get("classification_ms"), total_wall),
                write_las_ms=format_pct(row.get("write_las_ms"), total_wall),
            )
        )
    lines.append("")
    path.write_text("\n".join(lines), encoding="utf-8")


def process_las(path: Path, output_root: Path, args: argparse.Namespace) -> Dict[str, Any]:
    file_start = time.perf_counter()
    las_output_dir = output_root / path.stem
    las_output_dir.mkdir(parents=True, exist_ok=True)

    read_start = time.perf_counter()
    las = laspy.read(path)
    read_las_ms = ms_since(read_start)

    points = np.column_stack(
        (
            np.asarray(las.x, dtype=np.float64),
            np.asarray(las.y, dtype=np.float64),
            np.asarray(las.z, dtype=np.float64),
        )
    )

    csf = CSF.CSF()
    configure_csf(csf, args)

    set_start = time.perf_counter()
    csf.setPointCloud(points)
    set_point_cloud_ms = ms_since(set_start)

    ground = CSF.VecInt()
    non_ground = CSF.VecInt()
    try:
        csf.do_filtering(ground, non_ground, False)
    except Exception as exc:
        profile_raw = csf.getLastProfileJson()
        profile: Dict[str, Any] = json.loads(profile_raw) if profile_raw else {}
        profile.update(
            {
                "input_file": path.name,
                "input_path": str(path),
                "output_dir": str(las_output_dir),
                "read_las_ms": read_las_ms,
                "set_point_cloud_ms": set_point_cloud_ms,
                "write_las_ms": 0.0,
                "total_wall_ms": ms_since(file_start),
                "backend_error": str(exc),
                "point_count": int(points.shape[0]),
            }
        )
        (las_output_dir / "profile.json").write_text(
            json.dumps(profile, indent=2, ensure_ascii=False), encoding="utf-8"
        )
        write_profile_markdown(profile, las_output_dir / "profile.md")
        raise

    profile_raw = csf.getLastProfileJson()
    profile: Dict[str, Any] = json.loads(profile_raw) if profile_raw else {}

    ground_idx = vecint_to_numpy(ground)
    non_ground_idx = vecint_to_numpy(non_ground)
    if int(ground_idx.size + non_ground_idx.size) != int(points.shape[0]):
        raise RuntimeError(
            f"{path.name}: ground + non_ground does not equal input point count "
            f"({ground_idx.size} + {non_ground_idx.size} != {points.shape[0]})"
        )

    write_start = time.perf_counter()
    write_subset_las(las, ground_idx, las_output_dir / "ground.las")
    write_subset_las(las, non_ground_idx, las_output_dir / "non_ground.las")
    write_las_ms = ms_since(write_start)

    profile.update(
        {
            "input_file": path.name,
            "input_path": str(path),
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

    (las_output_dir / "profile.json").write_text(
        json.dumps(profile, indent=2, ensure_ascii=False), encoding="utf-8"
    )
    write_profile_markdown(profile, las_output_dir / "profile.md")
    return profile


def write_summary(rows: List[Dict[str, Any]], output_root: Path) -> None:
    (output_root / "summary.json").write_text(
        json.dumps(rows, indent=2, ensure_ascii=False), encoding="utf-8"
    )
    with (output_root / "summary.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=SUMMARY_FIELDS, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow(row)
    write_summary_markdown(rows, output_root / "summary.md")


def main() -> int:
    args = parse_args()
    optimization_config = None
    if args.optimization_config is not None:
        args.optimization_config = args.optimization_config.resolve()
        optimization_config = load_optimization_config(args.optimization_config)
        preflight_optimization_runtime(optimization_config)

    data_dir = args.data_dir.resolve()
    output_base = args.output_dir.resolve()

    las_files = sorted(data_dir.glob("*.las"))
    if not las_files:
        raise FileNotFoundError(f"No .las files found in {data_dir}")

    run_name = datetime.now().strftime("%Y%m%d_%H%M")
    output_root = output_base / run_name
    suffix = 1
    while output_root.exists():
        output_root = output_base / f"{run_name}_{suffix:02d}"
        suffix += 1
    output_root.mkdir(parents=True, exist_ok=False)
    if optimization_config is not None:
        (output_root / "optimization_config.json").write_text(
            json.dumps(optimization_config, indent=2, ensure_ascii=False),
            encoding="utf-8",
        )

    rows = []
    for las_path in las_files:
        print(f"[CSF baseline] processing {las_path.name}")
        rows.append(process_las(las_path, output_root, args))

    write_summary(rows, output_root)
    print(f"[CSF baseline] wrote reports to {output_root}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
