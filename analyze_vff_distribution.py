"""
High-level experiment distribution analyzer for CNC data-collection CSVs.

The data-generation executable reads one row per experiment. This script reads
the same CSV format and reports thesis-style approximate distributions for:

  * VFF signal families
  * VFF generator enum values
  * noise types
  * trajectory source/types
  * optional G-code move counts when G-code files are available
  * optional VFF sequence signal statistics when external .npy files exist

Examples:
    python analyze_vff_distribution.py experiments.csv
    python analyze_vff_distribution.py clean.csv noisy.csv --out-dir reports
    python analyze_vff_distribution.py comp_v2.csv --inspect-sequences
    python analyze_vff_distribution.py comp_v2.csv --comp-vff-table
"""

from __future__ import annotations

import argparse
import csv
import math
import re
import statistics
from collections import Counter, defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


FALLBACK_COLUMNS = [
    "experiment_id",
    "family_id",
    "output_directory",
    "num_trajectories",
    "trajectory_type",
    "noise_type",
    "noise_min_amplitude",
    "noise_max_amplitude",
    "noise_min_freq",
    "noise_max_freq",
    "noise_min_sines",
    "noise_max_sines",
    "noise_sparse_prob",
    "vff_type",
    "vff_mean_dwell",
    "vff_min_dwell",
    "vff_max_amplitude",
    "vff_max_freq",
    "reserved",
    "master_seed",
    "gcode_seed",
    "noise_seed",
    "vff_seed",
    "deviation_sequence_file",
    "vff_sequence_file",
    "gcode_file_path",
]

HEADER_ALIASES = {
    # Some old CSV headers kept the original DC/sparse names after the parser
    # changed these positions to dwell parameters.
    "vff_min_dc": "vff_mean_dwell",
    "vff_max_dc": "vff_min_dwell",
}

TRAJECTORY_TYPES = {
    "0": "Linear only",
    "1": "Circular only",
    "2": "Mixed",
    "3": "Existing G-code",
}

NOISE_TYPES = {
    "0": "Filtered Gaussian",
    "1": "Superimposed sines",
    "2": "Sparse impulse",
    "3": "None / clean",
    "4": "Existing sequence",
}

VFF_TYPES = {
    "0": "None / baseline",
    "1": "Square wave",
    "2": "Smooth ramp",
    "3": "Sum of sinusoids",
    "4": "Existing sequence",
}

VFF_NAME_PATTERNS = [
    (re.compile(r"(no[_-]?vff|none|zero|baseline)", re.I), "None / baseline"),
    (re.compile(r"(square|step)", re.I), "Square wave"),
    (re.compile(r"(smooth[_-]?ramp|ramp|transition)", re.I), "Smooth ramp"),
    (re.compile(r"(sum[_-]?of[_-]?sin|sinusoid|sine)", re.I), "Sum of sinusoids"),
    (re.compile(r"(gauss|gaussian|normal)", re.I), "Filtered Gaussian"),
    (re.compile(r"(sparse|impulse|spike)", re.I), "Sparse impulse"),
    (re.compile(r"(dc|offset)", re.I), "DC offset"),
    (re.compile(r"(lstm|gru|model)", re.I), "Model-generated VFF"),
    (re.compile(r"(direct|optimization|optimisation)", re.I), "None / baseline"),
]

GCODE_MOVE_RE = re.compile(r"(?<![A-Z0-9.])(G0?0|G0?1|G0?2|G0?3)\b", re.I)
GCODE_PLANE_RE = re.compile(r"(?<![A-Z0-9.])(G1[789])\b", re.I)
GCODE_FEED_RE = re.compile(r"(?<![A-Z0-9.])F([-+]?\d+(?:\.\d+)?)", re.I)


@dataclass
class ExperimentRow:
    source_csv: Path
    row_number: int
    data: dict[str, str]

    def text(self, name: str, default: str = "") -> str:
        return self.data.get(name, default).strip()

    def number(self, name: str, default: float = 0.0) -> float:
        raw = self.text(name)
        if not raw:
            return default
        try:
            return float(raw)
        except ValueError:
            return default

    def integer(self, name: str, default: int = 0) -> int:
        return int(round(self.number(name, default)))

    @property
    def weight(self) -> int:
        return max(1, self.integer("num_trajectories", 1))


@dataclass
class GCodeStats:
    total_moves: int
    move_counts: Counter[str]
    plane_counts: Counter[str]
    feedrates: list[float]


@dataclass
class SequenceStats:
    path: Path
    label: str
    rows: int
    max_abs: float
    rms: float
    fd_rms: float
    nonzero_fraction: float


def normalize_header(name: str) -> str:
    spaced = re.sub(r"(?<=[a-z0-9])(?=[A-Z])", "_", name.strip())
    return spaced.lower().replace(" ", "_").replace("-", "_")


def label_from_code(mapping: dict[str, str], value: str, fallback: str) -> str:
    key = value.strip()
    if key.endswith(".0"):
        key = key[:-2]
    return mapping.get(key, f"{fallback} {value}" if value else f"{fallback} missing")


def read_experiment_csv(path: Path) -> list[ExperimentRow]:
    rows: list[ExperimentRow] = []
    with path.open("r", newline="", encoding="utf-8-sig") as fh:
        sample = fh.readline()
        fh.seek(0)

        first_fields = next(csv.reader([sample]))
        normalized_first = [normalize_header(c) for c in first_fields]
        has_named_header = any(c in normalized_first for c in FALLBACK_COLUMNS)

        if has_named_header:
            reader = csv.DictReader(fh)
            for row_number, row in enumerate(reader, start=2):
                if not row or is_comment_or_blank(row.values()):
                    continue
                normalized = {}
                for k, v in row.items():
                    if not k:
                        continue
                    name = normalize_header(k)
                    normalized[HEADER_ALIASES.get(name, name)] = v or ""
                rows.append(ExperimentRow(path, row_number, normalized))
        else:
            reader = csv.reader(fh)
            for row_number, values in enumerate(reader, start=1):
                if not values or is_comment_or_blank(values):
                    continue
                padded = values + [""] * max(0, len(FALLBACK_COLUMNS) - len(values))
                rows.append(
                    ExperimentRow(
                        path,
                        row_number,
                        dict(zip(FALLBACK_COLUMNS, padded[: len(FALLBACK_COLUMNS)])),
                    )
                )

    return rows


def is_comment_or_blank(values: Iterable[str]) -> bool:
    vals = [str(v).strip() for v in values]
    return not any(vals) or vals[0].startswith("#")


def classify_vff_family(row: ExperimentRow) -> str:
    enum_label = label_from_code(VFF_TYPES, row.text("vff_type"), "VFF type")
    if enum_label != "Existing sequence":
        return enum_label

    sequence_path = row.text("vff_sequence_file")
    if not sequence_path:
        return enum_label

    name_blob = Path(sequence_path).as_posix().replace("/", " ")
    for pattern, label in VFF_NAME_PATTERNS:
        if pattern.search(name_blob):
            return label
    return "Existing sequence (unclassified)"


def resolve_input_path(raw_path: str, csv_path: Path) -> Path | None:
    if not raw_path:
        return None

    path = Path(raw_path)
    candidates = [path]
    if not path.is_absolute():
        candidates.append(csv_path.parent / path)

    for candidate in candidates:
        if candidate.exists():
            return candidate
    return candidates[-1]


def parse_gcode_file(path: Path) -> GCodeStats:
    move_counts: Counter[str] = Counter()
    plane_counts: Counter[str] = Counter()
    feedrates: list[float] = []
    current_plane = "G17"

    with path.open("r", encoding="utf-8", errors="ignore") as fh:
        for raw_line in fh:
            line = strip_gcode_comment(raw_line).upper()
            if not line:
                continue

            plane_match = GCODE_PLANE_RE.search(line)
            if plane_match:
                current_plane = plane_match.group(1).replace("G0", "G")
                plane_counts[current_plane] += 0

            for feed_match in GCODE_FEED_RE.finditer(line):
                try:
                    feedrates.append(float(feed_match.group(1)))
                except ValueError:
                    pass

            move_match = GCODE_MOVE_RE.search(line)
            if not move_match:
                continue

            move = move_match.group(1).replace("G00", "G0").replace("G01", "G1")
            move = move.replace("G02", "G2").replace("G03", "G3")
            move_counts[move] += 1
            if move in {"G2", "G3"}:
                plane_counts[current_plane] += 1

    return GCodeStats(sum(move_counts.values()), move_counts, plane_counts, feedrates)


def strip_gcode_comment(line: str) -> str:
    line = line.split(";", 1)[0]
    return re.sub(r"\([^)]*\)", "", line).strip()


def load_sequence_file(path: Path):
    import numpy as np

    if path.is_file():
        arr = np.load(path, mmap_mode="r")
        return np.asarray(arr)

    if path.is_dir():
        chunks = []
        for child in sorted(path.glob("*.npy"), key=numeric_path_key):
            chunks.append(np.asarray(np.load(child, mmap_mode="r")))
        if not chunks:
            raise ValueError(f"no .npy files found in {path}")
        return np.concatenate(chunks, axis=0)

    raise FileNotFoundError(path)


def numeric_path_key(path: Path) -> tuple[int, str]:
    try:
        return int(path.stem), path.name
    except ValueError:
        return 10**12, path.name


def summarize_sequence(path: Path, label: str) -> SequenceStats:
    import numpy as np

    arr = load_sequence_file(path)
    if arr.ndim == 1:
        arr = arr.reshape(-1, 1)
    if arr.ndim != 2:
        raise ValueError(f"expected 1D or 2D array, got shape {arr.shape}")

    values = arr.astype("float64", copy=False)
    max_abs = float(np.max(np.abs(values))) if values.size else 0.0
    rms = float(np.sqrt(np.mean(values * values))) if values.size else 0.0
    nonzero_fraction = float(np.mean(np.abs(values) > 1e-12)) if values.size else 0.0
    if values.shape[0] > 1:
        diff = np.diff(values, axis=0)
        fd_rms = float(np.sqrt(np.mean(diff * diff)))
    else:
        fd_rms = 0.0

    return SequenceStats(
        path=path,
        label=label,
        rows=int(values.shape[0]),
        max_abs=max_abs,
        rms=rms,
        fd_rms=fd_rms,
        nonzero_fraction=nonzero_fraction,
    )


def add_weighted(counter: Counter[str], label: str, weight: int) -> None:
    counter[label] += weight


def pct(count: float, total: float) -> str:
    if total == 0:
        return "0.0%"
    return f"{100.0 * count / total:.1f}%"


def markdown_table(headers: list[str], rows: list[list[object]]) -> str:
    out = []
    out.append("| " + " | ".join(headers) + " |")
    out.append("| " + " | ".join(["---"] * len(headers)) + " |")
    for row in rows:
        out.append("| " + " | ".join(str(v) for v in row) + " |")
    return "\n".join(out)


def counter_rows(counter: Counter[str], total: float) -> list[list[object]]:
    return [[label, format_count(count), pct(count, total)] for label, count in counter.most_common()]


def format_count(value: float) -> str:
    if abs(value - round(value)) < 1e-9:
        return f"{int(round(value)):,}"
    return f"{value:,.1f}"


def numeric_summary(values: list[float]) -> str:
    vals = [v for v in values if math.isfinite(v)]
    if not vals:
        return "n/a"
    if len(vals) == 1:
        return f"{vals[0]:g}"
    return (
        f"min {min(vals):g}, median {statistics.median(vals):g}, "
        f"max {max(vals):g}"
    )


def build_report(
    rows: list[ExperimentRow],
    *,
    inspect_sequences: bool,
    parse_gcode: bool,
) -> tuple[str, dict[str, list[dict[str, object]]]]:
    vff_family_counts: Counter[str] = Counter()
    vff_enum_counts: Counter[str] = Counter()
    noise_counts: Counter[str] = Counter()
    trajectory_counts: Counter[str] = Counter()
    gcode_move_counts: Counter[str] = Counter()
    gcode_plane_counts: Counter[str] = Counter()
    feedrates: list[float] = []
    vff_params: dict[str, dict[str, list[float]]] = defaultdict(lambda: defaultdict(list))
    sequence_stats: list[SequenceStats] = []
    sequence_failures: list[str] = []

    total_weight = 0
    seen_sequences: set[Path] = set()
    seen_gcode: set[Path] = set()

    for row in rows:
        weight = row.weight
        total_weight += weight

        vff_enum = label_from_code(VFF_TYPES, row.text("vff_type"), "VFF type")
        vff_family = classify_vff_family(row)
        noise_label = label_from_code(NOISE_TYPES, row.text("noise_type"), "Noise type")
        trajectory_label = label_from_code(
            TRAJECTORY_TYPES, row.text("trajectory_type"), "Trajectory type"
        )

        add_weighted(vff_enum_counts, vff_enum, weight)
        add_weighted(vff_family_counts, vff_family, weight)
        add_weighted(noise_counts, noise_label, weight)
        add_weighted(trajectory_counts, trajectory_label, weight)

        collect_vff_params(row, vff_family, vff_params)

        if parse_gcode:
            gcode_path = resolve_input_path(row.text("gcode_file_path"), row.source_csv)
            if gcode_path and gcode_path.exists() and gcode_path not in seen_gcode:
                seen_gcode.add(gcode_path)
                stats = parse_gcode_file(gcode_path)
                gcode_move_counts.update(stats.move_counts)
                gcode_plane_counts.update(stats.plane_counts)
                feedrates.extend(stats.feedrates)

        if inspect_sequences:
            sequence_path = resolve_input_path(row.text("vff_sequence_file"), row.source_csv)
            if not sequence_path or sequence_path in seen_sequences:
                continue
            seen_sequences.add(sequence_path)
            try:
                if sequence_path.exists():
                    sequence_stats.append(summarize_sequence(sequence_path, vff_family))
                else:
                    sequence_failures.append(f"missing: {sequence_path}")
            except Exception as exc:  # noqa: BLE001 - report and keep scanning
                sequence_failures.append(f"{sequence_path}: {exc}")

    report_lines = [
        "# Experiment Distribution Summary",
        "",
        f"- CSV files: {len({row.source_csv for row in rows}):,}",
        f"- Experiment rows: {len(rows):,}",
        f"- Weighted trajectory/run assignments: {total_weight:,}",
        "",
        "## VFF Signal Families",
        "",
        markdown_table(
            ["VFF family", "Weighted count", "%"],
            counter_rows(vff_family_counts, total_weight),
        ),
        "",
        "## VFF Generator Enums",
        "",
        markdown_table(
            ["VFF enum", "Weighted count", "%"],
            counter_rows(vff_enum_counts, total_weight),
        ),
        "",
        "## Noise Types",
        "",
        markdown_table(
            ["Noise type", "Weighted count", "%"],
            counter_rows(noise_counts, total_weight),
        ),
        "",
        "## Trajectory Sources",
        "",
        markdown_table(
            ["Trajectory source/type", "Weighted count", "%"],
            counter_rows(trajectory_counts, total_weight),
        ),
    ]

    parameter_rows = build_parameter_rows(vff_params)
    if parameter_rows:
        report_lines.extend(
            [
                "",
                "## VFF Parameter Ranges",
                "",
                markdown_table(
                    ["VFF family", "max amplitude", "mean dwell", "min dwell", "max freq"],
                    parameter_rows,
                ),
            ]
        )

    if parse_gcode and gcode_move_counts:
        move_total = sum(gcode_move_counts.values())
        plane_total = sum(gcode_plane_counts.values())
        report_lines.extend(
            [
                "",
                "## Parsed G-Code Moves",
                "",
                markdown_table(
                    ["Move", "Count", "%"], counter_rows(gcode_move_counts, move_total)
                ),
                "",
                "## Parsed Arc Planes",
                "",
                markdown_table(
                    ["Plane", "Arc count", "%"],
                    counter_rows(gcode_plane_counts, plane_total),
                ),
                "",
                f"Feedrate summary: {numeric_summary(feedrates)} mm/min",
            ]
        )

    if inspect_sequences:
        report_lines.extend(sequence_report(sequence_stats, sequence_failures))

    tables = {
        "vff_distribution": [
            {"label": label, "weighted_count": count, "percent": pct(count, total_weight)}
            for label, count in vff_family_counts.most_common()
        ],
        "vff_enum_distribution": [
            {"label": label, "weighted_count": count, "percent": pct(count, total_weight)}
            for label, count in vff_enum_counts.most_common()
        ],
        "noise_distribution": [
            {"label": label, "weighted_count": count, "percent": pct(count, total_weight)}
            for label, count in noise_counts.most_common()
        ],
        "trajectory_distribution": [
            {"label": label, "weighted_count": count, "percent": pct(count, total_weight)}
            for label, count in trajectory_counts.most_common()
        ],
        "vff_sequence_stats": [
            {
                "path": str(stat.path),
                "label": stat.label,
                "rows": stat.rows,
                "max_abs": stat.max_abs,
                "rms": stat.rms,
                "fd_rms": stat.fd_rms,
                "nonzero_fraction": stat.nonzero_fraction,
            }
            for stat in sequence_stats
        ],
    }

    return "\n".join(report_lines) + "\n", tables


def vff_type_code(row: ExperimentRow) -> str:
    raw = row.text("vff_type")
    if raw.endswith(".0"):
        raw = raw[:-2]
    return raw


def group_label(row: ExperimentRow, group_column: str | None) -> str:
    if group_column:
        value = row.text(group_column)
        if value:
            return value

    family_id = row.text("family_id")
    experiment_id = row.text("experiment_id")
    if family_id and experiment_id and family_id != experiment_id:
        return f"{family_id}/{experiment_id}"
    if experiment_id:
        return experiment_id
    if family_id:
        return family_id
    return f"{row.source_csv.name}:row{row.row_number}"


def numeric_values(rows: list[ExperimentRow], column: str) -> list[float]:
    values = []
    for row in rows:
        raw = row.text(column)
        if not raw:
            continue
        try:
            value = float(raw)
        except ValueError:
            continue
        if math.isfinite(value):
            values.append(value)
    return values


def min_or_blank(values: list[float]) -> float | str:
    return min(values) if values else ""


def max_or_blank(values: list[float]) -> float | str:
    return max(values) if values else ""


def range_or_blank(values: list[float]) -> str:
    if not values:
        return "n/a"
    lo = min(values)
    hi = max(values)
    if lo == hi:
        return f"{lo:g}"
    return f"{lo:g} to {hi:g}"


def interval_or_blank(lower_values: list[float], upper_values: list[float]) -> str:
    if not lower_values and not upper_values:
        return "n/a"
    lo = min(lower_values) if lower_values else min(upper_values)
    hi = max(upper_values) if upper_values else max(lower_values)
    if lo == hi:
        return f"{lo:g}"
    return f"{lo:g} to {hi:g}"


def comp_vff_bounds(type_code: str, rows: list[ExperimentRow]) -> tuple[str, dict[str, object]]:
    details = {
        "amplitude_lower": "",
        "amplitude_upper": "",
        "mean_dwell_lower": "",
        "mean_dwell_upper": "",
        "min_dwell_lower": "",
        "min_dwell_upper": "",
        "frequency_lower": "",
        "frequency_upper": "",
        "sine_count_lower": "",
        "sine_count_upper": "",
    }

    if type_code == "0":
        return "none", details

    if type_code in {"1", "2"}:
        max_amplitudes = numeric_values(rows, "vff_max_amplitude")
        mean_dwell = numeric_values(rows, "vff_mean_dwell")
        min_dwell = numeric_values(rows, "vff_min_dwell")

        if max_amplitudes:
            details["amplitude_lower"] = 0.0
            details["amplitude_upper"] = max(max_amplitudes)
        details["mean_dwell_lower"] = min_or_blank(mean_dwell)
        details["mean_dwell_upper"] = max_or_blank(mean_dwell)
        details["min_dwell_lower"] = min_or_blank(min_dwell)
        details["min_dwell_upper"] = max_or_blank(min_dwell)

        parts = [
            f"amplitude magnitude 0 to {max(max_amplitudes):g}" if max_amplitudes else "amplitude n/a",
            f"mean dwell {range_or_blank(mean_dwell)} samples",
            f"minimum dwell {range_or_blank(min_dwell)} samples",
        ]
        return "; ".join(parts), details

    if type_code == "3":
        # The C++ CSV parser maps SUM_OF_SINUSOIDS VFF parameters from the
        # noise columns: max amplitude, min/max frequency, and min/max sine count.
        max_amplitudes = numeric_values(rows, "noise_max_amplitude") or numeric_values(
            rows, "vff_max_amplitude"
        )
        min_freq = numeric_values(rows, "noise_min_freq")
        max_freq = numeric_values(rows, "noise_max_freq") or numeric_values(rows, "vff_max_freq")
        min_sines = numeric_values(rows, "noise_min_sines")
        max_sines = numeric_values(rows, "noise_max_sines")

        if max_amplitudes:
            details["amplitude_lower"] = 0.0
            details["amplitude_upper"] = max(max_amplitudes)
        details["frequency_lower"] = min_or_blank(min_freq)
        details["frequency_upper"] = max_or_blank(max_freq)
        details["sine_count_lower"] = min_or_blank(min_sines)
        details["sine_count_upper"] = max_or_blank(max_sines)

        parts = [
            f"peak amplitude 0 to {max(max_amplitudes):g}" if max_amplitudes else "amplitude n/a",
            f"frequency {interval_or_blank(min_freq, max_freq)} Hz",
            f"sine count {interval_or_blank(min_sines, max_sines)}",
        ]
        return "; ".join(parts), details

    return "unsupported VFF type", details


def build_comp_vff_report(
    rows: list[ExperimentRow],
    *,
    unique_trajectories_per_group: int,
    group_column: str | None,
    include_existing_in_denominator: bool,
) -> tuple[str, dict[str, list[dict[str, object]]]]:
    rows_by_type: dict[str, list[ExperimentRow]] = {code: [] for code in ["0", "1", "2", "3"]}
    groups_by_type: dict[str, set[str]] = {code: set() for code in ["0", "1", "2", "3"]}
    existing_groups: set[str] = set()

    for row in rows:
        code = vff_type_code(row)
        label = group_label(row, group_column)
        if code == "4":
            existing_groups.add(label)
            continue
        if code in rows_by_type:
            rows_by_type[code].append(row)
            groups_by_type[code].add(label)

    included_groups = set().union(*groups_by_type.values()) if groups_by_type else set()
    denominator_group_count = len(included_groups)
    denominator_note = "excluding vff_type=4 existing-sequence groups"
    if include_existing_in_denominator:
        denominator_group_count += len(existing_groups)
        denominator_note = "including ignored vff_type=4 groups in the denominator"

    total_trajectories = denominator_group_count * unique_trajectories_per_group

    table_rows = []
    csv_rows: list[dict[str, object]] = []
    for code in ["0", "1", "2", "3"]:
        type_rows = rows_by_type[code]
        groups = sorted(groups_by_type[code])
        group_count = len(groups)
        trajectories = group_count * unique_trajectories_per_group
        bounds_text, bounds = comp_vff_bounds(code, type_rows)
        percent = pct(trajectories, total_trajectories)
        group_list = ", ".join(groups) if groups else "-"

        table_rows.append(
            [
                f"{code}: {VFF_TYPES[code]}",
                group_count,
                f"{trajectories:,}",
                percent,
                bounds_text,
                group_list,
            ]
        )

        csv_rows.append(
            {
                "vff_type": code,
                "vff_label": VFF_TYPES[code],
                "group_count": group_count,
                "trajectories": trajectories,
                "percent_dataset": percent,
                "parameter_bounds": bounds_text,
                **bounds,
                "groups": group_list,
            }
        )

    ignored_text = (
        f"- Ignored existing VFF groups (vff_type=4): {len(existing_groups):,}"
        if existing_groups
        else "- Ignored existing VFF groups (vff_type=4): 0"
    )
    report_lines = [
        "# Compensation VFF Distribution",
        "",
        f"- CSV files: {len({row.source_csv for row in rows}):,}",
        f"- Experiment rows scanned: {len(rows):,}",
        f"- Unique trajectories per group: {unique_trajectories_per_group:,}",
        f"- Percent denominator: {denominator_group_count:,} groups, {denominator_note}",
        ignored_text,
        "",
        markdown_table(
            [
                "VFF type",
                "Groups",
                "Trajectories",
                "% dataset",
                "Relevant parameter bounds",
                "Groups with this VFF",
            ],
            table_rows,
        ),
    ]

    return "\n".join(report_lines) + "\n", {"comp_vff_distribution": csv_rows}


def collect_vff_params(
    row: ExperimentRow,
    vff_family: str,
    params: dict[str, dict[str, list[float]]],
) -> None:
    for column, key in [
        ("vff_max_amplitude", "max_amplitude"),
        ("vff_mean_dwell", "mean_dwell"),
        ("vff_min_dwell", "min_dwell"),
        ("vff_max_freq", "max_freq"),
    ]:
        raw = row.text(column)
        if not raw:
            continue
        try:
            params[vff_family][key].append(float(raw))
        except ValueError:
            pass

    if vff_family == "Sum of sinusoids":
        for source, key in [
            ("noise_min_freq", "min_freq"),
            ("noise_max_freq", "max_freq"),
            ("noise_min_sines", "min_sines"),
            ("noise_max_sines", "max_sines"),
        ]:
            raw = row.text(source)
            if not raw:
                continue
            try:
                params[vff_family][key].append(float(raw))
            except ValueError:
                pass


def build_parameter_rows(params: dict[str, dict[str, list[float]]]) -> list[list[object]]:
    rows = []
    for family in sorted(params):
        family_params = params[family]
        rows.append(
            [
                family,
                numeric_summary(family_params.get("max_amplitude", [])),
                numeric_summary(family_params.get("mean_dwell", [])),
                numeric_summary(family_params.get("min_dwell", [])),
                numeric_summary(family_params.get("max_freq", [])),
            ]
        )
    return rows


def sequence_report(stats: list[SequenceStats], failures: list[str]) -> list[str]:
    lines = ["", "## VFF Sequence Inspection", ""]
    if not stats and not failures:
        lines.append("No external VFF sequence paths were found in the CSV rows.")
        return lines

    if stats:
        rows = [
            [
                stat.label,
                stat.path.name,
                f"{stat.rows:,}",
                f"{stat.max_abs:.6g}",
                f"{stat.rms:.6g}",
                f"{stat.fd_rms:.6g}",
                f"{100.0 * stat.nonzero_fraction:.1f}%",
            ]
            for stat in stats
        ]
        lines.append(
            markdown_table(
                ["Label", "File/dir", "Rows", "Max abs", "RMS", "FD RMS", "Nonzero"],
                rows,
            )
        )

    if failures:
        lines.extend(["", "Sequence paths not inspected:"])
        lines.extend(f"- {failure}" for failure in failures[:20])
        if len(failures) > 20:
            lines.append(f"- ... {len(failures) - 20} more")

    return lines


def write_tables(out_dir: Path, report: str, tables: dict[str, list[dict[str, object]]]) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "distribution_summary.md").write_text(report, encoding="utf-8")

    for name, rows in tables.items():
        if not rows:
            continue
        path = out_dir / f"{name}.csv"
        with path.open("w", newline="", encoding="utf-8") as fh:
            writer = csv.DictWriter(fh, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Summarize high-level trajectory/noise/VFF distributions from experiment CSVs."
    )
    parser.add_argument("csv", nargs="+", type=Path, help="Experiment CSV file(s)")
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=None,
        help="Optional directory for distribution_summary.md and CSV tables",
    )
    parser.add_argument(
        "--inspect-sequences",
        action="store_true",
        help="Inspect external VFF .npy files/directories referenced by vff_sequence_file",
    )
    parser.add_argument(
        "--skip-gcode",
        action="store_true",
        help="Do not parse available G-code files referenced by gcode_file_path",
    )
    parser.add_argument(
        "--comp-vff-table",
        action="store_true",
        help="Emit the focused 4-row compensation VFF table and ignore vff_type=4",
    )
    parser.add_argument(
        "--unique-trajectories-per-group",
        type=int,
        default=1560,
        help="Trajectory count represented by each compensation group (default: 1560)",
    )
    parser.add_argument(
        "--group-column",
        default=None,
        help="Optional column to use as the group label, e.g. family_id or experiment_id",
    )
    parser.add_argument(
        "--include-existing-in-denominator",
        action="store_true",
        help="For --comp-vff-table, include vff_type=4 groups in the percentage denominator",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    rows: list[ExperimentRow] = []
    for csv_path in args.csv:
        rows.extend(read_experiment_csv(csv_path))

    if not rows:
        raise SystemExit("No experiment rows found.")

    if args.inspect_sequences and not args.comp_vff_table:
        try:
            import numpy  # noqa: F401
        except ImportError as exc:
            raise SystemExit(
                "--inspect-sequences requires NumPy. Run without that flag for config-only tables."
            ) from exc

    if args.comp_vff_table:
        report, tables = build_comp_vff_report(
            rows,
            unique_trajectories_per_group=args.unique_trajectories_per_group,
            group_column=args.group_column,
            include_existing_in_denominator=args.include_existing_in_denominator,
        )
    else:
        report, tables = build_report(
            rows,
            inspect_sequences=args.inspect_sequences,
            parse_gcode=not args.skip_gcode,
        )

    print(report)
    if args.out_dir:
        write_tables(args.out_dir, report, tables)
        print(f"Wrote report files to: {args.out_dir}")


if __name__ == "__main__":
    main()
