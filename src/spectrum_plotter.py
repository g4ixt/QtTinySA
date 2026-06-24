#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Plot tinySA spectrum CSV files and report the strongest peaks."""

from __future__ import annotations

import argparse
import os
import sys
import tempfile
from pathlib import Path
from typing import Iterable

os.environ.setdefault("MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "qttinysa-matplotlib"))

import matplotlib.pyplot as plt
import pandas as pd


DEFAULT_INPUT = Path("sweep_files/sweep.csv")
DEFAULT_SAVE_DIR = Path("sweep_files")


def read_spectrum_csv(path: Path) -> list[dict[str, object]]:
    """Read CLI CSVs, QtTinySA trace-pair CSVs, or headerless sweep CSVs."""
    df = pd.read_csv(path)
    traces: list[dict[str, object]] = []

    if {"frequency_hz", "dbm"}.issubset(df.columns):
        trace = df[["frequency_hz", "dbm"]].apply(pd.to_numeric, errors="coerce").dropna()
        return [{
            "label": path.stem,
            "freq_hz": trace["frequency_hz"],
            "dbm": trace["dbm"],
        }]

    x_columns = [column for column in df.columns if str(column).endswith("_x")]
    for x_column in x_columns:
        stem = str(x_column)[:-2]
        y_column = f"{stem}_y"
        if y_column not in df.columns:
            continue

        trace = df[[x_column, y_column]].apply(pd.to_numeric, errors="coerce").dropna()
        traces.append({
            "label": f"{path.stem} trace {stem}",
            "freq_hz": trace[x_column],
            "dbm": trace[y_column],
        })

    if traces:
        return traces

    # Fallback for QtTinySA save-sweep CSVs: no header, frequency in column 0,
    # one or more reading columns after it.
    df = pd.read_csv(path, header=None).apply(pd.to_numeric, errors="coerce")
    if df.shape[1] < 2:
        raise ValueError(f"{path} does not look like spectrum data")

    for column in df.columns[1:]:
        trace = df[[0, column]].dropna()
        traces.append({
            "label": f"{path.stem} sweep {column}",
            "freq_hz": trace[0],
            "dbm": trace[column],
        })
    return traces


def load_traces(paths: Iterable[Path]) -> list[dict[str, object]]:
    traces: list[dict[str, object]] = []
    for path in paths:
        traces.extend(read_spectrum_csv(path))
    if not traces:
        raise ValueError("no traces found")
    return traces


def local_peak_indexes(values: pd.Series) -> pd.Index:
    previous_values = values.shift(1)
    next_values = values.shift(-1)
    is_peak = values.ge(previous_values.fillna(float("-inf"))) & values.ge(next_values.fillna(float("-inf")))
    return values[is_peak].index


def strongest_peaks(traces: list[dict[str, object]], count: int = 5) -> pd.DataFrame:
    rows = []
    for trace in traces:
        freq_hz = trace["freq_hz"]
        dbm = trace["dbm"]
        assert isinstance(freq_hz, pd.Series)
        assert isinstance(dbm, pd.Series)
        median_dbm = dbm.median()

        peak_indexes = local_peak_indexes(dbm)
        peak_data = pd.DataFrame({
            "frequency_hz": freq_hz.loc[peak_indexes],
            "dbm": dbm.loc[peak_indexes],
        })
        if len(peak_data) < count:
            peak_data = pd.DataFrame({"frequency_hz": freq_hz, "dbm": dbm})

        peak_data = peak_data.sort_values("dbm", ascending=False).head(count)
        for rank, (_index, row) in enumerate(peak_data.iterrows(), start=1):
            rows.append({
                "trace": trace["label"],
                "maximum_rank": rank,
                "frequency_hz": int(round(row["frequency_hz"])),
                "frequency_mhz": row["frequency_hz"] / 1e6,
                "dbm": row["dbm"],
                "above_median_db": row["dbm"] - median_dbm,
            })
    return pd.DataFrame(rows)


def trace_summary(traces: list[dict[str, object]]) -> pd.DataFrame:
    rows = []
    for trace in traces:
        freq_hz = trace["freq_hz"]
        dbm = trace["dbm"]
        assert isinstance(freq_hz, pd.Series)
        assert isinstance(dbm, pd.Series)
        rows.append({
            "trace": trace["label"],
            "points": len(dbm),
            "start_MHz": freq_hz.min() / 1e6,
            "stop_MHz": freq_hz.max() / 1e6,
            "min_dBm": dbm.min(),
            "median_dBm": dbm.median(),
            "mean_dBm": dbm.mean(),
            "max_dBm": dbm.max(),
            "dynamic_range_dB": dbm.max() - dbm.min(),
        })
    return pd.DataFrame(rows)


def console_peak_table(peaks: pd.DataFrame) -> pd.DataFrame:
    return peaks[["trace", "maximum_rank", "frequency_mhz", "dbm", "above_median_db"]].rename(columns={
        "maximum_rank": "top_maximum",
        "frequency_mhz": "frequency_MHz",
        "dbm": "dBm",
        "above_median_db": "above_median_dB",
    })


def print_trace_summary(summary: pd.DataFrame) -> None:
    print("Trace summary")
    print(summary.to_string(index=False, formatters={
        "start_MHz": "{:.6f}".format,
        "stop_MHz": "{:.6f}".format,
        "min_dBm": "{:.2f}".format,
        "median_dBm": "{:.2f}".format,
        "mean_dBm": "{:.2f}".format,
        "max_dBm": "{:.2f}".format,
        "dynamic_range_dB": "{:.2f}".format,
    }))


def plot_traces(traces: list[dict[str, object]], title: str) -> tuple[plt.Figure, plt.Axes]:
    fig, ax = plt.subplots(figsize=(12, 6))
    for trace in traces:
        freq_hz = trace["freq_hz"]
        dbm = trace["dbm"]
        assert isinstance(freq_hz, pd.Series)
        assert isinstance(dbm, pd.Series)
        ax.plot(freq_hz / 1e6, dbm, linewidth=1.4, label=str(trace["label"]))

    ax.set_title(title)
    ax.set_xlabel("Frequency (MHz)")
    ax.set_ylabel("Amplitude (dBm)")
    ax.grid(True, alpha=0.35)
    ax.legend()
    fig.tight_layout()
    return fig, ax


def save_outputs(fig: plt.Figure, peaks: pd.DataFrame, save_dir: Path, stem: str) -> tuple[Path, Path]:
    save_dir.mkdir(parents=True, exist_ok=True)
    graph_path = save_dir / f"{stem}_spectrum.png"
    peaks_path = save_dir / f"{stem}_peaks.csv"
    fig.savefig(graph_path, dpi=150)
    peaks.to_csv(peaks_path, index=False)
    return graph_path, peaks_path


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Plot tinySA spectrum CSV data and print the 5 strongest peaks per trace."
    )
    parser.add_argument(
        "csv_files",
        nargs="*",
        type=Path,
        default=[DEFAULT_INPUT],
        help=f"CSV file(s) to plot (default: {DEFAULT_INPUT})",
    )
    parser.add_argument("--peaks", type=int, default=5, help="number of peaks per trace (default: 5)")
    parser.add_argument("--save", action="store_true", help="save graph and peak CSV to sweep_files")
    parser.add_argument("--save-dir", type=Path, default=DEFAULT_SAVE_DIR, help="save directory (default: sweep_files)")
    parser.add_argument("--no-show", action="store_true", help="do not open an interactive plot window")
    return parser


def main(argv: Iterable[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    try:
        traces = load_traces(args.csv_files)
        summary = trace_summary(traces)
        peaks = strongest_peaks(traces, args.peaks)
        title = ", ".join(path.name for path in args.csv_files)
        fig, _ax = plot_traces(traces, title)

        print_trace_summary(summary)
        print()
        print(f"Top {args.peaks} maxima per trace")
        print(console_peak_table(peaks).to_string(index=False, formatters={
            "frequency_MHz": "{:.6f}".format,
            "dBm": "{:.2f}".format,
            "above_median_dB": "{:.2f}".format,
        }))

        if args.save:
            stem = args.csv_files[0].stem if len(args.csv_files) == 1 else "spectrum"
            graph_path, peaks_path = save_outputs(fig, peaks, args.save_dir, stem)
            print(f"\nsaved graph: {graph_path}")
            print(f"saved peaks: {peaks_path}")

        if not args.no_show:
            plt.show()
        else:
            plt.close(fig)
        return 0
    except (OSError, ValueError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
