#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Minimal headless tinySA sweep CLI.

This script intentionally does not import QtTinySA.py because that module starts
the Qt GUI at import time. It implements the small subset of the serial protocol
needed for a one-shot sweep and CSV export.
"""

from __future__ import annotations

import argparse
import csv
import logging
import struct
import sys
import time
from pathlib import Path
from typing import Iterable, Sequence

try:
    import serial
    from serial.tools import list_ports
except ImportError as exc:  # pragma: no cover - exercised only without pyserial
    raise SystemExit("pyserial is required. Install the project requirements first.") from exc


BAUDRATE = 576000
TINYSA_VID = 0x0483
TINYSA_PID = 0x5740
PROMPT = b"ch> "
DEFAULT_OUTPUT_DIR = Path("sweep_files")


class SweepError(RuntimeError):
    """Raised when the sweep cannot complete cleanly."""


def parse_frequency(value: str) -> int:
    """Parse CLI frequency values such as '2400e6' into integer Hz."""
    try:
        frequency = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"invalid frequency value: {value!r}") from exc

    if frequency <= 0:
        raise argparse.ArgumentTypeError("frequency values must be positive")
    return int(round(frequency))


def parse_points(value: str) -> int:
    try:
        points = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"invalid point count: {value!r}") from exc

    if points < 2:
        raise argparse.ArgumentTypeError("points must be at least 2")
    return points


def center_span_to_start_stop(center_hz: int, span_hz: int) -> tuple[int, int]:
    if span_hz <= 0:
        raise ValueError("span must be positive")

    start_hz = int(round(center_hz - span_hz / 2))
    stop_hz = int(round(center_hz + span_hz / 2))
    if start_hz < 0:
        raise ValueError("center/span produces a negative start frequency")
    if start_hz >= stop_hz:
        raise ValueError("start frequency must be lower than stop frequency")
    return start_hz, stop_hz


def frequency_axis(start_hz: int, stop_hz: int, points: int) -> list[int]:
    if points < 2:
        raise ValueError("points must be at least 2")

    step = (stop_hz - start_hz) / (points - 1)
    return [int(round(start_hz + index * step)) for index in range(points)]


def scale_for_version(version: str) -> int:
    """Return the GUI-compatible dBm conversion scale for a firmware string."""
    if version.startswith("tinySA4"):
        return 174
    if version.startswith("tinySA"):
        return 128
    raise SweepError(f"device did not identify as tinySA: {version!r}")


def is_ultra(version: str) -> bool:
    return version.startswith("tinySA4")


def decode_sample(block: bytes, scale: int) -> float:
    if len(block) != 3:
        raise SweepError(f"expected 3 data bytes, got {len(block)}")

    _marker, data = struct.unpack("<cH", block)
    return (data / 32) - scale


def write_csv(path: Path, frequencies_hz: Sequence[int], dbm_values: Sequence[float]) -> None:
    if len(frequencies_hz) != len(dbm_values):
        raise ValueError("frequency and reading counts differ")

    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["frequency_hz", "dbm"])
        for frequency_hz, dbm in zip(frequencies_hz, dbm_values):
            writer.writerow([frequency_hz, f"{dbm:.2f}"])


def default_output_path(output_dir: Path = DEFAULT_OUTPUT_DIR) -> Path:
    timestamp = time.strftime("%Y-%m-%d-%H%M%S")
    path = output_dir / f"{timestamp}_sweep.csv"
    counter = 1
    while path.exists():
        path = output_dir / f"{timestamp}_sweep_{counter}.csv"
        counter += 1
    return path


def find_tinysa_ports() -> list[object]:
    return [
        port
        for port in list_ports.comports()
        if port.vid == TINYSA_VID and port.pid == TINYSA_PID
    ]


def choose_port(port_override: str | None) -> str:
    if port_override:
        return port_override

    ports = find_tinysa_ports()
    if not ports:
        raise SweepError("no tinySA device found; pass --port to select a serial device")
    if len(ports) > 1:
        port_list = ", ".join(port.device for port in ports)
        raise SweepError(f"multiple tinySA devices found ({port_list}); pass --port")
    return ports[0].device


def clear_buffer(usb: serial.Serial) -> None:
    while usb.in_waiting:
        usb.read_all()
        time.sleep(0.01)


def serial_query(usb: serial.Serial, command: str) -> str:
    payload = command.encode()
    usb.write(payload)
    usb.read_until(payload + b"\n")
    response = usb.read_until(PROMPT)
    if not response.endswith(PROMPT):
        raise SweepError(f"timed out waiting for prompt after {command.strip()!r}")
    return response[: -len(PROMPT)].decode(errors="replace").strip()


def serial_write(usb: serial.Serial, command: str) -> None:
    payload = command.encode()
    usb.write(payload)
    response = usb.read_until(PROMPT)
    if not response.endswith(PROMPT):
        raise SweepError(f"timed out waiting for prompt after {command.strip()!r}")


def setup_device(usb: serial.Serial, version: str, rbw: str) -> None:
    commands = [
        "abort on\r",
        f"rbw {rbw}\r",
        "attenuate auto\r",
    ]
    if is_ultra(version):
        commands.extend(["lna off\r", "spur auto\r"])
    else:
        commands.append("spur on\r")

    for command in commands:
        logging.info("setup: %s", command.strip())
        serial_write(usb, command)


def sweep_timeout_seconds(start_hz: int, stop_hz: int, points: int, rbw: str) -> float:
    """Use the same broad timeout heuristic as the GUI for one 20-point block."""
    if rbw == "auto":
        rbw_khz = (stop_hz - start_hz) * 7e-6
    else:
        try:
            rbw_khz = float(rbw)
        except ValueError:
            return 10.0

    rbw_khz = min(max(rbw_khz, 0.2), 850)
    timeout = ((stop_hz - start_hz) / 20e3) / (rbw_khz**2) + points / 500
    return max(timeout * 20 / points + 1, 1.0)


def run_sweep(
    usb: serial.Serial,
    start_hz: int,
    stop_hz: int,
    points: int,
    scale: int,
    rbw: str,
) -> list[float]:
    command = f"scanraw {start_hz} {stop_hz} {points} 3\r"
    usb.timeout = sweep_timeout_seconds(start_hz, stop_hz, points, rbw)

    logging.info("sweep: %s", command.strip())
    usb.write(command.encode())
    header = usb.read_until(command.encode() + b"\n{")
    if not header.endswith(command.encode() + b"\n{"):
        raise SweepError("timed out waiting for scanraw data start")

    readings = []
    for _point in range(points):
        block = usb.read(3)
        if block == b"}":
            raise SweepError("sweep stopped by tinySA screen touch or jog button")
        readings.append(decode_sample(block, scale))

    # Newer firmware can auto-repeat scanraw. Stop after the first sweep and
    # drain back to the command prompt so the device is left in command mode.
    usb.write(b"abort\r")
    usb.read_until(PROMPT)
    return readings


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run one headless tinySA sweep and save frequency_hz,dbm CSV data."
    )
    parser.add_argument("--center", required=True, type=parse_frequency, help="center frequency in Hz")
    parser.add_argument("--span", required=True, type=parse_frequency, help="sweep bandwidth/span in Hz")
    parser.add_argument("--points", default=300, type=parse_points, help="number of sweep points (default: 300)")
    parser.add_argument("--out", type=Path, help="CSV output path (default: timestamped CSV in sweep_files)")
    parser.add_argument("--port", help="serial port override, e.g. /dev/ttyACM0 or COM3")
    parser.add_argument("--rbw", default="auto", help="RBW in kHz or 'auto' (default: auto)")
    parser.add_argument("--verbose", action="store_true", help="show setup and sweep progress")
    return parser


def main(argv: Iterable[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    logging.basicConfig(format="%(message)s", level=logging.INFO if args.verbose else logging.WARNING)

    try:
        start_hz, stop_hz = center_span_to_start_stop(args.center, args.span)
        frequencies_hz = frequency_axis(start_hz, stop_hz, args.points)
        output_path = args.out or default_output_path()
        port = choose_port(args.port)

        logging.info("opening %s at %s baud", port, BAUDRATE)
        with serial.Serial(port, baudrate=BAUDRATE, timeout=2) as usb:
            clear_buffer(usb)
            version = serial_query(usb, "version\r")
            scale = scale_for_version(version)
            logging.info("device: %s", version)
            setup_device(usb, version, args.rbw)
            readings = run_sweep(usb, start_hz, stop_hz, args.points, scale, args.rbw)

        write_csv(output_path, frequencies_hz, readings)
        print(f"wrote {len(readings)} points to {output_path}")
        return 0
    except (OSError, serial.SerialException, SweepError, ValueError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
