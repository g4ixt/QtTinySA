# SPDX-License-Identifier: GPL-3.0-or-later
#
# Copyright (C) 2026 <Your Name / QtTinySA project>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.
#
# This module was developed collaboratively with Claude, Anthropic's AI assistant
# (https://www.anthropic.com), as a coding aid during development.

"""
soapy_receiver_mp.py

Multiprocessing counterpart to soapy_receiver.py: runs the SoapySDR acquisition loop in a dedicated OS process
instead of a QThreadPool worker, and delivers spectra via a multiprocessing.Queue instead of a Qt signal. This
trades the threading version's shared-memory simplicity for full process isolation -- worthwhile if the FFT/
NumPy work in the acquisition loop is heavy enough to compete with QtTinySA's GUI thread for the GIL, or if you
want a SoapySDR/driver crash to be unable to take down the whole GUI process.

Architecture, and why it looks different from soapy_receiver.py:
    A SoapySDR.Device handle cannot cross a process boundary -- it wraps a live driver/hardware connection, not
    picklable data. So the device can only ever exist inside the child process; nothing in the parent process
    ever touches it directly. Consequently:
      - There is no Qt dependency here at all. SoapyReceiverProcess (the parent-side class you construct) is a
        plain Python object; it doesn't need to live on a particular thread or run inside a Qt event loop.
      - Every setting is either applied locally to SoapyReceiverProcess's own attributes (used the next time
        start() creates a fresh child process) or, if the receiver is already running, sent as a command into a
        multiprocessing.Queue for the worker to apply against its own local device handle -- mirroring exactly
        which settings soapy_receiver.py allows live vs. requires stop()/wait()/start() for, just via IPC
        instead of shared attributes.
      - Anything that needs the live device (TX antenna selection, the TSG) can no longer be reached into
        directly the way LimeLoopbackCalibrator does in the threading version -- see set_tx_antenna(),
        enable_tsg(), disable_tsg() below, and lime_loopback_mp.py.
      - Spectra go out on two separate queues, not one: data_queue for normal operation (the one QtTinySA's own
        dedicated draining thread should read), and calibration_queue, used only while a TSG tone is active.
        This is deliberate: if the loopback calibrator also had to read from data_queue, it would be racing
        QtTinySA's own consumer thread for the exact samples it needs, with no way to guarantee it wins. A
        separate channel avoids that.

Requires:
    pip install numpy
    SoapySDR Python bindings (build/install from
    https://github.com/pothosware/SoapySDR/wiki/PythonSupport)

Portability note: multiprocessing's default start method matters here. Windows and macOS have always defaulted
to "spawn"; Linux defaulted to "fork" through Python 3.13, but as of Python 3.14 it defaults to "forkserver"
(see https://docs.python.org/3/library/multiprocessing.html#contexts-and-start-methods) -- a deliberate change
away from fork, since forking an already multi-threaded process (which a running Qt application with QThreadPool
workers typically is) is fragile. Both "spawn" and "forkserver" reconstruct the child process by re-importing
your top-level script rather than cloning memory, so the code that constructs a SoapyReceiverProcess and calls
start() must sit behind `if __name__ == "__main__":` in your top-level script -- otherwise that script's own
top-level code (e.g. launching the Qt application) runs again inside the child, which for a GUI app means a
second window opening. This is a standard multiprocessing constraint, not specific to this module, but it's easy
to miss if you developed against an older Python where Linux's default ("fork") didn't require it.
"""

import logging
import math
import multiprocessing
import queue
import time
from dataclasses import dataclass, field
from typing import Any

import numpy as np

try:
    import SoapySDR
    from SoapySDR import SOAPY_SDR_RX, SOAPY_SDR_TX, SOAPY_SDR_CF32
except ImportError as exc:  # pragma: no cover
    raise ImportError(
        "SoapySDR Python bindings not found. Install SoapySDR with Python "
        "support: https://github.com/pothosware/SoapySDR/wiki/PythonSupport"
    ) from exc

logger = logging.getLogger(__name__)


# ----------------------------------------------------------------------------------------------------------------
# Preset sample rates, FFT/window sizing, and other constants -- identical to soapy_receiver.py, duplicated here
# rather than imported since this is meant to be a standalone module.
# ----------------------------------------------------------------------------------------------------------------
BASE_CLOCK_HZ = 30_720_000.0  # 30.72 MHz
DEFAULT_SAMPLE_RATE_DIVISORS: tuple[int, ...] = (1, 2, 4, 8, 16, 32, 64, 128)
DEFAULT_PRESET_SAMPLE_RATES: tuple[float, ...] = tuple(
    BASE_CLOCK_HZ / d for d in DEFAULT_SAMPLE_RATE_DIVISORS
)

MAX_FFT_SIZE = 65536
MIN_FFT_SIZE = 256

DEFAULT_KAISER_BETA = 10.5

LIME_VALID_OVERSAMPLING_RATIOS: tuple[int, ...] = (0, 1, 2, 4, 8, 16, 32)
LIME_CALIBRATE_BANDWIDTH_RANGE_HZ: tuple[float, float] = (2.5e6, 1.2e8)

# DC offset removal modes: "hardware" uses the device's own correction (where supported); "software" subtracts
# the measured mean IQ offset from every captured block; "both" does both (hardware handles gross/analog offset
# before the ADC, software mops up whatever residual or drift remains); "none" disables removal entirely.
DC_OFFSET_MODES: tuple[str, ...] = ("software", "hardware", "both", "none")

# How often (seconds) the worker checks command_queue and stop_event when there is otherwise no data to
# capture (e.g. waiting on a device that failed to open). While streaming, commands are checked once per
# acquisition block, which is typically much faster than this.
IDLE_POLL_INTERVAL_S = 0.05


@dataclass
class SoapyDeviceInfo:
    """Convenience wrapper around a SoapySDR enumeration result."""
    driver: str
    label: str
    args: dict[str, str] = field(default_factory=dict)


class ReceiverBusyError(RuntimeError):
    """Raised when a setting is changed, or start() is called, while the worker process is active."""


# ----------------------------------------------------------------------------------------------------------------
# Pure helper functions, usable from both the parent process (for up-front validation, e.g. set_rbw() raising
# before a child is ever spawned) and the worker process (for actually applying settings).
# ----------------------------------------------------------------------------------------------------------------
def is_lime_driver(driver: str | None) -> bool:
    """True for a LimeSDR variant (classic LimeSuite's 'lime' or LimeSuiteNG's 'limesuiteng')."""
    return bool(driver) and "lime" in driver.lower()


def kaiser_enbw_factor(beta: float, reference_size: int = 8192) -> float:
    """
    Equivalent noise bandwidth (ENBW), in bins, of a numpy Kaiser window with the given beta -- how much wider
    a window's main lobe is than an ideal rectangular one of the same length, as a multiplying factor. Used by
    calculate_fft_size() to compensate for that broadening, since the achievable RBW is wider than a raw
    sample_rate/fft_size bin spacing would suggest once a window is applied.
    """
    w = np.kaiser(reference_size, beta)
    return float(reference_size * np.sum(w ** 2) / (np.sum(w) ** 2))


def calculate_fft_size(
    rbw_hz: float,
    sample_rate: float,
    beta: float = DEFAULT_KAISER_BETA,
    max_fft_size: int = MAX_FFT_SIZE,
    min_fft_size: int = MIN_FFT_SIZE,
) -> int:
    """
    Compute the power-of-two FFT size needed to achieve rbw_hz at sample_rate, accounting for the Kaiser
    window's ENBW (see kaiser_enbw_factor()). Raises ValueError, naming the finest achievable RBW instead, if
    the required FFT size would exceed max_fft_size.
    """
    if rbw_hz <= 0:
        raise ValueError("rbw_hz must be > 0")
    if sample_rate <= 0:
        raise ValueError("sample_rate must be > 0")

    enbw_factor = kaiser_enbw_factor(beta)
    ideal = (sample_rate / rbw_hz) * enbw_factor
    fft_size = 1 << max(1, int(ideal - 1)).bit_length()

    if fft_size > max_fft_size:
        achievable_rbw = (sample_rate / max_fft_size) * enbw_factor
        raise ValueError(
            f"Requested RBW {rbw_hz:.1f} Hz is not achievable at a {sample_rate:.0f} Hz sample rate (span) "
            f"with Kaiser beta {beta:.2f} and a max FFT size of {max_fft_size}. The finest achievable RBW at "
            f"this span is approximately {achievable_rbw:.1f} Hz -- increase the RBW, reduce the span, or "
            f"lower the Kaiser beta."
        )

    return max(min_fft_size, fft_size)


def make_window(size: int, beta: float) -> np.ndarray:
    """
    Build a coherent-gain-normalized numpy Kaiser window: the raw window divided by its own mean, so a
    full-scale sinusoid reports the same peak power regardless of beta -- changing beta trades off main-lobe
    width against sidelobe suppression without shifting the reported signal level.
    """
    win = np.kaiser(size, beta)
    coherent_gain = win.mean()
    if coherent_gain <= 0:
        coherent_gain = 1.0
    return (win / coherent_gain).astype(np.float32)


def choose_sample_rate_for_span(span_hz: float, preset_sample_rates) -> float:
    """Smallest preset sample rate that is >= span_hz. Raises ValueError if span_hz exceeds every preset.
    Shared by set_span() (single-capture mode) and set_sweep() (per-segment width), so both pick rates the
    same way."""
    if span_hz <= 0:
        raise ValueError("span_hz must be > 0")
    presets = sorted(preset_sample_rates)
    chosen = next((rate for rate in presets if rate >= span_hz), None)
    if chosen is None:
        raise ValueError(
            f"Requested span {span_hz:.0f} Hz exceeds the largest available sample rate "
            f"({presets[-1]:.0f} Hz); this receiver cannot capture a wider span."
        )
    return chosen


def compute_segment_crop_bins(fft_size: int, sample_rate: float, segment_span_nominal_hz: float) -> int:
    """
    Number of bins to trim from each side of an internal segment boundary when stitching a composite sweep, so
    the result has a precisely predictable total length regardless of rounding. 0 if segments tile exactly with
    no overlap (sample_rate == segment_span_nominal_hz, the normal case when a sweep divides evenly into a
    preset rate).
    """
    excess_hz = sample_rate - segment_span_nominal_hz
    if excess_hz <= 0:
        return 0
    bin_spacing = sample_rate / fft_size
    return max(0, round((excess_hz / 2.0) / bin_spacing))


def compute_composite_size(fft_size: int, num_segments: int, crop_bins: int) -> int:
    """
    Total length of a composite spectrum built from num_segments segments of fft_size bins each, with crop_bins
    trimmed from each side of every internal (not outer) segment boundary. Reduces to fft_size when
    num_segments is 1, matching ordinary (non-sweep) single-capture mode.
    """
    if num_segments <= 1:
        return fft_size
    return num_segments * fft_size - (num_segments - 1) * 2 * crop_bins


def compute_spectrum(iq: np.ndarray, window: np.ndarray) -> np.ndarray:
    """Apply the (coherent-gain-normalized) window, FFT, and convert to dB: 20*log10(|FFT(iq*window)|/N)."""
    windowed = iq * window
    spectrum = np.fft.fftshift(np.fft.fft(windowed))
    n = len(spectrum)
    power = 20 * np.log10(np.abs(spectrum) / n + 1e-20)
    return power.astype(np.float32)


def list_devices() -> list[SoapyDeviceInfo]:
    """Enumerate all SoapySDR devices currently visible on the system. Safe to call without a worker process."""
    results: list[SoapyDeviceInfo] = []
    try:
        for args in SoapySDR.Device.enumerate():
            d = dict(args)
            driver = d.get("driver", "unknown")
            label = d.get("label", driver)
            results.append(SoapyDeviceInfo(driver=driver, label=label, args=d))
    except Exception:
        logger.exception("SoapySDR device enumeration failed")
    return results


def _import_tsg_class():
    """
    Import LimeTestSignalGenerator whether this module is used as part of a package (e.g. QtTinySA's modules/
    directory, imported as modules.soapy_receiver_mp) or as a top-level module. Done lazily, on first TSG use, so
    a worker that never uses the TSG doesn't need lime_tsg_mp importable at all.
    """
    try:
        from .lime_tsg_mp import LimeTestSignalGenerator
    except ImportError:
        from lime_tsg_mp import LimeTestSignalGenerator
    return LimeTestSignalGenerator


# ----------------------------------------------------------------------------------------------------------------
# Worker-side: everything below this point runs inside the child process only.
# ----------------------------------------------------------------------------------------------------------------
class _ReceiverWorker:
    """
    Owns the live SoapySDR device and runs the acquisition loop, entirely inside the child process. Constructed
    from a plain settings dict (see SoapyReceiverProcess._snapshot_settings()) rather than shared attributes,
    since nothing here can be reached from the parent process directly.
    """

    def __init__(
        self,
        settings: dict[str, Any],
        command_queue: multiprocessing.Queue,
        data_queue: multiprocessing.Queue,
        calibration_queue: multiprocessing.Queue,
        event_queue: multiprocessing.Queue,
        stop_event: Any,
        finished_event: Any,
    ):
        self._s = dict(settings)
        self._command_queue = command_queue
        self._data_queue = data_queue
        self._calibration_queue = calibration_queue
        self._event_queue = event_queue
        self._stop_event = stop_event
        self._finished_event = finished_event

        self._device: "SoapySDR.Device | None" = None
        self._rx_stream = None
        self._tsg_active = False  # while True, spectra go to calibration_queue instead of data_queue

    # -- device lifecycle -------------------------------------------------------------------------------------
    def _is_lime(self) -> bool:
        return is_lime_driver(self._s["driver"])

    @staticmethod
    def _describe_visible_devices() -> str:
        """What SoapySDR.Device.enumerate() can see from inside this worker process, for error messages. An empty
        result while the parent process can enumerate the device usually means something else is holding it open."""
        try:
            found = [dict(d) for d in SoapySDR.Device.enumerate()]
        except Exception as exc:
            return f"(enumerate() itself failed: {exc})"
        if not found:
            return "none"
        return "; ".join(
            f"driver={d.get('driver', '?')} serial={d.get('serial', '?')} label={d.get('label', '?')}"
            for d in found
        )

    def _open_device(self) -> None:
        try:
            self._device = SoapySDR.Device(self._s["device_args"])
        except Exception as exc:
            raise RuntimeError(
                f"Failed to open SoapySDR device with args {self._s['device_args']}: {exc}. "
                f"Devices visible to this worker process: {self._describe_visible_devices()}"
            ) from exc

        self._apply_oversampling()
        self._apply_antenna()
        self._apply_rate_and_freq()
        self._apply_gfir_lpf()
        self._apply_rx_gain()
        self._apply_bandwidth()

        if self._s["auto_calibrate"] and self._is_lime():
            self._calibrate()

        self._apply_dc_offset()
        self._apply_iq_balance()

        try:
            self._rx_stream = self._device.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32)
            self._device.activateStream(self._rx_stream)
        except Exception as exc:
            raise RuntimeError(f"Failed to set up RX stream: {exc}") from exc

        self._event_queue.put((
            "status",
            f"Opened {self._s['driver']} device, streaming at {self._s['sample_rate']:.0f} Sps, "
            f"FFT size {self._s['fft_size']}, Kaiser beta {self._s['kaiser_beta']:.2f}",
        ))

    def _close_device(self) -> None:
        try:
            if self._device is not None and self._rx_stream is not None:
                self._device.deactivateStream(self._rx_stream)
                self._device.closeStream(self._rx_stream)
        except Exception:
            logger.exception("Error closing SoapySDR stream")
        finally:
            self._rx_stream = None
            self._device = None

    def _apply_antenna(self) -> None:
        antenna = self._s.get("antenna")
        if not antenna or self._device is None:
            return
        try:
            available = self._device.listAntennas(SOAPY_SDR_RX, 0)
            if antenna in available:
                self._device.setAntenna(SOAPY_SDR_RX, 0, antenna)
            else:
                logger.warning("Antenna '%s' not available (options: %s)", antenna, available)
        except Exception:
            logger.exception("Failed to set antenna")

    def _apply_rate_and_freq(self) -> None:
        if self._device is None:
            return
        try:
            self._device.setSampleRate(SOAPY_SDR_RX, 0, self._s["sample_rate"])
        except Exception:
            logger.exception("Failed to set sample rate")
        self._apply_frequency()

    def _supports_offset_tuning_arg(self) -> bool:
        if self._device is None:
            return False
        try:
            return any(arg.key == "OFFSET" for arg in self._device.getFrequencyArgsInfo(SOAPY_SDR_RX, 0))
        except Exception:
            return False

    def _supports_offset_tuning_components(self) -> bool:
        if self._device is None:
            return False
        try:
            return "BB" in self._device.listFrequencies(SOAPY_SDR_RX, 0)
        except Exception:
            return False

    def _apply_frequency(self, center_freq: float | None = None) -> None:
        """
        Apply a center frequency. With no argument, uses self._s["center_freq"] (the single-capture/live-retune
        path). Called with an explicit value per segment during a sweep (see _run_sweep_loop()), in which case
        RF/NCO offset tuning is skipped entirely -- unsupported in combination with sweeping for now, since each
        segment has its own LO position and the interaction needs more thought than a quick bolt-on deserves.
        """
        if self._device is None:
            return
        if center_freq is None:
            center_freq = self._s["center_freq"]
        offset_hz = 0.0 if self._s.get("sweep_active") else self._s["tuning_offset_hz"]
        try:
            if offset_hz and self._supports_offset_tuning_arg():
                self._device.setFrequency(SOAPY_SDR_RX, 0, center_freq, {"OFFSET": str(offset_hz)})
            elif offset_hz and self._supports_offset_tuning_components():
                self._device.setFrequency(SOAPY_SDR_RX, 0, "RF", center_freq + offset_hz)
                self._device.setFrequency(SOAPY_SDR_RX, 0, "BB", -offset_hz)
            else:
                if offset_hz:
                    logger.info("Offset tuning requested but unsupported by this device; ignoring")
                self._device.setFrequency(SOAPY_SDR_RX, 0, center_freq)
        except Exception:
            logger.exception("Failed to set frequency")

    def _apply_rx_gain(self) -> None:
        if self._device is None:
            return
        try:
            if hasattr(self._device, "setGainMode"):
                self._device.setGainMode(SOAPY_SDR_RX, 0, bool(self._s["rx_agc"]))
            if not self._s["rx_agc"] and self._s["rx_gain"] is not None:
                self._device.setGain(SOAPY_SDR_RX, 0, self._s["rx_gain"])
        except Exception:
            logger.exception("Failed to set RX gain")

    def _set_tx_gain(self, gain_db: float) -> None:
        if self._device is None:
            return
        try:
            self._device.setGain(SOAPY_SDR_TX, self._s.get("tx_channel", 0), gain_db)
        except Exception:
            logger.exception("Failed to set TX gain")

    def _apply_bandwidth(self) -> None:
        if self._device is None or self._s["bandwidth"] is None:
            return
        try:
            self._device.setBandwidth(SOAPY_SDR_RX, 0, self._s["bandwidth"])
        except Exception:
            logger.exception("Failed to set bandwidth")

    def _apply_dc_offset(self) -> None:
        if self._device is None:
            return
        mode = self._s["dc_offset_mode"]
        want_hardware = mode in ("hardware", "both")
        try:
            if self._device.hasDCOffsetMode(SOAPY_SDR_RX, 0):
                self._device.setDCOffsetMode(SOAPY_SDR_RX, 0, want_hardware)
            elif want_hardware:
                logger.info(
                    "No hardware DC offset correction on this device; mode '%s' still applies software "
                    "removal, but the hardware component of it is a no-op",
                    mode,
                )
        except Exception:
            logger.exception("Failed to set hardware DC offset mode")

    def _apply_iq_balance(self) -> None:
        if self._device is None:
            return
        try:
            if not hasattr(self._device, "hasIQBalanceMode") or not self._device.hasIQBalanceMode(
                SOAPY_SDR_RX, 0
            ):
                return
            self._device.setIQBalanceMode(SOAPY_SDR_RX, 0, bool(self._s["iq_balance_removal"]))
        except Exception:
            logger.exception("Failed to set IQ balance mode")

    def _apply_oversampling(self) -> None:
        if self._device is None or not self._is_lime():
            return
        try:
            self._device.writeSetting("OVERSAMPLING", str(self._s["oversampling"]))
        except Exception:
            logger.exception("Failed to set LimeSDR OVERSAMPLING")

    def _apply_gfir_lpf(self) -> None:
        bw = self._s.get("gfir_lpf_bandwidth")
        if self._device is None or not self._is_lime() or bw is None:
            return
        try:
            self._device.writeSetting(SOAPY_SDR_RX, 0, "ENABLE_GFIR_LPF", str(bw))
        except Exception:
            logger.exception("Failed to set LimeSDR GFIR LPF bandwidth")

    def _calibrate(self) -> None:
        if self._device is None or not self._is_lime():
            return
        requested = self._s["bandwidth"] or self._s["sample_rate"]
        low, high = LIME_CALIBRATE_BANDWIDTH_RANGE_HZ
        bandwidth = max(low, min(high, requested))
        try:
            self._device.writeSetting(SOAPY_SDR_RX, 0, "CALIBRATE", str(bandwidth))
            self._event_queue.put(("status", f"LimeSDR RX calibration complete (bandwidth {bandwidth:.0f} Hz)"))
        except Exception:
            logger.exception("LimeSDR CALIBRATE failed")

    # -- TX-side helpers, used by TSG/loopback commands --------------------------------------------------------
    def _set_tx_antenna(self, name: str) -> None:
        if self._device is None:
            return
        try:
            self._device.setAntenna(SOAPY_SDR_TX, self._s.get("tx_channel", 0), name)
        except Exception:
            logger.exception("Failed to set TX antenna")

    def _enable_tsg(self, divisor: int, level_dbfs: float) -> None:
        if self._device is None:
            return
        try:
            LimeTestSignalGenerator = _import_tsg_class()
            tsg = LimeTestSignalGenerator(self._device, direction=SOAPY_SDR_TX, channel=self._s.get("tx_channel", 0))
            tsg.enable(divisor=divisor, level_dbfs=level_dbfs)
            self._tsg_active = True
        except Exception:
            logger.exception("Failed to enable TSG")

    def _disable_tsg(self) -> None:
        if self._device is None:
            return
        try:
            LimeTestSignalGenerator = _import_tsg_class()
            tsg = LimeTestSignalGenerator(self._device, direction=SOAPY_SDR_TX, channel=self._s.get("tx_channel", 0))
            tsg.disable()
        except Exception:
            logger.exception("Failed to disable TSG")
        finally:
            self._tsg_active = False

    # -- command processing -------------------------------------------------------------------------------------
    def _drain_commands(self) -> None:
        """Apply every pending command without blocking. Called once per acquisition block."""
        while True:
            try:
                cmd = self._command_queue.get_nowait()
            except queue.Empty:
                return
            self._apply_command(cmd)

    def _apply_command(self, cmd: tuple) -> None:
        name = cmd[0]
        try:
            if name == "set_center_frequency":
                self._s["center_freq"] = cmd[1]
                self._apply_frequency()
            elif name == "set_amplitude_calibration":
                self._s["cal_freqs_hz"] = cmd[1]
                self._s["cal_offsets_db"] = cmd[2]
            elif name == "clear_amplitude_calibration":
                self._s["cal_freqs_hz"] = None
                self._s["cal_offsets_db"] = None
            elif name == "set_tx_antenna":
                self._set_tx_antenna(cmd[1])
            elif name == "set_tx_gain":
                self._set_tx_gain(cmd[1])
            elif name == "enable_tsg":
                self._enable_tsg(cmd[1], cmd[2])
            elif name == "disable_tsg":
                self._disable_tsg()
            else:
                logger.warning("Unknown worker command: %r", cmd)
        except Exception:
            logger.exception("Failed to apply command %r", cmd)

    # -- main loop -----------------------------------------------------------------------------------------------
    def run(self) -> None:
        if not self._s["driver"]:
            self._event_queue.put(("error", "No SoapySDR device selected; call set_device() first."))
            self._finished_event.set()
            return

        try:
            self._open_device()
        except Exception as exc:
            self._event_queue.put(("error", str(exc)))
            self._finished_event.set()
            return

        try:
            fft_size = self._s["fft_size"]
            window = make_window(fft_size, self._s["kaiser_beta"])
            read_buf = np.zeros(fft_size, dtype=np.complex64)
            sample_rate = self._s["sample_rate"]

            if self._s.get("sweep_active"):
                self._run_sweep_loop(fft_size, window, read_buf, sample_rate)
            else:
                self._run_single_loop(fft_size, window, read_buf, sample_rate)
        finally:
            self._close_device()
            self._event_queue.put(("status", "SoapySDR receiver process stopped"))
            self._finished_event.set()

    def _capture_block(self, fft_size: int, window: np.ndarray, read_buf: np.ndarray) -> np.ndarray | None:
        """
        Fill read_buf with fft_size IQ samples via readStream(), apply software DC removal if configured, and
        return the computed power spectrum (before calibration correction or any sweep-mode cropping). Returns
        None on timeout, an unrecoverable readStream error, or if stop_event fires mid-read -- callers should
        treat None as "skip this capture, try again" (or, in composite sweep mode, "discard this whole cycle").
        """
        n_collected = 0
        timed_out = False
        while n_collected < fft_size:
            sr = self._device.readStream(
                self._rx_stream, [read_buf[n_collected:]], fft_size - n_collected, timeoutUs=200_000,
            )
            if sr.ret > 0:
                n_collected += sr.ret
            elif sr.ret == SoapySDR.SOAPY_SDR_TIMEOUT:
                timed_out = True
                break
            elif sr.ret == SoapySDR.SOAPY_SDR_OVERFLOW:
                logger.debug("SoapySDR overflow, continuing")
                continue
            else:
                self._event_queue.put(("error", f"SoapySDR readStream error: {sr.ret}"))
                timed_out = True
                break
            if self._stop_event.is_set():
                return None

        if self._stop_event.is_set() or timed_out or n_collected < fft_size:
            return None

        iq = read_buf.copy()
        if self._s["dc_offset_mode"] in ("software", "both"):
            iq = iq - np.mean(iq)
        return compute_spectrum(iq, window)

    def _emit(self, freqs: np.ndarray, power: np.ndarray) -> None:
        """Queue one (freqs, power, timestamp, port_in_use) item -- to calibration_queue while the TSG is
        active, data_queue otherwise. Shared by both the single-capture and sweep loops."""
        out = (freqs, power, time.time(), self._s.get("port_in_use"))
        target = self._calibration_queue if self._tsg_active else self._data_queue
        try:
            target.put(out)
        except Exception:
            logger.exception("Failed to enqueue spectrum data")

    def _run_single_loop(
        self, fft_size: int, window: np.ndarray, read_buf: np.ndarray, sample_rate: float
    ) -> None:
        """Ordinary single-frequency acquisition: one capture per iteration at self._s["center_freq"], which
        may change live between iterations (see set_center_frequency())."""
        center_freq = self._s["center_freq"]
        freqs = np.fft.fftshift(np.fft.fftfreq(fft_size, d=1.0 / sample_rate)) + center_freq
        freqs = freqs.astype(np.float64)

        while not self._stop_event.is_set():
            self._drain_commands()

            if self._s["center_freq"] != center_freq:
                center_freq = self._s["center_freq"]
                freqs = np.fft.fftshift(np.fft.fftfreq(fft_size, d=1.0 / sample_rate)) + center_freq
                freqs = freqs.astype(np.float64)

            if self._device is None or self._rx_stream is None:
                time.sleep(IDLE_POLL_INTERVAL_S)
                continue

            power = self._capture_block(fft_size, window, read_buf)
            if power is None:
                continue

            cal_freqs = self._s.get("cal_freqs_hz")
            cal_offsets = self._s.get("cal_offsets_db")
            if cal_freqs is not None and cal_offsets is not None:
                power = power - np.interp(freqs, cal_freqs, cal_offsets).astype(np.float32)

            self._emit(freqs, power)

    def _run_sweep_loop(
        self, fft_size: int, window: np.ndarray, read_buf: np.ndarray, sample_rate: float
    ) -> None:
        """
        Multi-segment sweep: repeatedly steps through every segment's center frequency, capturing one block per
        segment. Segment layout (centers, per-segment frequency axes, crop amount for composite mode) is fixed
        for the lifetime of a configured sweep -- live retuning isn't supported while sweeping, so none of this
        needs to be recomputed mid-loop the way the single-capture path's freqs axis does.
        """
        start_hz = self._s["sweep_start_hz"]
        num_segments = self._s["num_segments"]
        segment_span_nominal = self._s["segment_span_nominal_hz"]
        output_mode = self._s["sweep_output_mode"]
        crop_bins = compute_segment_crop_bins(fft_size, sample_rate, segment_span_nominal)

        segment_centers = [start_hz + segment_span_nominal * (i + 0.5) for i in range(num_segments)]
        segment_freqs = [
            (np.fft.fftshift(np.fft.fftfreq(fft_size, d=1.0 / sample_rate)) + c).astype(np.float64)
            for c in segment_centers
        ]

        while not self._stop_event.is_set():
            composite_power_parts: list[np.ndarray] = []
            composite_freqs_parts: list[np.ndarray] = []
            cycle_ok = True

            for i, center_freq in enumerate(segment_centers):
                if self._stop_event.is_set():
                    return
                self._drain_commands()

                if self._device is None or self._rx_stream is None:
                    time.sleep(IDLE_POLL_INTERVAL_S)
                    cycle_ok = False
                    continue

                self._apply_frequency(center_freq)
                power = self._capture_block(fft_size, window, read_buf)
                if power is None:
                    cycle_ok = False
                    continue

                freqs = segment_freqs[i]
                cal_freqs = self._s.get("cal_freqs_hz")
                cal_offsets = self._s.get("cal_offsets_db")
                if cal_freqs is not None and cal_offsets is not None:
                    power = power - np.interp(freqs, cal_freqs, cal_offsets).astype(np.float32)

                if output_mode == "segments":
                    self._emit(freqs, power)
                elif cycle_ok:
                    if i == 0:
                        lo, hi = 0, fft_size - crop_bins
                    elif i == num_segments - 1:
                        lo, hi = crop_bins, fft_size
                    else:
                        lo, hi = crop_bins, fft_size - crop_bins
                    composite_freqs_parts.append(freqs[lo:hi])
                    composite_power_parts.append(power[lo:hi])

            if output_mode == "composite" and cycle_ok and composite_power_parts and not self._stop_event.is_set():
                self._emit(np.concatenate(composite_freqs_parts), np.concatenate(composite_power_parts))


def _worker_main(
    settings: dict[str, Any],
    command_queue: multiprocessing.Queue,
    data_queue: multiprocessing.Queue,
    calibration_queue: multiprocessing.Queue,
    event_queue: multiprocessing.Queue,
    stop_event: Any,
    finished_event: Any,
) -> None:
    """multiprocessing.Process target -- constructs and runs a _ReceiverWorker."""
    worker = _ReceiverWorker(
        settings, command_queue, data_queue, calibration_queue, event_queue, stop_event, finished_event
    )
    worker.run()


# ----------------------------------------------------------------------------------------------------------------
# Parent-side: SoapyReceiverProcess is what your (QtTinySA) code actually constructs and calls.
# ----------------------------------------------------------------------------------------------------------------
class SoapyReceiverProcess:
    """
    Parent-side handle for a SoapySDR acquisition worker running in its own OS process.

    Public API deliberately mirrors soapy_receiver.SoapySDRReceiver's setters (set_device, set_center_frequency,
    set_span, set_sweep, set_rbw, set_rx_gain, set_antenna, set_bandwidth, set_dc_offset_removal,
    set_iq_balance_mode, set_auto_calibrate, set_gfir_lpf_bandwidth, set_oversampling, set_kaiser_beta,
    set_offset_tuning, set_amplitude_calibration, clear_amplitude_calibration, start, stop, wait), plus
    set_tx_gain(), set_tx_antenna(), enable_tsg(), disable_tsg() -- new here, since the threading version let
    LimeLoopbackCalibrator reach into receiver.device directly, which isn't possible across a process boundary.

    There is no spectrum_ready signal. Read data_queue (normal operation) and calibration_queue (only populated
    while a TSG tone is active -- see module docstring above) directly; QtTinySA is expected to drain data_queue
    itself in its own dedicated thread. event_queue carries status and error messages as (kind, message) tuples,
    kind being "status" or "error" -- there is only this one queue for both, not separate ones.

    Typical usage:

        rx = SoapyReceiverProcess()
        rx.set_device("lime", {"driver": "lime"}, port_in_use=port_info)
        rx.set_center_frequency(100e6)
        rx.set_rx_gain(30)
        rx.set_span(2e6)
        rx.set_rbw(10e3)
        rx.start()
        rx.set_tx_gain(20)  # TX-side settings only take effect once running -- see set_tx_gain()
        ...
        freqs, power, timestamp, port_in_use = rx.data_queue.get(timeout=1.0)
        ...
        rx.shutdown()  # stops cleanly, or forcibly terminates a wedged worker -- see shutdown()
    """

    def __init__(self) -> None:
        ctx = multiprocessing.get_context()
        self._process: multiprocessing.Process | None = None
        self._command_queue: multiprocessing.Queue = ctx.Queue()
        self.data_queue: multiprocessing.Queue = ctx.Queue()
        self.calibration_queue: multiprocessing.Queue = ctx.Queue()
        self.event_queue: multiprocessing.Queue = ctx.Queue()
        self._stop_event = ctx.Event()
        self._finished_event = ctx.Event()
        self._finished_event.set()  # idle at construction time
        self._active = False

        self._driver: str | None = None
        self._device_args: dict[str, str] = {}
        self._port_in_use: object | None = None
        self._tx_channel: int = 0

        self._center_freq: float = 100e6
        self._span: float = DEFAULT_PRESET_SAMPLE_RATES[0]
        self._sample_rate: float = self._span
        self._sweep_active: bool = False
        self._sweep_start_hz: float | None = None
        self._sweep_stop_hz: float | None = None
        self._num_segments: int = 1
        self._segment_span_nominal_hz: float | None = None
        self._sweep_output_mode: str = "segments"
        self._bandwidth: float | None = None
        self._rx_gain: float | None = None
        self._rx_agc: bool = False
        self._antenna: str | None = None
        self._preset_sample_rates: list[float] = list(DEFAULT_PRESET_SAMPLE_RATES)
        self._dc_offset_mode: str = "software"
        self._iq_balance_removal: bool = True
        self._auto_calibrate: bool = True
        self._gfir_lpf_bandwidth: float | None = None
        self._oversampling: int = 0
        self._tuning_offset_hz: float = 0.0
        self._kaiser_beta: float = DEFAULT_KAISER_BETA
        self._rbw: float = 10_000.0
        self._fft_size: int = calculate_fft_size(self._rbw, self._sample_rate, self._kaiser_beta)
        self._cal_freqs_hz: np.ndarray | None = None
        self._cal_offsets_db: np.ndarray | None = None

    # ------------------------------------------------------------------
    @staticmethod
    def list_devices() -> list[SoapyDeviceInfo]:
        return list_devices()

    def _is_lime_driver(self) -> bool:
        return is_lime_driver(self._driver)

    def _check_idle(self, what: str) -> None:
        if self._active:
            raise ReceiverBusyError(
                f"Cannot change {what} while the receiver is running. Call stop() and wait() first."
            )

    # ------------------------------------------------------------------
    # Setters requiring stop()/wait()/start() -- unchanged contract from soapy_receiver.py
    # ------------------------------------------------------------------
    def set_device(
        self, driver: str, device_args: dict[str, str] | None = None, port_in_use: object | None = None,
        tx_channel: int = 0,
    ) -> None:
        """Select which SoapySDR driver/device to use. port_in_use is carried verbatim in every queued
        spectrum tuple; QtTinySA always supplies this. tx_channel selects which TX channel set_tx_antenna()/
        enable_tsg() operate on (default 0)."""
        self._check_idle("device")
        self._driver = driver
        self._device_args = dict(device_args or {"driver": driver})
        self._port_in_use = port_in_use
        self._tx_channel = tx_channel

    def set_antenna(self, antenna: str) -> None:
        self._check_idle("antenna")
        self._antenna = antenna

    def set_span(self, span_hz: float) -> None:
        self._check_idle("span")
        self._sample_rate = choose_sample_rate_for_span(span_hz, self._preset_sample_rates)
        self._span = span_hz
        self._sweep_active = False

    def set_sweep(self, start_hz: float, stop_hz: float, output_mode: str = "segments") -> None:
        """
        Configure a multi-segment sweep from start_hz to stop_hz, for spans wider than any single preset sample
        rate can capture in one block. Splits the requested range into the smallest number of equal-width
        segments whose width fits within the largest available preset sample rate, retunes between them, and
        either emits each segment separately (output_mode="segments", the default -- closer to how a normal
        swept-tuned spectrum analyzer behaves) or stitches them into one fixed-length composite spectrum per
        full sweep cycle (output_mode="composite").

        Call set_rbw() afterwards, same ordering requirement as set_span(). Query num_segments, segment_span_hz,
        and (for composite mode) composite_size after both calls to size things on your side before start() --
        all three are pure functions of current settings and need no running worker.

        Segments mode: each data_queue item is one full captured segment (freqs, power, timestamp, port_in_use),
        the same 4-tuple shape as always -- including a small amount of overlap at segment edges if the chosen
        sample rate is wider than the nominal per-segment slice, since each segment's own freqs array already
        shows where it sits, so no extra fields are needed to place it. If one segment's capture times out in a
        given cycle, the others are still emitted; composite mode instead drops that whole cycle's composite
        rather than emit a short array (see _run_sweep_loop()).

        Composite mode: segments are cropped at internal boundaries before concatenating, so the composite's
        length is fixed and known ahead of time regardless of rounding; the outer edges of the first and last
        segment are left uncropped, so the composite's actual frequency coverage (see
        composite_frequency_range_hz) is very slightly wider than [start_hz, stop_hz], never narrower.

        Live center_frequency retuning and RF/NCO offset tuning are not supported while a sweep is configured --
        set_center_frequency() is ignored (logged) during a sweep, and set_offset_tuning() has no effect.
        """
        self._check_idle("sweep")
        if stop_hz <= start_hz:
            raise ValueError("stop_hz must be greater than start_hz")
        if output_mode not in ("segments", "composite"):
            raise ValueError("output_mode must be 'segments' or 'composite'")

        total_span = stop_hz - start_hz
        max_preset = max(self._preset_sample_rates)
        num_segments = max(1, math.ceil(total_span / max_preset))
        segment_span_nominal = total_span / num_segments
        sample_rate = choose_sample_rate_for_span(segment_span_nominal, self._preset_sample_rates)

        self._sweep_active = True
        self._sweep_start_hz = start_hz
        self._sweep_stop_hz = stop_hz
        self._num_segments = num_segments
        self._segment_span_nominal_hz = segment_span_nominal
        self._sweep_output_mode = output_mode
        self._span = sample_rate
        self._sample_rate = sample_rate

    def set_rbw(self, rbw_hz: float) -> None:
        self._check_idle("RBW")
        fft_size = calculate_fft_size(rbw_hz, self._sample_rate, self._kaiser_beta)
        self._rbw = float(rbw_hz)
        self._fft_size = fft_size

    def set_preset_sample_rates(self, rates: list[float]) -> None:
        self._check_idle("preset sample rates")
        self._preset_sample_rates = sorted(rates)

    def set_kaiser_beta(self, beta: float) -> None:
        self._check_idle("Kaiser beta")
        old_beta = self._kaiser_beta
        self._kaiser_beta = float(beta)
        try:
            self.set_rbw(self._rbw)
        except ValueError:
            self._kaiser_beta = old_beta
            raise

    def set_rx_gain(self, gain_db: float | None) -> None:
        """Set the RX gain. Pass None to enable the device's automatic gain control."""
        self._check_idle("RX gain")
        self._rx_gain = gain_db
        self._rx_agc = gain_db is None

    def set_tx_gain(self, gain_db: float) -> None:
        """
        Set the TX gain used for TSG/loopback purposes (see set_tx_antenna(), enable_tsg()). Only meaningful
        while running -- sent as a command applied against the worker's live device, mirroring
        set_tx_antenna(); a no-op with a logged warning if called before start().

        Fixing this explicitly matters for external-power-meter calibration workflows: the TSG's actual RF
        output level depends on TX gain, which otherwise defaults to whatever the driver picks. Use the
        identical value here in both the power-meter reference-measurement session and the later loopback
        sweep (LimeLoopbackCalibrator), or the two measurements won't agree with each other.
        """
        if self._active:
            self._command_queue.put(("set_tx_gain", gain_db))
        else:
            logger.warning("set_tx_gain() called while not running; command dropped")

    def set_bandwidth(self, bandwidth_hz: float | None) -> None:
        self._check_idle("bandwidth")
        self._bandwidth = bandwidth_hz

    def set_dc_offset_removal(self, mode: str) -> None:
        """
        Set DC offset removal mode: one of DC_OFFSET_MODES ("software", "hardware", "both", "none").
        "hardware" uses the device's own correction where supported (falls back to a no-op, logged, if not);
        "software" subtracts the measured mean IQ offset from every captured block, which always works
        regardless of hardware support; "both" runs both simultaneously -- worth trying if hardware alone isn't
        removing the offset adequately, since hardware handles gross/analog offset before the ADC while software
        mops up whatever residual or drift remains; "none" disables removal entirely.
        """
        if mode not in DC_OFFSET_MODES:
            raise ValueError(f"mode must be one of {DC_OFFSET_MODES}, got {mode!r}")
        self._check_idle("DC offset removal")
        self._dc_offset_mode = mode

    def set_iq_balance_mode(self, enabled: bool) -> None:
        self._check_idle("IQ balance mode")
        self._iq_balance_removal = bool(enabled)

    def set_auto_calibrate(self, enabled: bool) -> None:
        self._check_idle("auto-calibrate")
        self._auto_calibrate = bool(enabled)

    def set_gfir_lpf_bandwidth(self, bandwidth_hz: float | None) -> None:
        self._check_idle("GFIR LPF bandwidth")
        self._gfir_lpf_bandwidth = bandwidth_hz

    def set_oversampling(self, ratio: int) -> None:
        if ratio not in LIME_VALID_OVERSAMPLING_RATIOS:
            raise ValueError(f"ratio must be one of {LIME_VALID_OVERSAMPLING_RATIOS}, got {ratio}")
        self._check_idle("oversampling ratio")
        self._oversampling = ratio

    def set_offset_tuning(self, offset_hz: float) -> None:
        self._check_idle("offset tuning")
        self._tuning_offset_hz = float(offset_hz)

    # ------------------------------------------------------------------
    # Live-safe: applied locally now, or sent to the worker if already running
    # ------------------------------------------------------------------
    def set_center_frequency(self, freq_hz: float) -> None:
        """Safe to call while running -- sent to the worker process, applied at the top of its next
        acquisition-block iteration, same timing/semantics as soapy_receiver.SoapySDRReceiver. Has no effect
        (logged, not raised) while a sweep is configured -- see set_sweep()."""
        if self._sweep_active:
            logger.warning("set_center_frequency() has no effect while a sweep is configured (see set_sweep())")
            return
        self._center_freq = float(freq_hz)
        if self._active:
            self._command_queue.put(("set_center_frequency", self._center_freq))

    @property
    def num_segments(self) -> int:
        """Number of segments a configured sweep splits into; 1 outside of sweep mode. Pure function of
        current settings -- safe to query before start()."""
        return self._num_segments

    @property
    def segment_span_hz(self) -> float:
        """The actual per-capture bandwidth: one segment's width in sweep mode, or the single span otherwise."""
        return self._sample_rate

    @property
    def composite_size(self) -> int:
        """
        Length of the stitched array a composite-mode sweep emits. Computed the same way regardless of which
        output_mode set_sweep() was actually given -- it tells you what a composite WOULD be even while
        "segments" mode is configured, which is occasionally useful (e.g. deciding whether to switch modes) but
        is NOT the length of what you'll actually receive from data_queue in segments mode.

        In "segments" mode, every item received is a full, uncropped single segment -- always exactly fft_size
        long, never composite_size. The two are not related by a simple division (composite_size has every
        internal-boundary overlap removed; segments don't), so don't derive one from the other -- use fft_size
        directly for segment length, composite_size only when output_mode="composite".

        Reduces to fft_size when num_segments is 1 (no sweep configured), since there's nothing to crop/stitch.
        Pure function of current settings -- safe to query before start(), so you can size a display buffer
        ahead of time regardless of which mode you're using.
        """
        segment_span_nominal = self._segment_span_nominal_hz or self._sample_rate
        crop_bins = compute_segment_crop_bins(self._fft_size, self._sample_rate, segment_span_nominal)
        return compute_composite_size(self._fft_size, self._num_segments, crop_bins)

    @property
    def composite_frequency_range_hz(self) -> tuple[float, float] | None:
        """
        Actual frequency coverage of a composite-mode sweep's output -- slightly wider than the requested
        [start_hz, stop_hz] passed to set_sweep(), since the outer edges of the first/last segment are left
        uncropped rather than clipped. None if no sweep is configured.

        Computed directly from the first/last segment's own uncropped outer bin, matching exactly what the
        worker actually emits -- not reconstructed via crop_bins*bin_spacing, which would disagree with it by a
        rounding error (crop_bins is a rounded integer bin count; the true half-width excess,
        (sample_rate - segment_span_nominal_hz)/2, generally isn't a whole number of bins). The lower and upper
        edges also aren't symmetric around each endpoint's own center: fftshift(fftfreq(...)) for an even-length
        FFT runs from exactly -sample_rate/2 up to +sample_rate/2 - bin_spacing (one bin short), since there's
        one more negative-frequency bin than positive by convention -- confirmed empirically, not just derived.
        """
        if not self._sweep_active or self._segment_span_nominal_hz is None:
            return None
        bin_spacing = self._sample_rate / self._fft_size
        half_excess_hz = (self._sample_rate - self._segment_span_nominal_hz) / 2.0
        return (
            self._sweep_start_hz - half_excess_hz,
            self._sweep_stop_hz + half_excess_hz - bin_spacing,
        )

    @property
    def segment_centers_hz(self) -> list[float]:
        """Center frequency of each segment in a configured sweep; [center_freq] outside of sweep mode."""
        if not self._sweep_active or self._segment_span_nominal_hz is None:
            return [self._center_freq]
        return [
            self._sweep_start_hz + self._segment_span_nominal_hz * (i + 0.5) for i in range(self._num_segments)
        ]

    def set_amplitude_calibration(self, freqs_hz: np.ndarray, offsets_db: np.ndarray) -> None:
        """
        Install a per-frequency amplitude correction table: every emitted spectrum's power is corrected as
        power - interp(freq, freqs_hz, offsets_db), linearly interpolated between points and holding the
        nearest endpoint's value outside the calibrated range. Typically populated from
        LimeLoopbackCalibrator.run_calibration_sweep()'s result. Safe to call while running -- sent to the
        worker as a command, applied from the next capture onward.
        """
        freqs_hz = np.asarray(freqs_hz, dtype=np.float64)
        offsets_db = np.asarray(offsets_db, dtype=np.float64)
        if freqs_hz.shape != offsets_db.shape or freqs_hz.ndim != 1:
            raise ValueError("freqs_hz and offsets_db must be 1-D arrays of the same length")
        order = np.argsort(freqs_hz)
        self._cal_freqs_hz = freqs_hz[order]
        self._cal_offsets_db = offsets_db[order]
        if self._active:
            self._command_queue.put(("set_amplitude_calibration", self._cal_freqs_hz, self._cal_offsets_db))

    def clear_amplitude_calibration(self) -> None:
        self._cal_freqs_hz = None
        self._cal_offsets_db = None
        if self._active:
            self._command_queue.put(("clear_amplitude_calibration",))

    def set_tx_antenna(self, name: str) -> None:
        """Set the TX antenna used for TSG/loopback purposes. Only meaningful while running -- the worker
        applies it against its live device; there is nothing to do if the device isn't open yet."""
        if self._active:
            self._command_queue.put(("set_tx_antenna", name))
        else:
            logger.warning("set_tx_antenna() called while not running; command dropped")

    def enable_tsg(self, divisor: int = 4, level_dbfs: float = -6.0) -> None:
        """Enable the LimeSDR TSG tone on the TX side. Only meaningful while running. See lime_tsg_mp.py."""
        if self._active:
            self._command_queue.put(("enable_tsg", divisor, level_dbfs))
        else:
            logger.warning("enable_tsg() called while not running; command dropped")

    def disable_tsg(self) -> None:
        if self._active:
            self._command_queue.put(("disable_tsg",))
        else:
            logger.warning("disable_tsg() called while not running; command dropped")

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------
    def _snapshot_settings(self) -> dict[str, Any]:
        return {
            "driver": self._driver,
            "device_args": dict(self._device_args),
            "port_in_use": self._port_in_use,
            "tx_channel": self._tx_channel,
            "center_freq": self._center_freq,
            "sweep_active": self._sweep_active,
            "sweep_start_hz": self._sweep_start_hz,
            "sweep_stop_hz": self._sweep_stop_hz,
            "num_segments": self._num_segments,
            "segment_span_nominal_hz": self._segment_span_nominal_hz,
            "sweep_output_mode": self._sweep_output_mode,
            "sample_rate": self._sample_rate,
            "bandwidth": self._bandwidth,
            "rx_gain": self._rx_gain,
            "rx_agc": self._rx_agc,
            "antenna": self._antenna,
            "fft_size": self._fft_size,
            "dc_offset_mode": self._dc_offset_mode,
            "iq_balance_removal": self._iq_balance_removal,
            "auto_calibrate": self._auto_calibrate,
            "gfir_lpf_bandwidth": self._gfir_lpf_bandwidth,
            "oversampling": self._oversampling,
            "tuning_offset_hz": self._tuning_offset_hz,
            "kaiser_beta": self._kaiser_beta,
            "cal_freqs_hz": self._cal_freqs_hz,
            "cal_offsets_db": self._cal_offsets_db,
        }

    def start(self) -> None:
        self._check_idle("start")
        if not self._driver:
            raise RuntimeError("No SoapySDR device selected; call set_device() first.")
        ctx = multiprocessing.get_context()
        self._stop_event = ctx.Event()
        self._finished_event = ctx.Event()
        self._active = True
        self._process = ctx.Process(
            target=_worker_main,
            args=(
                self._snapshot_settings(), self._command_queue, self.data_queue, self.calibration_queue,
                self.event_queue, self._stop_event, self._finished_event,
            ),
            daemon=True,
        )
        self._process.start()

    def stop(self) -> None:
        """Request the worker process to stop. Call wait() afterwards to confirm it has."""
        self._stop_event.set()

    def wait(self, timeout: float | None = None) -> bool:
        """Block until the worker process has fully exited (device closed). Also joins the OS process itself
        once the exit event fires, so no zombie process is left behind."""
        finished = self._finished_event.wait(timeout)
        if finished and self._process is not None:
            self._process.join(timeout=5.0)
            self._active = False
        return finished

    def shutdown(self, timeout: float = 5.0) -> None:
        """
        Stop the worker process and guarantee it's gone -- the function to call when disconnecting the device or
        closing the application. Tries a clean stop()/wait() first (the worker closes its SoapySDR device and
        stream normally); if it hasn't exited within timeout seconds -- e.g. it's stuck inside a blocked
        readStream() call to a wedged device, or some other hang -- forcibly terminates the OS process instead
        of leaving it orphaned or hanging the caller indefinitely. Safe to call even if start() was never called,
        or if the receiver is already stopped; does nothing harmful in either case.
        """
        self.stop()
        if self.wait(timeout=timeout):
            return
        if self._process is not None and self._process.is_alive():
            logger.warning("Worker process did not stop within %.1fs; terminating it forcibly", timeout)
            self._process.terminate()
            self._process.join(timeout=2.0)
            if self._process.is_alive():
                logger.error("Worker process did not respond to terminate(); killing it")
                self._process.kill()
                self._process.join(timeout=2.0)
        self._active = False

    @property
    def port_in_use(self) -> object | None:
        return self._port_in_use

    @property
    def fft_size(self) -> int:
        """
        The currently computed FFT size, as last set by set_rbw() -- or the constructor's default if set_rbw()
        hasn't been called yet. This is the length of each individual capture: every data_queue item outside of
        sweep mode, and every segment in "segments" sweep mode -- but NOT a composite-mode sweep's output, which
        is composite_size instead (see its docstring for why the two aren't simply related by num_segments).
        Purely a parent-side computed value; safe to read at any time, including before start() is ever called,
        since it never depends on the worker process being alive.
        """
        return self._fft_size
