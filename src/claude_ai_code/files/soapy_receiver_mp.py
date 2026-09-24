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

Portability note: multiprocessing's default start method is "fork" on Linux (cheap, shares already-imported
code via copy-on-write) but "spawn" on Windows and macOS (each child re-imports this module fresh). Under
"spawn", the code that constructs a SoapyReceiverProcess and calls start() must sit behind
`if __name__ == "__main__":` in your top-level script, or the child process will try to re-run it. This is a
standard multiprocessing constraint, not specific to this module.
"""

import logging
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
    """Equivalent noise bandwidth (in bins) of a numpy Kaiser window with the given beta. See soapy_receiver.py
    for the full rationale -- this compensates calculate_fft_size() for windowing's main-lobe broadening."""
    w = np.kaiser(reference_size, beta)
    return float(reference_size * np.sum(w ** 2) / (np.sum(w) ** 2))


def calculate_fft_size(
    rbw_hz: float,
    sample_rate: float,
    beta: float = DEFAULT_KAISER_BETA,
    max_fft_size: int = MAX_FFT_SIZE,
    min_fft_size: int = MIN_FFT_SIZE,
) -> int:
    """Compute the FFT size needed for rbw_hz at sample_rate; raises ValueError if unachievable within
    max_fft_size. See soapy_receiver.py's calculate_fft_size() for full details -- identical logic."""
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
    """Coherent-gain-normalized numpy Kaiser window. See soapy_receiver.py -- identical logic."""
    win = np.kaiser(size, beta)
    coherent_gain = win.mean()
    if coherent_gain <= 0:
        coherent_gain = 1.0
    return (win / coherent_gain).astype(np.float32)


def compute_spectrum(iq: np.ndarray, window: np.ndarray) -> np.ndarray:
    """Windowed FFT power in dB. See soapy_receiver.py -- identical logic."""
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

    def _open_device(self) -> None:
        try:
            self._device = SoapySDR.Device(self._s["device_args"])
        except Exception as exc:
            raise RuntimeError(f"Failed to open SoapySDR device: {exc}") from exc

        self._apply_oversampling()
        self._apply_antenna()
        self._apply_rate_and_freq()
        self._apply_gfir_lpf()
        self._apply_gain()
        self._apply_bandwidth()

        if self._s["auto_calibrate"] and self._is_lime():
            self._calibrate()

        self._apply_dc_offset()
        self._apply_iq_balance()

        try:
            self._rx_stream = self._device.setupStream(SOAPY_SDR_RX, 0, SOAPY_SDR_CF32)
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

    def _apply_frequency(self) -> None:
        if self._device is None:
            return
        center_freq = self._s["center_freq"]
        offset_hz = self._s["tuning_offset_hz"]
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

    def _apply_gain(self) -> None:
        if self._device is None:
            return
        try:
            if hasattr(self._device, "setGainMode"):
                self._device.setGainMode(SOAPY_SDR_RX, 0, bool(self._s["agc"]))
            if not self._s["agc"] and self._s["gain"] is not None:
                self._device.setGain(SOAPY_SDR_RX, 0, self._s["gain"])
        except Exception:
            logger.exception("Failed to set gain")

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
        removal = self._s["dc_offset_removal"]
        mode = self._s["dc_offset_mode"]
        if removal and mode == "hardware":
            try:
                if self._device.hasDCOffsetMode(SOAPY_SDR_RX, 0):
                    self._device.setDCOffsetMode(SOAPY_SDR_RX, 0, True)
                else:
                    logger.info("No hardware DC offset correction; falling back to software removal")
                    self._s["dc_offset_mode"] = "software"
            except Exception:
                logger.exception("Failed to set hardware DC offset mode")
        else:
            try:
                if self._device.hasDCOffsetMode(SOAPY_SDR_RX, 0):
                    self._device.setDCOffsetMode(SOAPY_SDR_RX, 0, False)
            except Exception:
                pass

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
        # Imported lazily so a worker that never uses the TSG doesn't need lime_tsg_mp importable/available.
        from lime_tsg_mp import LimeTestSignalGenerator

        try:
            tsg = LimeTestSignalGenerator(self._device, direction=SOAPY_SDR_TX, channel=self._s.get("tx_channel", 0))
            tsg.enable(divisor=divisor, level_dbfs=level_dbfs)
            self._tsg_active = True
        except Exception:
            logger.exception("Failed to enable TSG")

    def _disable_tsg(self) -> None:
        if self._device is None:
            return
        from lime_tsg_mp import LimeTestSignalGenerator

        try:
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
                        break

                if self._stop_event.is_set():
                    break
                if timed_out or n_collected < fft_size:
                    continue

                iq = read_buf.copy()
                if self._s["dc_offset_removal"] and self._s["dc_offset_mode"] == "software":
                    iq = iq - np.mean(iq)

                power = compute_spectrum(iq, window)

                cal_freqs = self._s.get("cal_freqs_hz")
                cal_offsets = self._s.get("cal_offsets_db")
                if cal_freqs is not None and cal_offsets is not None:
                    power = power - np.interp(freqs, cal_freqs, cal_offsets).astype(np.float32)

                out = (freqs, power, time.time(), self._s.get("port_in_use"))
                target = self._calibration_queue if self._tsg_active else self._data_queue
                try:
                    target.put(out)
                except Exception:
                    logger.exception("Failed to enqueue spectrum data")
        finally:
            self._close_device()
            self._event_queue.put(("status", "SoapySDR receiver process stopped"))
            self._finished_event.set()


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
    set_span, set_rbw, set_gain, set_antenna, set_bandwidth, set_dc_offset_removal, set_iq_balance_mode,
    set_auto_calibrate, set_gfir_lpf_bandwidth, set_oversampling, set_kaiser_beta, set_offset_tuning,
    set_amplitude_calibration, clear_amplitude_calibration, start, stop, wait), plus set_tx_antenna(),
    enable_tsg(), disable_tsg() -- new here, since the threading version let LimeLoopbackCalibrator reach into
    receiver.device directly, which isn't possible across a process boundary.

    There is no spectrum_ready signal. Read data_queue (normal operation) and calibration_queue (only populated
    while a TSG tone is active -- see class docstring above) directly; QtTinySA is expected to drain data_queue
    itself in its own dedicated thread. error_queue and status_queue carry the equivalents of soapy_receiver's
    error/status signals as plain (message,) tuples.

    Typical usage:

        rx = SoapyReceiverProcess()
        rx.set_device("lime", {"driver": "lime"}, port_in_use=port_info)
        rx.set_center_frequency(100e6)
        rx.set_gain(30)
        rx.set_span(2e6)
        rx.set_rbw(10e3)
        rx.start()
        ...
        freqs, power, timestamp, port_in_use = rx.data_queue.get(timeout=1.0)
        ...
        rx.stop()
        rx.wait()
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
        self._bandwidth: float | None = None
        self._gain: float | None = None
        self._agc: bool = False
        self._antenna: str | None = None
        self._preset_sample_rates: list[float] = list(DEFAULT_PRESET_SAMPLE_RATES)
        self._dc_offset_removal: bool = True
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
        if span_hz <= 0:
            raise ValueError("span_hz must be > 0")
        presets = sorted(self._preset_sample_rates)
        chosen = next((rate for rate in presets if rate >= span_hz), None)
        if chosen is None:
            raise ValueError(
                f"Requested span {span_hz:.0f} Hz exceeds the largest available sample rate "
                f"({presets[-1]:.0f} Hz); this receiver cannot capture a wider span."
            )
        self._span = span_hz
        self._sample_rate = chosen

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

    def set_gain(self, gain_db: float | None) -> None:
        self._check_idle("gain")
        self._gain = gain_db
        self._agc = gain_db is None

    def set_bandwidth(self, bandwidth_hz: float | None) -> None:
        self._check_idle("bandwidth")
        self._bandwidth = bandwidth_hz

    def set_dc_offset_removal(self, enabled: bool, mode: str = "software") -> None:
        if mode not in ("software", "hardware"):
            raise ValueError("mode must be 'software' or 'hardware'")
        self._check_idle("DC offset removal")
        self._dc_offset_removal = enabled
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
        acquisition-block iteration, same timing/semantics as soapy_receiver.SoapySDRReceiver."""
        self._center_freq = float(freq_hz)
        if self._active:
            self._command_queue.put(("set_center_frequency", self._center_freq))

    def set_amplitude_calibration(self, freqs_hz: np.ndarray, offsets_db: np.ndarray) -> None:
        """Safe to call while running. See soapy_receiver.py's version for the full rationale."""
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
            "sample_rate": self._sample_rate,
            "bandwidth": self._bandwidth,
            "gain": self._gain,
            "agc": self._agc,
            "antenna": self._antenna,
            "fft_size": self._fft_size,
            "dc_offset_removal": self._dc_offset_removal,
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

    @property
    def port_in_use(self) -> object | None:
        return self._port_in_use
