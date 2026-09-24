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
soapy_receiver.py

SoapySDR streaming module for QtTinySA.

Runs SoapySDR device acquisition on a QThreadPool worker (QRunnable) and emits computed spectrum (FFT power) data via Qt
signals connected to QtTinySA's router function on the GUI thread. PySide6's queued signal/slot connections make that
cross-thread delivery safe on their own -- Qt determines the connection type by comparing the emitting thread against
the thread affinity of the object that owns the connected slot, not the object that owns the signal, so emitting from a
pool thread is automatically queued to the GUI thread.

Threading model: settings (device, frequency, gain, antenna, RBW, Kaiser beta, DC offset mode) are plain attributes, not
mutex-protected. This assumes QtTinySA's usual pattern of stop() -> wait() -> change settings -> start() for anything
that requires reopening the SoapySDR stream anyway (sample rate / FFT size / device / antenna changes all fall in this
category). wait() blocks on a threading.Event that the worker sets when its run loop actually exits, so once it returns
there is no worker thread left to race with. Setters therefore simply refuse to run while a worker is active rather than
trying to apply changes live.

Requires:
    pip install PySide6 numpy
    SoapySDR Python bindings (build/install from
    https://github.com/pothosware/SoapySDR/wiki/PythonSupport)
    Any device-specific SoapySDR modules you need, e.g. SoapyLMS7 for LimeSDR, SoapyRTLSDR for RTL-SDR dongles, etc.
"""

import logging
import threading
import time
from dataclasses import dataclass, field

import numpy as np
from PySide6.QtCore import QObject, QRunnable, QThreadPool, Signal

try:
    import SoapySDR
    from SoapySDR import SOAPY_SDR_RX, SOAPY_SDR_CF32
except ImportError as exc:  # pragma: no cover
    raise ImportError(
        "SoapySDR Python bindings not found. Install SoapySDR with Python "
        "support: https://github.com/pothosware/SoapySDR/wiki/PythonSupport"
    ) from exc

logger = logging.getLogger(__name__)


# ----------------------------------------------------------------------------------------------------------------
# Preset sample rates. QtTinySA/LTE-style SDR pipelines commonly derive their sample rates as integer divisions of
# a 30.72 MHz base clock, so the defaults below are 30.72 MHz divided by successive powers of two, with 30.72 MHz
# itself as the maximum (no multiples are used, since any multiple would exceed that maximum).
# ----------------------------------------------------------------------------------------------------------------
BASE_CLOCK_HZ = 30_720_000.0  # 30.72 MHz
DEFAULT_SAMPLE_RATE_DIVISORS: tuple[int, ...] = (1, 2, 4, 8, 16, 32, 64, 128)
DEFAULT_PRESET_SAMPLE_RATES: tuple[float, ...] = tuple(
    BASE_CLOCK_HZ / d for d in DEFAULT_SAMPLE_RATE_DIVISORS
)

MAX_FFT_SIZE = 65536  # ceiling so a very narrow RBW can't stall the UI
MIN_FFT_SIZE = 256

# Default numpy Kaiser window beta. ~10.5 gives sidelobe suppression comparable to a Blackman-Harris window while
# staying tunable via a single scalar.
DEFAULT_KAISER_BETA = 10.5

# Valid ratios for LimeSDR's "OVERSAMPLING" writeSetting (SoapyLMS7-specific); 0 lets LimeSuite auto-select.
LIME_VALID_OVERSAMPLING_RATIOS: tuple[int, ...] = (0, 1, 2, 4, 8, 16, 32)

# Valid range for LimeSDR's "CALIBRATE" writeSetting bandwidth, per SoapySDRUtil --probe on both the classic
# SoapyLMS7 (LimeSuite) and SoapyLMS (LimeSuiteNG) drivers.
LIME_CALIBRATE_BANDWIDTH_RANGE_HZ: tuple[float, float] = (2.5e6, 1.2e8)


@dataclass
class SoapyDeviceInfo:
    """Convenience wrapper around a SoapySDR enumeration result."""
    driver: str
    label: str
    args: dict[str, str] = field(default_factory=dict)


class ReceiverBusyError(RuntimeError):
    """Raised when a setting is changed, or start() is called, while a worker is active."""


class SoapySDRReceiver(QObject):
    """
    Owns a SoapySDR RX stream and continuously pushes computed spectrum data to the GUI thread via Qt signals.
    Acquisition runs on a dedicated QThreadPool (a single-worker pool owned by this instance) rather than a QThread
    subclass.

    Typical usage from QtTinySA (stop/reconfigure/start pattern):

        self.sdr = SoapySDRReceiver()
        self.sdr.spectrum_ready.connect(self.router)   # queued, cross-thread safe
        self.sdr.error.connect(self.on_sdr_error)
        self.sdr.status.connect(self.on_sdr_status)

        self.sdr.set_device("lime", {"driver": "lime"}, port_in_use=port_info)  # port_info from QtTinySA
        self.sdr.set_antenna("LNAW")                    # LimeSDR only
        self.sdr.set_center_frequency(100e6)
        self.sdr.set_gain(30)                            # or set_gain(None) for AGC
        self.sdr.set_kaiser_beta(10.5)
        self.sdr.set_span(2e6)                            # sets sample rate; call before set_rbw()
        try:
            self.sdr.set_rbw(10e3)                        # picks FFT size for the current span
        except ValueError as exc:
            ...                                           # QtTinySA should show this to the user
        self.sdr.set_dc_offset_removal(True, mode="software")
        self.sdr.start()
        ...
        # to change any setting later:
        self.sdr.stop()
        self.sdr.wait()
        self.sdr.set_center_frequency(433.9e6)
        self.sdr.start()
    """

    # (freq_array_hz, power_array_db, timestamp_s, port_in_use) -- port_in_use is whatever set_device() was
    # given (a QtTinySA FakePortInfo/pyserial ListPortInfo, typically), carried through unmodified.
    spectrum_ready = Signal(np.ndarray, np.ndarray, float, object)
    error = Signal(str)
    device_list_ready = Signal(list)
    status = Signal(str)

    def __init__(self, parent: QObject | None = None):
        super().__init__(parent)

        self._pool = QThreadPool()
        self._pool.setMaxThreadCount(1)
        self._active = False
        self._running = False
        self._finished_event = threading.Event()
        self._finished_event.set()  # idle at construction time

        # Device state
        self._driver: str | None = None
        self._device_args: dict[str, str] = {}
        self._device: SoapySDR.Device | None = None
        self._rx_stream = None
        self._port_in_use: object | None = None

        # RF / acquisition settings -- plain attributes, only changed while no worker is active (see _check_idle)
        self._center_freq: float = 100e6
        self._span: float = DEFAULT_PRESET_SAMPLE_RATES[0]        # usable span == sample rate for IQ capture
        self._sample_rate: float = self._span
        self._bandwidth: float | None = None
        self._gain: float | None = None          # None => device AGC
        self._agc: bool = False
        self._antenna: str | None = None
        self._preset_sample_rates: list[float] = list(DEFAULT_PRESET_SAMPLE_RATES)
        self._dc_offset_removal: bool = True
        self._dc_offset_mode: str = "software"    # "software" or "hardware"
        self._iq_balance_removal: bool = True     # hardware automatic IQ balance, where supported
        self._auto_calibrate: bool = True         # run LimeSDR CALIBRATE_RX on (re)open, LimeSDR only
        self._gfir_lpf_bandwidth: float | None = None     # Hz; None leaves LimeSuite's own GFIR setting alone
        self._oversampling: int = 0               # 0 = auto; LimeSDR-specific OVERSAMPLING setting
        self._tuning_offset_hz: float = 0.0       # RF/BB NCO offset-tuning shift; 0 = disabled
        self._kaiser_beta: float = DEFAULT_KAISER_BETA
        self._rbw: float = 10_000.0
        self._fft_size: int = self.calculate_fft_size(self._rbw, self._sample_rate, self._kaiser_beta)
        self._window: np.ndarray = self._make_window(self._fft_size, self._kaiser_beta)

        # Optional amplitude calibration table (frequency_hz -> offset_db), set via set_amplitude_calibration()
        # -- e.g. by LimeLoopbackCalibrator after a calibration sweep. When present, it is applied to every
        # emitted spectrum automatically (see _acquisition_loop()); None means no correction is applied.
        self._cal_freqs_hz: np.ndarray | None = None
        self._cal_offsets_db: np.ndarray | None = None

    # ------------------------------------------------------------------------------------------------------------------
    # Static / class helpers
    # ------------------------------------------------------------------------------------------------------------------
    @staticmethod
    def list_devices() -> list[SoapyDeviceInfo]:
        """Enumerate all SoapySDR devices currently visible on the system."""
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

    @staticmethod
    def _kaiser_enbw_factor(beta: float, reference_size: int = 8192) -> float:
        """
        Equivalent noise bandwidth (in bins) of a numpy Kaiser window with the given beta, estimated from a
        reference-length window. This value converges very quickly with window length and is used to compensate the
        FFT-size calculation for the main-lobe broadening that windowing introduces, so the achieved resolution
        bandwidth tracks the requested RBW rather than just sample_rate / fft_size (which is only correct for a
        rectangular window).
        """
        w = np.kaiser(reference_size, beta)
        return float(reference_size * np.sum(w ** 2) / (np.sum(w) ** 2))

    @classmethod
    def calculate_fft_size(
        cls,
        rbw_hz: float,
        sample_rate: float,
        beta: float = DEFAULT_KAISER_BETA,
        max_fft_size: int = MAX_FFT_SIZE,
        min_fft_size: int = MIN_FFT_SIZE,
    ) -> int:
        """
        Compute the power-of-two FFT size needed to achieve the requested RBW at a given sample rate, accounting for
        the main-lobe broadening introduced by the numpy Kaiser window (via its equivalent noise bandwidth, ENBW).

        Raises ValueError if rbw_hz is too narrow to achieve at this sample rate within max_fft_size -- e.g. because
        the span (== sample_rate) is too wide for the requested RBW. Callers (such as QtTinySA's RBW control) should
        catch this and reject/clamp the requested RBW rather than silently substituting a different one.
        """
        if rbw_hz <= 0:
            raise ValueError("rbw_hz must be > 0")
        if sample_rate <= 0:
            raise ValueError("sample_rate must be > 0")

        enbw_factor = cls._kaiser_enbw_factor(beta)
        ideal = (sample_rate / rbw_hz) * enbw_factor
        fft_size = 1 << max(1, int(ideal - 1)).bit_length()  # next power of two >= ideal

        if fft_size > max_fft_size:
            achievable_rbw = (sample_rate / max_fft_size) * enbw_factor
            raise ValueError(
                f"Requested RBW {rbw_hz:.1f} Hz is not achievable at a {sample_rate:.0f} Hz sample rate "
                f"(span) with Kaiser beta {beta:.2f} and a max FFT size of {max_fft_size}. The finest "
                f"achievable RBW at this span is approximately {achievable_rbw:.1f} Hz -- increase the "
                f"RBW, reduce the span, or lower the Kaiser beta."
            )

        return max(min_fft_size, fft_size)

    @staticmethod
    def _make_window(size: int, beta: float) -> np.ndarray:
        """
        Build a coherent-gain-normalized numpy Kaiser window. Dividing by the window's own mean means a full-scale
        sinusoid reports the same peak power regardless of beta -- changing beta trades off main-lobe width against
        sidelobe suppression without shifting the reported signal level.
        """
        win = np.kaiser(size, beta)
        coherent_gain = win.mean()
        if coherent_gain <= 0:
            coherent_gain = 1.0
        return (win / coherent_gain).astype(np.float32)

    # ------------------------------------------------------------------------------------------------------------------
    # Guard: settings/start may only be changed while no worker is active
    # ------------------------------------------------------------------------------------------------------------------
    def _check_idle(self, what: str) -> None:
        if self._active:
            raise ReceiverBusyError(
                f"Cannot change {what} while the receiver is running. Call stop() and wait() first."
            )

    # ------------------------------------------------------------------------------------------------------------------
    # Configuration setters -- call only while no worker is active
    # ------------------------------------------------------------------------------------------------------------------
    def set_device(
        self, driver: str, device_args: dict[str, str] | None = None, port_in_use: object | None = None
    ) -> None:
        """
        Select which SoapySDR driver/device to use, e.g. 'lime', 'rtlsdr', 'airspy'.

        port_in_use identifies which physical device/connection this receiver instance represents, matching
        QtTinySA router's own port_in_use signal argument (a FakePortInfo, QtTinySA's own helper mimicking
        pyserial's ListPortInfo -- not defined in this module). It's carried verbatim through every
        spectrum_ready emission (see _acquisition_loop()) so the router can tell which of several concurrently
        running SoapySDRReceiver instances (e.g. a LimeSDR and an RTL-SDR, or two LimeSDRs distinguished by
        serial number in device_args) a given reading came from, the same way it already distinguishes real
        tinySA units on different serial ports. QtTinySA always supplies this, so it's stored exactly as given
        -- nothing is built or substituted here if it's left as None.
        """
        self._check_idle("device")
        self._driver = driver
        self._device_args = dict(device_args or {"driver": driver})
        self._port_in_use = port_in_use

    @property
    def port_in_use(self) -> object | None:
        """The port_in_use value passed to set_device(), or None if not set."""
        return self._port_in_use

    def set_antenna(self, antenna: str) -> None:
        """
        Select the RX antenna port. Primarily relevant for LimeSDR, which exposes multiple RX antennas (e.g. 'LNAW',
        'LNAH', 'LNAL'). Silently ignored (with a logged warning) if the current device has no matching antenna option.
        """
        self._check_idle("antenna")
        self._antenna = antenna

    def set_center_frequency(self, freq_hz: float) -> None:
        """
        Set the RX center frequency. Unlike most settings, this is safe to call while the receiver is running:
        the new value is picked up and applied by the worker thread itself -- never this calling thread
        directly, since making concurrent SoapySDR device calls from two threads at once isn't guaranteed safe
        -- at the top of its next acquisition loop iteration (see _acquisition_loop()), typically within one
        FFT block's worth of time. The displayed frequency axis is recomputed at the same time, so there's no
        stale-axis window.
        """
        self._center_freq = float(freq_hz)

    def set_span(self, span_hz: float) -> None:
        """
        Set the requested frequency span (as set in the QtTinySA GUI). Because this receiver captures IQ samples,
        the usable span equals the acquisition sample rate, so this picks the smallest preset sample rate that is
        >= span_hz and uses it as both the capture rate and the displayed span.

        Raises ValueError if span_hz exceeds the largest available preset sample rate (30.72 MHz by default).
        Changing the span can change which RBWs are achievable, so call set_rbw() again afterwards (or catch the
        ValueError it raises) to confirm the current RBW is still valid at the new span.
        """
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
        """
        Set the requested resolution bandwidth (as set in the QtTinySA GUI). Computes the FFT size needed to hit
        this RBW at the *current* span/sample rate (call set_span() first), accounting for the current Kaiser
        beta, and rebuilds the FFT window at the new size.

        Raises ValueError (propagated from calculate_fft_size()) if rbw_hz is too narrow to achieve at the current
        span within the maximum FFT size -- QtTinySA should catch this and show the user a message rather than
        silently accepting an unreachable RBW.
        """
        self._check_idle("RBW")
        fft_size = self.calculate_fft_size(rbw_hz, self._sample_rate, self._kaiser_beta)
        self._rbw = float(rbw_hz)
        self._fft_size = fft_size
        self._window = self._make_window(fft_size, self._kaiser_beta)

    def set_preset_sample_rates(self, rates: list[float]) -> None:
        """Override the sample-rate presets used by set_span()."""
        self._check_idle("preset sample rates")
        self._preset_sample_rates = sorted(rates)

    def set_kaiser_beta(self, beta: float) -> None:
        """
        Change the Kaiser window beta (default 10.5) and recompute the FFT size for the currently requested RBW at
        the current span, since the main-lobe broadening -- and therefore the FFT size needed to hit a given RBW --
        depends on beta. Raises ValueError (propagated from set_rbw()) if the current RBW is no longer achievable
        at the new beta.
        """
        self._check_idle("Kaiser beta")
        old_beta = self._kaiser_beta
        self._kaiser_beta = float(beta)
        try:
            self.set_rbw(self._rbw)
        except ValueError:
            self._kaiser_beta = old_beta  # roll back so the receiver is left in a valid state
            raise

    def set_gain(self, gain_db: float | None) -> None:
        """Pass None to enable the device's automatic gain control."""
        self._check_idle("gain")
        self._gain = gain_db
        self._agc = gain_db is None

    def set_bandwidth(self, bandwidth_hz: float | None) -> None:
        self._check_idle("bandwidth")
        self._bandwidth = bandwidth_hz

    def set_dc_offset_removal(self, enabled: bool, mode: str = "software") -> None:
        """
        mode="hardware" asks the SDR itself to correct DC offset (only applied if the device reports support via
        hasDCOffsetMode). mode="software" subtracts the measured mean IQ offset from every captured block before the
        FFT; this works regardless of hardware support and is the safe default.
        """
        if mode not in ("software", "hardware"):
            raise ValueError("mode must be 'software' or 'hardware'")
        self._check_idle("DC offset removal")
        self._dc_offset_removal = enabled
        self._dc_offset_mode = mode

    def set_iq_balance_mode(self, enabled: bool) -> None:
        """
        Enable/disable automatic frontend IQ/gain-phase balance correction via the generic SoapySDR
        setIQBalanceMode() call. This corrects gain/phase mismatch between the I and Q ADC paths, which
        otherwise mirrors real signals into a spurious image on the opposite side of DC. Support varies by
        driver/hardware -- checked via hasIQBalanceMode() when the device is opened; silently ignored (with a
        logged message) on drivers/devices that don't support it.
        """
        self._check_idle("IQ balance mode")
        self._iq_balance_removal = bool(enabled)

    def set_auto_calibrate(self, enabled: bool) -> None:
        """
        Enable/disable automatically running the LimeSDR-specific CALIBRATE_RX routine (see calibrate()) every
        time the device is (re)opened. This is a LimeSDR/SoapyLMS7-specific writeSetting, not a generic SoapySDR
        call, so it has no effect on other drivers regardless of this setting.
        """
        self._check_idle("auto-calibrate")
        self._auto_calibrate = bool(enabled)

    def set_gfir_lpf_bandwidth(self, bandwidth_hz: float | None) -> None:
        """
        Set the LimeSDR-specific digital GFIR (General FIR) low-pass filter bandwidth, via the
        "ENABLE_GFIR_LPF" writeSetting. This is a separate, sharper digital filter stage in the LMS7002M's
        decimation chain, applied in addition to -- not instead of -- the analog baseband filter set by
        set_bandwidth() (whose achievable range on LimeSDR spans roughly 1.4001 MHz to 130 MHz across its
        low-band/high-band paths). Pass None (the default) to leave LimeSuite's own automatic GFIR
        reconfiguration, which it already performs whenever the sample rate changes, untouched. LimeSDR-specific;
        ignored on other drivers. Applied after the sample rate is set, as the underlying API requires.
        """
        self._check_idle("GFIR LPF bandwidth")
        self._gfir_lpf_bandwidth = bandwidth_hz

    def set_oversampling(self, ratio: int) -> None:
        """
        Set the LimeSDR-specific ADC/DAC oversampling ratio, via the "OVERSAMPLING" writeSetting. Must be one of
        LIME_VALID_OVERSAMPLING_RATIOS (0, 1, 2, 4, 8, 16, 32), where 0 lets LimeSuite pick automatically.
        LimeSDR-specific; ignored on other drivers.
        """
        if ratio not in LIME_VALID_OVERSAMPLING_RATIOS:
            raise ValueError(f"ratio must be one of {LIME_VALID_OVERSAMPLING_RATIOS}, got {ratio}")
        self._check_idle("oversampling ratio")
        self._oversampling = ratio

    def set_offset_tuning(self, offset_hz: float) -> None:
        """
        Enable RF/NCO offset tuning: tunes the RF LO offset_hz away from the requested center frequency and
        digitally re-centers in the baseband, so the displayed spectrum still reads out centered on center_freq.
        This moves LO self-mixing DC spike, 1/f noise, and other near-DC artifacts out of the displayed span
        entirely, at the cost of needing extra RF-passband/sample-rate headroom to accommodate the shift (leave
        margin between the requested span and set_span()'s chosen sample rate). Pass 0 to disable (the default).

        Two mechanisms are tried, in order, when the device is opened -- both are checked at runtime, not by
        driver name: an "OFFSET" setFrequency() tuning arg (confirmed via SoapySDRUtil --probe on LimeSuiteNG's
        SoapyLMS driver, which also compensates via its own baseband CORDIC), or, failing that, a separate "BB"
        tunable frequency component (the older SoapyLMS7 mechanism, split manually into RF and BB calls here).
        If neither is supported, the setting is silently ignored with a logged message. In practice this is
        mainly LimeSDR (both SoapyLMS7 and LimeSuiteNG's SoapyLMS), though the checks themselves are generic.
        """
        self._check_idle("offset tuning")
        self._tuning_offset_hz = float(offset_hz)

    # ------------------------------------------------------------------------------------------------------------------
    # Lifecycle: start / stop / wait
    # ------------------------------------------------------------------------------------------------------------------
    def start(self) -> None:
        """Submit the acquisition worker to this receiver's QThreadPool."""
        self._check_idle("start")
        if not self._driver:
            raise RuntimeError("No SoapySDR device selected; call set_device() first.")
        self._running = True
        self._active = True
        self._finished_event.clear()
        worker = _AcquisitionWorker(self)
        self._pool.start(worker)

    def stop(self) -> None:
        """Request the acquisition loop to stop. Call wait() afterwards to confirm it has."""
        self._running = False

    def set_amplitude_calibration(self, freqs_hz: np.ndarray, offsets_db: np.ndarray) -> None:
        """
        Install a per-frequency amplitude calibration table: every emitted spectrum's power is corrected as
        `power - interp(freq, freqs_hz, offsets_db)` before spectrum_ready fires (see _acquisition_loop()),
        linearly interpolating between points and holding the nearest endpoint's value outside the calibrated
        range. Typically populated from LimeLoopbackCalibrator.run_calibration_sweep()'s result, but any
        (freqs_hz, offsets_db) pair works.

        Unlike most setters, this is safe to call while the receiver is running (no hardware is touched, and the
        two numpy arrays are swapped in with a single attribute assignment each, so the worker thread picks up
        either the old or the new table cleanly, never a half-updated one) -- so you can load or update a saved
        calibration without needing to stop() the live stream first.
        """
        freqs_hz = np.asarray(freqs_hz, dtype=np.float64)
        offsets_db = np.asarray(offsets_db, dtype=np.float64)
        if freqs_hz.shape != offsets_db.shape or freqs_hz.ndim != 1:
            raise ValueError("freqs_hz and offsets_db must be 1-D arrays of the same length")
        order = np.argsort(freqs_hz)
        self._cal_freqs_hz = freqs_hz[order]
        self._cal_offsets_db = offsets_db[order]

    def clear_amplitude_calibration(self) -> None:
        """Remove any installed amplitude calibration table; subsequent spectra are emitted uncorrected."""
        self._cal_freqs_hz = None
        self._cal_offsets_db = None

    @property
    def device(self) -> SoapySDR.Device | None:
        """
        The underlying, already-open SoapySDR device handle, or None if the receiver hasn't been started. Exposed
        read-only so companion modules (e.g. lime_tsg.py, lime_loopback.py) can drive the TX side or issue
        LimeSDR-specific writeSetting calls against the same hardware session the receiver is already streaming
        from, without opening a second, conflicting connection.
        """
        return self._device

    def wait(self, timeout: float | None = None) -> bool:
        """
        Block until the acquisition worker has fully exited (device closed, thread returned to the pool). Returns True
        if it finished within timeout seconds, False on timeout.
        """
        return self._finished_event.wait(timeout)

    # ------------------------------------------------------------------------------------------------------------------
    # Device lifecycle helpers (worker thread only)
    # ------------------------------------------------------------------------------------------------------------------
    def _is_lime_driver(self) -> bool:
        """
        True if the selected driver is a LimeSDR variant, gating LimeSDR-specific calls. Matches both the classic
        LimeSuite driver string ("lime") and LimeSuiteNG's ("limesuiteng") via substring, since both register
        under names containing "lime".
        """
        return bool(self._driver) and "lime" in self._driver.lower()

    def _open_device(self) -> None:
        try:
            self._device = SoapySDR.Device(self._device_args)
        except Exception as exc:
            raise RuntimeError(f"Failed to open SoapySDR device: {exc}") from exc

        # Oversampling affects the internal ADC/CGEN clock config, so it's set before sample rate/frequency.
        self._apply_oversampling()
        self._apply_antenna()
        self._apply_rate_and_freq()

        # GFIR must follow the sample rate, per LimeSuite's own requirement.
        self._apply_gfir_lpf()
        self._apply_gain()
        self._apply_bandwidth()

        if self._auto_calibrate and self._is_lime_driver():
            self.calibrate()

        # Applied after calibrate(): LimeSDR's CALIBRATE_RX routine can itself re-enable hardware DC offset
        # correction as a side effect, so these run last to make sure our requested settings are the ones that
        # actually stick, regardless of what calibration toggled internally.
        self._apply_dc_offset()
        self._apply_iq_balance()

        try:
            self._rx_stream = self._device.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32)
            self._device.activateStream(self._rx_stream)
        except Exception as exc:
            raise RuntimeError(f"Failed to set up RX stream: {exc}") from exc

        self.status.emit(
            f"Opened {self._driver} device, streaming at {self._sample_rate:.0f} Sps, "
            f"FFT size {self._fft_size}, Kaiser beta {self._kaiser_beta:.2f}"
        )

    def calibrate(self) -> None:
        """
        Run the LimeSDR one-shot RF-loopback calibration, via the per-channel "CALIBRATE" writeSetting, which
        recalibrates both DC offset and IQ balance for the current frequency/gain/bandwidth in a single step.
        Confirmed via SoapySDRUtil --probe against both classic LimeSuite (SoapyLMS7) and LimeSuiteNG (SoapyLMS):
        the key is "CALIBRATE" (not "CALIBRATE_RX"/"CALIBRATE_TX") and is set per-channel as
        writeSetting(direction, channel, "CALIBRATE", bandwidth_hz), with a valid bandwidth range of
        LIME_CALIBRATE_BANDWIDTH_RANGE_HZ (2.5 MHz-120 MHz); values outside that range are clamped here, with a
        logged warning, before being sent. This is LimeSDR-specific, not a generic SoapySDR call -- it is a no-op
        (with a logged warning) on any other driver, and requires the device to already be open.
        """
        if self._device is None:
            logger.warning("calibrate() called with no open device; ignoring")
            return
        if not self._is_lime_driver():
            logger.warning(
                "calibrate() is LimeSDR-specific (CALIBRATE); ignoring for driver '%s'", self._driver
            )
            return
        requested_bandwidth = self._bandwidth or self._sample_rate
        low, high = LIME_CALIBRATE_BANDWIDTH_RANGE_HZ
        calibration_bandwidth = max(low, min(high, requested_bandwidth))
        if calibration_bandwidth != requested_bandwidth:
            logger.warning(
                "Requested CALIBRATE bandwidth %.0f Hz outside device range [%.0f, %.0f] Hz; clamped to %.0f Hz",
                requested_bandwidth, low, high, calibration_bandwidth,
            )
        try:
            self._device.writeSetting(SOAPY_SDR_RX, 0, "CALIBRATE", str(calibration_bandwidth))
            self.status.emit(f"LimeSDR RX calibration complete (bandwidth {calibration_bandwidth:.0f} Hz)")
        except Exception:
            logger.exception("LimeSDR CALIBRATE failed")

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
        if not self._antenna or self._device is None:
            return
        try:
            available = self._device.listAntennas(SOAPY_SDR_RX, 0)
            if self._antenna in available:
                self._device.setAntenna(SOAPY_SDR_RX, 0, self._antenna)
            else:
                logger.warning(
                    "Antenna '%s' not available on this device (options: %s)",
                    self._antenna, available,
                )
        except Exception:
            logger.exception("Failed to set antenna")

    def _apply_rate_and_freq(self) -> None:
        if self._device is None:
            return
        try:
            self._device.setSampleRate(SOAPY_SDR_RX, 0, self._sample_rate)
        except Exception:
            logger.exception("Failed to set sample rate")
        self._apply_frequency()

    def _apply_frequency(self) -> None:
        """
        Apply self._center_freq (and self._tuning_offset_hz, if set) to the device. Split out from
        _apply_rate_and_freq() so it can also be called on its own from _acquisition_loop() for a live
        center-frequency retune, without touching the sample rate/FFT size/stream at all.
        """
        if self._device is None:
            return
        try:
            if self._tuning_offset_hz and self._supports_offset_tuning_arg():
                # Preferred path (confirmed via SoapySDRUtil --probe on LimeSuiteNG's SoapyLMS driver): a
                # single setFrequency() call with an "OFFSET" tuning arg. The driver places the RF LO
                # offset_hz away from center_freq and compensates with its own baseband CORDIC (NCO), so the
                # streamed signal still reads out centered on center_freq.
                self._device.setFrequency(
                    SOAPY_SDR_RX, 0, self._center_freq, {"OFFSET": str(self._tuning_offset_hz)}
                )
            elif self._tuning_offset_hz and self._supports_offset_tuning_components():
                # Fallback (older/classic SoapyLMS7): manually split into separate RF and BB (NCO) components.
                self._device.setFrequency(SOAPY_SDR_RX, 0, "RF", self._center_freq + self._tuning_offset_hz)
                self._device.setFrequency(SOAPY_SDR_RX, 0, "BB", -self._tuning_offset_hz)
            else:
                if self._tuning_offset_hz:
                    logger.info(
                        "Offset tuning requested but device supports neither an OFFSET tuning arg nor a "
                        "separate BB/NCO frequency component; ignoring"
                    )
                self._device.setFrequency(SOAPY_SDR_RX, 0, self._center_freq)
        except Exception:
            logger.exception("Failed to set frequency")

    def _supports_offset_tuning_arg(self) -> bool:
        """True if the device advertises an 'OFFSET' setFrequency() tuning arg (e.g. LimeSuiteNG's SoapyLMS)."""
        if self._device is None:
            return False
        try:
            return any(arg.key == "OFFSET" for arg in self._device.getFrequencyArgsInfo(SOAPY_SDR_RX, 0))
        except Exception:
            return False

    def _supports_offset_tuning_components(self) -> bool:
        """True if the device exposes a separate 'BB' (baseband NCO) tunable frequency component."""
        if self._device is None:
            return False
        try:
            return "BB" in self._device.listFrequencies(SOAPY_SDR_RX, 0)
        except Exception:
            return False

    def _apply_oversampling(self) -> None:
        if self._device is None or not self._is_lime_driver():
            return
        try:
            self._device.writeSetting("OVERSAMPLING", str(self._oversampling))
        except Exception:
            logger.exception("Failed to set LimeSDR OVERSAMPLING")

    def _apply_gfir_lpf(self) -> None:
        if self._device is None or not self._is_lime_driver() or self._gfir_lpf_bandwidth is None:
            return
        try:
            self._device.writeSetting(SOAPY_SDR_RX, 0, "ENABLE_GFIR_LPF", str(self._gfir_lpf_bandwidth))
        except Exception:
            logger.exception("Failed to set LimeSDR GFIR LPF bandwidth")

    def _apply_gain(self) -> None:
        if self._device is None:
            return
        try:
            if hasattr(self._device, "setGainMode"):
                self._device.setGainMode(SOAPY_SDR_RX, 0, bool(self._agc))
            if not self._agc and self._gain is not None:
                self._device.setGain(SOAPY_SDR_RX, 0, self._gain)
        except Exception:
            logger.exception("Failed to set gain")

    def _apply_bandwidth(self) -> None:
        if self._device is None or self._bandwidth is None:
            return
        try:
            self._device.setBandwidth(SOAPY_SDR_RX, 0, self._bandwidth)
        except Exception:
            logger.exception("Failed to set bandwidth")

    def _apply_dc_offset(self) -> None:
        if self._device is None:
            return
        if self._dc_offset_removal and self._dc_offset_mode == "hardware":
            try:
                if self._device.hasDCOffsetMode(SOAPY_SDR_RX, 0):
                    self._device.setDCOffsetMode(SOAPY_SDR_RX, 0, True)
                else:
                    logger.info(
                        "Device has no hardware DC offset correction; falling back to software removal"
                    )
                    self._dc_offset_mode = "software"
            except Exception:
                logger.exception("Failed to set hardware DC offset mode")
        else:
            try:
                if self._device.hasDCOffsetMode(SOAPY_SDR_RX, 0):
                    self._device.setDCOffsetMode(SOAPY_SDR_RX, 0, False)
            except Exception:
                pass  # not all drivers support querying/toggling this

    def _apply_iq_balance(self) -> None:
        if self._device is None:
            return
        try:
            if not hasattr(self._device, "hasIQBalanceMode") or not self._device.hasIQBalanceMode(
                SOAPY_SDR_RX, 0
            ):
                if self._iq_balance_removal:
                    logger.info("Device has no automatic hardware IQ balance correction; skipping")
                return
            self._device.setIQBalanceMode(SOAPY_SDR_RX, 0, bool(self._iq_balance_removal))
        except Exception:
            logger.exception("Failed to set IQ balance mode")

    # ------------------------------------------------------------------------------------------------------------------
    # Main acquisition loop (runs on a QThreadPool worker thread)
    # ------------------------------------------------------------------------------------------------------------------
    def _acquisition_loop(self) -> None:
        try:
            self._open_device()
        except Exception as exc:
            self.error.emit(str(exc))
            return

        try:
            fft_size = self._fft_size
            window = self._window
            read_buf = np.zeros(fft_size, dtype=np.complex64)
            dc_removal = self._dc_offset_removal
            dc_mode = self._dc_offset_mode
            sample_rate = self._sample_rate
            center_freq = self._center_freq
            freqs = np.fft.fftshift(np.fft.fftfreq(fft_size, d=1.0 / sample_rate)) + center_freq
            freqs = freqs.astype(np.float64)

            while self._running:
                if self._center_freq != center_freq:
                    # Live retune: picked up here (worker thread only touches the device), applied without
                    # touching sample rate/FFT size/stream, and the frequency axis is refreshed to match.
                    center_freq = self._center_freq
                    self._apply_frequency()
                    freqs = np.fft.fftshift(np.fft.fftfreq(fft_size, d=1.0 / sample_rate)) + center_freq
                    freqs = freqs.astype(np.float64)

                n_collected = 0
                timed_out = False
                while n_collected < fft_size:
                    sr = self._device.readStream(
                        self._rx_stream,
                        [read_buf[n_collected:]],
                        fft_size - n_collected,
                        timeoutUs=200_000,
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
                        self.error.emit(f"SoapySDR readStream error: {sr.ret}")
                        timed_out = True
                        break

                    if not self._running:
                        break

                if not self._running:
                    break
                if timed_out or n_collected < fft_size:
                    continue

                iq = read_buf.copy()

                if dc_removal and dc_mode == "software":
                    iq = iq - np.mean(iq)

                power = self._compute_spectrum(iq, window)

                cal_freqs = self._cal_freqs_hz
                cal_offsets = self._cal_offsets_db
                if cal_freqs is not None and cal_offsets is not None:
                    correction = np.interp(freqs, cal_freqs, cal_offsets).astype(np.float32)
                    power = power - correction

                self.spectrum_ready.emit(freqs, power, time.time(), self._port_in_use)
        finally:
            self._close_device()
            self._active = False
            self._finished_event.set()
            self.status.emit("SoapySDR receiver stopped")

    # ------------------------------------------------------------------------------------------------------------------
    # Signal processing helpers
    # ------------------------------------------------------------------------------------------------------------------
    @staticmethod
    def _compute_spectrum(iq: np.ndarray, window: np.ndarray) -> np.ndarray:
        """
        Apply the coherent-gain-normalized Kaiser window, FFT, and convert to dB. Because `window` was already divided
        by its own mean in _make_window(), a full-scale tone reports the same peak power regardless of beta -- only
        sidelobe/main-lobe behaviour changes.
        """
        windowed = iq * window
        spectrum = np.fft.fftshift(np.fft.fft(windowed))
        n = len(spectrum)
        power = 20 * np.log10(np.abs(spectrum) / n + 1e-20)
        return power.astype(np.float32)


class _AcquisitionWorker(QRunnable):
    """QRunnable that hands control back to the owning receiver's acquisition loop."""

    def __init__(self, receiver: SoapySDRReceiver):
        super().__init__()
        self._receiver = receiver
        self.setAutoDelete(True)

    def run(self) -> None:
        self._receiver._acquisition_loop()
