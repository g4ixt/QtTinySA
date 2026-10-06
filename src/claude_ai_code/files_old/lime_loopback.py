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
lime_loopback.py

Amplitude calibration for the SoapySDRReceiver spectrum analyzer pipeline, using the LimeSDR TSG (see
lime_tsg.py) as a known reference tone routed through either LimeSDR's internal RF loopback antenna paths
("LB1"/"LB2") or an external cable connecting the TX and RX ports.

Design: rather than opening a second, competing RX stream, this module works entirely through the receiver's
own already-running acquisition pipeline. It listens for spectrum_ready emissions -- the same windowed,
DC-corrected, Kaiser-normalized power spectrum used for live display -- locates the injected tone's peak, and
compares its level against a pre-measured reference power (from an external power meter, hardcoded per
frequency by the caller) to build a per-frequency calibration offset table. Because calibration is measured
through the exact same pipeline used for display, any internal FFT/window scaling constant cancels out; only
the actual hardware gain uncertainty is being corrected for.

The resulting offsets are applied as a pure software correction rather than by adjusting PGA gain per
frequency -- this keeps calibration independent of whatever gain the user/app chooses for sensitivity/dynamic-
range reasons, and gives sub-dB correction resolution instead of the PGA's 1 dB steps. run_calibration_sweep()
pushes the finished table into the receiver via SoapySDRReceiver.set_amplitude_calibration(), so every
spectrum_ready emission from then on is corrected automatically -- no per-point call from the caller is needed.
The offset is only strictly valid at the gain setting used during calibration; if you change overall gain
afterwards for live use, either keep it fixed at the calibration gain, or track and add a
(current_gain_db - calibration_gain_db) delta yourself, which this module does not currently do.

Requires the receiver to already be constructed with a LimeSDR device selected (receiver.set_device(...)).
"""

import logging
import time
from dataclasses import dataclass

import numpy as np
from PySide6.QtCore import QCoreApplication

try:
    from SoapySDR import SOAPY_SDR_TX
except ImportError as exc:  # pragma: no cover
    raise ImportError(
        "SoapySDR Python bindings not found. Install SoapySDR with Python support."
    ) from exc

from lime_tsg import DEFAULT_TSG_LEVEL_DBFS, TSG_NCO_DIVISORS, LimeTestSignalGenerator

logger = logging.getLogger(__name__)


class LimeLoopbackError(RuntimeError):
    """Raised for loopback/calibration configuration or measurement errors."""


@dataclass
class CalibrationPoint:
    frequency_hz: float
    reference_dbm: float                  # measured externally with a power meter; supplied by the caller
    measured_db: float | None = None      # this pipeline's raw reading for the tone
    offset_db: float | None = None        # measured_db - reference_dbm; corrected_dbm = raw_db - offset_db


class LimeLoopbackCalibrator:
    """
    Drives a SoapySDRReceiver through a loopback amplitude-calibration sweep against a table of externally
    measured TSG reference powers.

    Typical usage:

        # Measured externally with a power meter at the TSG's output/reference plane, at whatever TSG level/
        # divisor you plan to calibrate with:
        reference_table = {
            433.9e6: -20.0,
            868.0e6: -19.5,
            2400.0e6: -18.7,
        }

        cal = LimeLoopbackCalibrator(receiver, reference_table, loopback="internal")
        offsets_db = cal.run_calibration_sweep(sample_rate=2e6, span=2e6, rbw=10e3, tsg_divisor=4)
        # offsets_db is already installed on the receiver at this point -- every spectrum_ready emission from
        # here on is corrected automatically. No further action needed for live display.

        # Resume normal use:
        cal.restore_normal_antenna("LNAW")
        receiver.start()
    """

    def __init__(
        self,
        receiver: "SoapySDRReceiver",
        reference_table: dict[float, float],
        loopback: str = "internal",
        rx_loopback_antenna: str = "LB1",
        tx_loopback_antenna: str = "Band1",
        tx_channel: int = 0,
    ):
        """
        loopback="internal" uses LimeSDR's onboard RF loopback paths (rx_loopback_antenna/tx_loopback_antenna,
        default "LB1"/"Band1"); which LB path pairs with which TX Band output is board/revision-specific --
        verify against LimeSuiteGUI or your board's documentation before trusting the defaults.
        loopback="external" assumes you've physically cabled TX to RX yourself; pass whichever normal antenna
        names match your cabling as rx_loopback_antenna/tx_loopback_antenna instead of the LB/Band defaults.
        """
        if loopback not in ("internal", "external"):
            raise ValueError("loopback must be 'internal' or 'external'")
        self._receiver = receiver
        self._reference_table = dict(reference_table)
        self._loopback = loopback
        self._rx_loopback_antenna = rx_loopback_antenna
        self._tx_loopback_antenna = tx_loopback_antenna
        self._tx_channel = tx_channel
        self._offsets_db: dict[float, float] = {}
        self._cal_freqs_hz: np.ndarray = np.array([])
        self._cal_offsets_db: np.ndarray = np.array([])

    # ------------------------------------------------------------------
    def run_calibration_sweep(
        self,
        sample_rate: float,
        span: float,
        rbw: float,
        tsg_divisor: int = 4,
        tsg_level_dbfs: float = DEFAULT_TSG_LEVEL_DBFS,
        captures_per_point: int = 5,
        settle_time_s: float = 0.05,
        capture_timeout_s: float = 3.0,
    ) -> dict[float, float]:
        """
        Run the full loopback calibration sweep across every frequency in the reference table. Stops and
        restarts the receiver as needed for each point; leaves it stopped (with the loopback antenna still
        selected) when finished -- call restore_normal_antenna() and receiver.start() afterwards to resume
        normal use.

        Returns {frequency_hz: offset_db}, where corrected_dbm = raw_measured_power_db - offset_db. Also
        available afterwards via correct_dbm() and the offsets_db property.
        """
        if tsg_divisor not in TSG_NCO_DIVISORS:
            raise LimeLoopbackError(f"tsg_divisor must be one of {TSG_NCO_DIVISORS}, got {tsg_divisor}")
        if not self._reference_table:
            raise LimeLoopbackError("reference_table is empty; nothing to calibrate against")

        self._offsets_db = {}
        for freq_hz, reference_dbm in sorted(self._reference_table.items()):
            offset_db = self._calibrate_one_point(
                freq_hz, reference_dbm, sample_rate, span, rbw, tsg_divisor, tsg_level_dbfs,
                captures_per_point, settle_time_s, capture_timeout_s,
            )
            if offset_db is not None:
                self._offsets_db[freq_hz] = offset_db

        if self._offsets_db:
            ordered = sorted(self._offsets_db.items())
            self._cal_freqs_hz = np.array([f for f, _ in ordered], dtype=np.float64)
            self._cal_offsets_db = np.array([o for _, o in ordered], dtype=np.float64)
            # Push the table into the receiver so every subsequent spectrum_ready emission is corrected
            # automatically -- this is the actual point where calibration takes effect for live display.
            self._receiver.set_amplitude_calibration(self._cal_freqs_hz, self._cal_offsets_db)
        else:
            logger.warning("Calibration sweep produced no usable points; receiver's amplitude table left as-is")

        return dict(self._offsets_db)

    def _wait_for_device(self, timeout_s: float) -> object | None:
        """
        start() submits the acquisition worker to a QThreadPool and returns immediately -- the device isn't
        open yet until that worker thread actually runs _open_device(). Poll receiver.device (pumping the Qt
        event loop, same reasoning as _average_tone_power()) until it's available or timeout_s elapses.
        """
        app = QCoreApplication.instance()
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            device = self._receiver.device
            if device is not None:
                return device
            if app is not None:
                app.processEvents()
            time.sleep(0.01)
        return None

    def _calibrate_one_point(
        self, freq_hz, reference_dbm, sample_rate, span, rbw, tsg_divisor, tsg_level_dbfs,
        captures_per_point, settle_time_s, capture_timeout_s,
    ) -> float | None:
        self._receiver.stop()
        self._receiver.wait()
        self._receiver.set_antenna(self._rx_loopback_antenna)
        self._receiver.set_center_frequency(freq_hz)
        self._receiver.set_span(span)
        self._receiver.set_rbw(rbw)
        self._receiver.start()

        device = self._wait_for_device(timeout_s=capture_timeout_s)
        if device is None:
            logger.error(
                "Receiver did not open its device within %.1fs; aborting calibration point %.0f Hz",
                capture_timeout_s, freq_hz,
            )
            self._receiver.stop()
            self._receiver.wait()
            return None

        device.setAntenna(SOAPY_SDR_TX, self._tx_channel, self._tx_loopback_antenna)
        tsg = LimeTestSignalGenerator(device, direction=SOAPY_SDR_TX, channel=self._tx_channel)
        tsg.enable(divisor=tsg_divisor, level_dbfs=tsg_level_dbfs)
        expected_offset_hz = tsg.expected_tone_offset_hz(sample_rate, tsg_divisor)

        time.sleep(settle_time_s)  # let the tone and gain chain settle before measuring
        measured_db = self._average_tone_power(expected_offset_hz, captures_per_point, capture_timeout_s)

        tsg.disable()
        self._receiver.stop()
        self._receiver.wait()

        if measured_db is None:
            logger.error("Failed to capture TSG tone at %.0f Hz; skipping calibration point", freq_hz)
            return None

        offset_db = measured_db - reference_dbm
        logger.info(
            "Calibration point %.0f Hz: measured %.2f dB, reference %.2f dBm, offset %.2f dB",
            freq_hz, measured_db, reference_dbm, offset_db,
        )
        return offset_db

    def _average_tone_power(
        self, expected_offset_hz: float, captures: int, timeout_s: float
    ) -> float | None:
        """
        Collect `captures` spectra from the receiver's normal spectrum_ready signal and return the average peak
        power found outside a small DC guard band. A search (rather than trusting a fixed sideband sign) is used
        deliberately -- see LimeTestSignalGenerator's module docstring -- since during a clean loopback capture
        the injected tone should be the only strong feature away from center anyway.

        The receiver emits spectrum_ready from its own QThreadPool worker thread, so delivery to the plain
        Python callback here is a queued Qt connection that only gets dispatched while this thread's Qt event
        loop runs. If this method is called from the GUI thread (the normal case -- e.g. a "Calibrate" button
        handler), it pumps QCoreApplication.processEvents() while waiting so those queued signals actually get
        delivered; note this makes the GUI unresponsive for the duration of each capture. If you need a
        non-blocking sweep, run run_calibration_sweep() on a worker thread that itself runs a Qt event loop
        (e.g. QEventLoop), rather than a plain threading.Thread.
        """
        collected: list[float] = []
        dc_guard_hz = max(expected_offset_hz * 0.1, 5e3)

        def on_spectrum(freqs: np.ndarray, power: np.ndarray, _timestamp: float) -> None:
            center = freqs[len(freqs) // 2]
            valid = np.abs(freqs - center) > dc_guard_hz
            if not np.any(valid):
                return
            collected.append(float(np.max(power[valid])))

        self._receiver.spectrum_ready.connect(on_spectrum)
        try:
            app = QCoreApplication.instance()
            deadline = time.monotonic() + timeout_s
            while len(collected) < captures and time.monotonic() < deadline:
                if app is not None:
                    app.processEvents()
                time.sleep(0.01)
        finally:
            self._receiver.spectrum_ready.disconnect(on_spectrum)

        if not collected:
            return None
        return float(np.mean(collected))

    # ------------------------------------------------------------------
    def correct_dbm(self, freq_hz: float, raw_power_db: float) -> float:
        """
        Manually apply the calibration offset at freq_hz to a single raw power value, with the same linear
        interpolation/endpoint-holding as the receiver's own automatic correction.

        You normally don't need to call this: run_calibration_sweep() already pushes the finished table into
        the receiver via set_amplitude_calibration(), so every spectrum_ready emission after that is corrected
        automatically. This method exists for cases outside that live pipeline -- e.g. correcting previously
        recorded raw values, or spot-checking the table -- not as the primary way calibration gets applied.
        """
        if self._cal_freqs_hz.size == 0:
            return raw_power_db
        offset_db = float(np.interp(freq_hz, self._cal_freqs_hz, self._cal_offsets_db))
        return raw_power_db - offset_db

    @property
    def offsets_db(self) -> dict[float, float]:
        return dict(self._offsets_db)

    def restore_normal_antenna(self, rx_antenna: str) -> None:
        """Convenience: set the RX antenna back to a normal (non-loopback) port after calibration. Receiver
        must be stopped; call receiver.start() yourself afterwards."""
        self._receiver.set_antenna(rx_antenna)
