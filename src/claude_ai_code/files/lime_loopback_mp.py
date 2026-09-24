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
lime_loopback_mp.py

Multiprocessing counterpart to lime_loopback.py, paired with soapy_receiver_mp.py and lime_tsg_mp.py.

Amplitude calibration for the SoapyReceiverProcess spectrum analyzer pipeline, using the LimeSDR TSG as a known
reference tone routed through either LimeSDR's internal RF loopback antenna paths ("LB1"/"LB2") or an external
cable connecting the TX and RX ports.

How this differs from lime_loopback.py, and why: the threading version could reach into
SoapySDRReceiver.device directly (TX antenna selection, constructing a LimeTestSignalGenerator) and listen to
its spectrum_ready Qt signal. Neither is possible here -- a SoapySDR.Device handle cannot cross a process
boundary, so it only ever exists inside soapy_receiver_mp.py's worker process. Consequently:
    - TX antenna selection and TSG enable/disable go through SoapyReceiverProcess.set_tx_antenna()/
      enable_tsg()/disable_tsg(), which send commands into the worker process rather than touching a device
      directly.
    - Tone power is read by draining SoapyReceiverProcess.calibration_queue directly with plain blocking
      queue.get() calls -- no Qt event loop or QCoreApplication.processEvents() pumping needed at all, since
      multiprocessing.Queue doesn't have the queued-cross-thread-signal-delivery issue lime_loopback.py had to
      work around. This is genuinely simpler code as a result.
    - calibration_queue is a channel separate from data_queue specifically so this doesn't race QtTinySA's own
      dedicated thread draining data_queue for normal display -- see soapy_receiver_mp.py's module docstring.

The resulting offsets are applied as a pure software correction, pushed into the worker process via
SoapyReceiverProcess.set_amplitude_calibration() once the sweep finishes -- every subsequent item on data_queue
is corrected automatically from then on, exactly as in the threading version. See lime_loopback.py's module
docstring for the fuller rationale on why this is preferred over adjusting PGA gain per frequency.

Requires the receiver to already be constructed with a LimeSDR device selected (receiver.set_device(...)).
"""

import logging
import queue
import time
from dataclasses import dataclass

import numpy as np

from lime_tsg_mp import DEFAULT_TSG_LEVEL_DBFS, TSG_NCO_DIVISORS, LimeTestSignalGenerator

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
    Drives a SoapyReceiverProcess through a loopback amplitude-calibration sweep against a table of externally
    measured TSG reference powers.

    Typical usage:

        reference_table = {
            433.9e6: -20.0,
            868.0e6: -19.5,
            2400.0e6: -18.7,
        }

        cal = LimeLoopbackCalibrator(receiver, reference_table, loopback="internal")
        offsets_db = cal.run_calibration_sweep(sample_rate=2e6, span=2e6, rbw=10e3, tsg_divisor=4)
        # offsets_db is already installed on the receiver at this point -- every data_queue item from here on
        # is corrected automatically. No further action needed for live display.

        # Resume normal use:
        cal.restore_normal_antenna("LNAW")
        receiver.start()
    """

    def __init__(
        self,
        receiver: "SoapyReceiverProcess",
        reference_table: dict[float, float],
        loopback: str = "internal",
        rx_loopback_antenna: str = "LB1",
        tx_loopback_antenna: str = "Band1",
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
        restarts the receiver's worker process as needed for each point; leaves it stopped (with the loopback
        antenna still selected) when finished -- call restore_normal_antenna() and receiver.start() afterwards
        to resume normal use.

        Returns {frequency_hz: offset_db}, where corrected_dbm = raw_measured_power_db - offset_db. Also
        available afterwards via the offsets_db property.
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
            # Push the table into the worker process so every subsequent data_queue item is corrected
            # automatically -- this is the actual point where calibration takes effect for live display.
            self._receiver.set_amplitude_calibration(self._cal_freqs_hz, self._cal_offsets_db)
        else:
            logger.warning("Calibration sweep produced no usable points; receiver's amplitude table left as-is")

        return dict(self._offsets_db)

    def _drain_events(self) -> None:
        """Log anything currently waiting on event_queue (status/error messages), non-blocking, best-effort."""
        while True:
            try:
                kind, message = self._receiver.event_queue.get_nowait()
            except queue.Empty:
                return
            except Exception:
                return
            if kind == "error":
                logger.error("Worker process: %s", message)
            else:
                logger.info("Worker process: %s", message)

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

        # No explicit "wait for device ready" step is needed here: set_tx_antenna()/enable_tsg() just enqueue
        # commands, which sit harmlessly until the worker's main loop starts draining them (after it opens the
        # device). If the device fails to open entirely, _average_tone_power()'s own timeout below bounds how
        # long this waits before giving up.
        self._receiver.set_tx_antenna(self._tx_loopback_antenna)
        self._receiver.enable_tsg(divisor=tsg_divisor, level_dbfs=tsg_level_dbfs)
        expected_offset_hz = LimeTestSignalGenerator.expected_tone_offset_hz(sample_rate, tsg_divisor)

        time.sleep(settle_time_s)  # let the tone and gain chain settle before measuring
        measured_db = self._average_tone_power(expected_offset_hz, captures_per_point, capture_timeout_s)

        self._receiver.disable_tsg()
        self._receiver.stop()
        self._receiver.wait()

        if measured_db is None:
            self._drain_events()
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
        Collect `captures` items from the receiver's calibration_queue and return the average peak power found
        outside a small DC guard band. A search (rather than trusting a fixed sideband sign) is used
        deliberately -- see LimeTestSignalGenerator's module docstring -- since during a clean loopback capture
        the injected tone should be the only strong feature away from center anyway.
        """
        collected: list[float] = []
        dc_guard_hz = max(expected_offset_hz * 0.1, 5e3)
        deadline = time.monotonic() + timeout_s

        while len(collected) < captures and time.monotonic() < deadline:
            remaining = max(0.05, deadline - time.monotonic())
            try:
                freqs, power, _timestamp, _port_in_use = self._receiver.calibration_queue.get(timeout=remaining)
            except queue.Empty:
                break
            center = freqs[len(freqs) // 2]
            valid = np.abs(freqs - center) > dc_guard_hz
            if not np.any(valid):
                continue
            collected.append(float(np.max(power[valid])))

        if not collected:
            return None
        return float(np.mean(collected))

    # ------------------------------------------------------------------
    def correct_dbm(self, freq_hz: float, raw_power_db: float) -> float:
        """
        Manually apply the calibration offset at freq_hz to a single raw power value, with the same linear
        interpolation/endpoint-holding as the receiver's own automatic correction.

        You normally don't need to call this: run_calibration_sweep() already pushes the finished table into
        the receiver via set_amplitude_calibration(), so every data_queue item after that is corrected
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
