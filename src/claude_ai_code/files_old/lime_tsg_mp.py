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
lime_tsg_mp.py

Multiprocessing-variant copy of lime_tsg.py, named separately to pair with soapy_receiver_mp.py. The
LimeTestSignalGenerator class itself is unchanged from lime_tsg.py -- it's already just a stateless wrapper
around an already-open SoapySDR device handle, with no threading or process assumptions baked in, so there was
nothing to adapt. It's duplicated under this name (rather than imported from lime_tsg.py) so the multiprocessing
variant of this project is fully self-contained, and because soapy_receiver_mp.py's worker process imports it
by this name (see _ReceiverWorker._enable_tsg()/_disable_tsg()).

Controls the LimeSDR/LMS7002M's built-in Test Signal Generator (TSG), which injects a fixed-offset CW tone
directly into the transceiver's digital signal chain, via the "TSG_NCO" and "TSP_CONST" writeSettings.

LimeSDR-specific (confirmed via SoapySDRUtil --probe on both classic LimeSuite/SoapyLMS7 and LimeSuiteNG/SoapyLMS,
both listing these as per-channel "Other Settings"):
    TSG_NCO   - selects/enables an NCO-generated tone: -1 disables it; 4 places it at approximately
                +/- sample_rate/4 from the tuned center frequency; 8 places it at approximately +/- sample_rate/8.
                Which sideband the tone actually lands on is not documented at the generic SoapySDR level and may
                be chip/register-revision dependent -- see the note in enable() and lime_loopback_mp.py, which
                locates the tone by searching rather than assuming a fixed sign.
    TSP_CONST - digital signal amplitude in the TSP chain, as a signed 16-bit full-scale integer (0-32767).

Requires an already-open SoapySDR device handle -- this module does not open or manage its own device
connection. In the multiprocessing architecture, that handle only ever exists inside soapy_receiver_mp.py's
worker process; this class is used from there, never directly from the parent process.
"""

import logging

logger = logging.getLogger(__name__)

# TSG_NCO options: -1 disables the tone; 4 or 8 place a tone at roughly +/- sample_rate/4 or /8.
TSG_NCO_DISABLED = -1
TSG_NCO_DIVISORS: tuple[int, ...] = (4, 8)

# Digital full-scale for TSP_CONST (signed 16-bit).
TSP_CONST_FULL_SCALE = 32767

# Recommended linear operating range for the TSG tone level, in dBFS relative to full scale. Levels near 0 dBFS
# risk clipping/compression in the DAC's most nonlinear region; levels much below -20 dBFS start to lose SNR for
# a calibration measurement. This is a conservative default range -- tighten it if your board's DAC compression
# behaviour calls for it.
TSG_LINEAR_RANGE_DBFS: tuple[float, float] = (-20.0, -3.0)
DEFAULT_TSG_LEVEL_DBFS = -6.0


class LimeTsgError(RuntimeError):
    """Raised for TSG configuration errors (invalid divisor/level, or a failed writeSetting call)."""


class LimeTestSignalGenerator:
    """
    Controls the LimeSDR TSG on one direction/channel of an already-open SoapySDR device.

    Typical usage (TX-side tone, looped back into RX for calibration -- see lime_loopback_mp.py):

        from SoapySDR import SOAPY_SDR_TX
        tsg = LimeTestSignalGenerator(device, direction=SOAPY_SDR_TX, channel=0)
        tsg.enable(divisor=4, level_dbfs=-6.0)
        ...
        tsg.disable()
    """

    def __init__(self, device, direction: int, channel: int = 0):
        if device is None:
            raise LimeTsgError("device must be an already-open SoapySDR device, not None")
        self._device = device
        self._direction = direction
        self._channel = channel

    def enable(self, divisor: int = 4, level_dbfs: float = DEFAULT_TSG_LEVEL_DBFS) -> None:
        """
        Turn on the CW test tone. `divisor` selects the NCO tone spacing (nominally sample_rate/divisor from the
        tuned center frequency, sign unconfirmed -- see module docstring); must be 4 or 8. `level_dbfs` sets the
        digital signal level, clamped to TSG_LINEAR_RANGE_DBFS with a logged warning if out of range.
        """
        if divisor not in TSG_NCO_DIVISORS:
            raise LimeTsgError(f"divisor must be one of {TSG_NCO_DIVISORS}, got {divisor}")

        low, high = TSG_LINEAR_RANGE_DBFS
        clamped_level = max(low, min(high, level_dbfs))
        if clamped_level != level_dbfs:
            logger.warning(
                "TSG level %.1f dBFS outside recommended linear range [%.1f, %.1f] dBFS; clamped to %.1f dBFS",
                level_dbfs, low, high, clamped_level,
            )
        raw_level = round(TSP_CONST_FULL_SCALE * (10 ** (clamped_level / 20.0)))
        raw_level = max(0, min(TSP_CONST_FULL_SCALE, raw_level))

        try:
            self._device.writeSetting(self._direction, self._channel, "TSP_CONST", str(raw_level))
            self._device.writeSetting(self._direction, self._channel, "TSG_NCO", str(divisor))
        except Exception as exc:
            raise LimeTsgError(f"Failed to enable TSG: {exc}") from exc

    def disable(self) -> None:
        """Turn off the CW test tone."""
        try:
            self._device.writeSetting(self._direction, self._channel, "TSG_NCO", str(TSG_NCO_DISABLED))
        except Exception as exc:
            raise LimeTsgError(f"Failed to disable TSG: {exc}") from exc

    @staticmethod
    def expected_tone_offset_hz(sample_rate_hz: float, divisor: int) -> float:
        """
        Nominal magnitude of the tone's offset from the tuned center frequency. The actual sideband sign is not
        assumed here -- callers (see lime_loopback_mp.py) should search both +/- this offset in a captured
        spectrum rather than trusting one sign.
        """
        if divisor not in TSG_NCO_DIVISORS:
            raise LimeTsgError(f"divisor must be one of {TSG_NCO_DIVISORS}, got {divisor}")
        return sample_rate_hz / divisor
