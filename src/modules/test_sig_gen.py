#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Oct 31 11:33:52 2026

@author: ian
"""
import logging
import time
import SoapySDR
from SoapySDR import SOAPY_SDR_TX, SOAPY_SDR_RX, SOAPY_SDR_CF32

# Configure logging format
logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - [%(name)s] %(message)s")
logger = logging.getLogger("SignalGenerator")

class SignalGeneratorError(RuntimeError):
    """Raised for configuration errors or hardware limits violations."""

class SignalGenerator:
    # Strict hardware limits definitions
    HARDWARE_LIMITS = {
        "limesuiteng": {
            "freq_range_hz": (100.0e3, 3.8e9),   # 100 kHz to 3.8 GHz
            "gain_range_db": (-12.0, 61.0),      # Standard TX PAD gain limits
            "sample_rate_hz": (100.0e3, 61.44e6)  # LMS7002M max clock boundaries
        },
        "hackrf": {
            "freq_range_hz": (1.0e6, 6.0e9),     # 1 MHz to 6 GHz
            "gain_range_db": (0.0, 47.0),        # Combined TX VGA (0-47 dB)
            "sample_rate_hz": (2.0e6, 20.0e6)    # Safe stable limits for HackRF
        }
    }

    def __init__(self, driver: str = "limesuiteng", channel: int = 0):
        """
        Initialises a standalone RF Signal Generator wrapper with strict bounds checking.
        Supported drivers: 'lime', 'hackrf'
        """
        self.driver = driver.lower()
        if self.driver not in self.HARDWARE_LIMITS:
            raise SignalGeneratorError(f"Unsupported driver '{driver}'. Supported: {list(self.HARDWARE_LIMITS.keys())}")
            
        if not isinstance(channel, int) or channel < 0:
            raise SignalGeneratorError(f"Invalid channel index: {channel}. Must be a non-negative integer.")

        self.channel = channel
        self.tx_stream = None
        self._limits = self.HARDWARE_LIMITS[self.driver]
        
        logger.info("Connecting to hardware device using driver: '%s'...", self.driver)
        try:
            self.sdr = SoapySDR.Device(dict(driver=self.driver))
        except Exception as e:
            raise SignalGeneratorError(f"Failed to connect to SDR using driver '{self.driver}': {e}") from e
        
    def configure(self, target_freq: float = 437.0e6, sample_rate: float = 10.0e6, 
                  gain: float = 30.0, offset_hz: float = 2.0e6, loopback: bool = False):
        """Sets up the radio parameters after validating all physical device bounds."""
        
        # 1. Validate Sample Rate
        sr_min, sr_max = self._limits["sample_rate_hz"]
        if not (sr_min <= sample_rate <= sr_max):
            raise SignalGeneratorError(f"Sample rate {sample_rate/1e6} MHz out of hardware bounds [{sr_min/1e6}, {sr_max/1e6}] MHz for {self.driver}")
            
        # 2. Validate Frequency & Offset Boundaries
        f_min, f_max = self._limits["freq_range_hz"]
        if not (f_min <= target_freq <= f_max):
            raise SignalGeneratorError(f"Target frequency {target_freq/1e6} MHz out of hardware bounds [{f_min/1e6}, {f_max/1e6}] MHz for {self.driver}")

        if self.driver == "lime":
            # Ensure baseband-shifted frequency calculation stays within synthesizer reach
            shifted_rf_freq = target_freq - offset_hz
            if not (f_min <= shifted_rf_freq <= f_max):
                raise SignalGeneratorError(f"Baseband shift results in an invalid RF frequency: {shifted_rf_freq/1e6} MHz")

        # 3. Validate Gain
        g_min, g_max = self._limits["gain_range_db"]
        if not (g_min <= gain <= g_max):
            raise SignalGeneratorError(f"Gain {gain} dB out of hardware bounds [{g_min}, {g_max}] dB for {self.driver}")

        # Guard against loopback saturation
        if loopback and gain > 35.0:
            logger.warning("High TX Gain (%.1f dB) requested during Loopback! Clamping to 30.0 dB to prevent RX saturation.", gain)
            gain = 30.0

        # Apply settings down to the hardware abstraction layer
        self.target_freq = target_freq
        self.sample_rate = sample_rate
        self.gain = gain
        self.offset_hz = offset_hz
        
        logger.info("Configuring SDR parameters (Rate: %.2f MHz, Gain: %.1f dB)", sample_rate / 1e6, gain)
        self.sdr.setSampleRate(SOAPY_SDR_TX, self.channel, self.sample_rate)
        self.sdr.setGain(SOAPY_SDR_TX, self.channel, self.gain)
        
        self._apply_loopback_configuration(loopback)
        self._apply_frequency_tuning(self.target_freq)

    def _apply_loopback_configuration(self, enable_loopback: bool):
        """Internal helper to manage hardware-specific loopback configurations safely."""
        if self.driver == "lime":
            if enable_loopback:
                logger.info("LimeSDR: Routing internal RF loopback (TXPAD -> RX LNA)...")
                self.sdr.writeSetting(SOAPY_SDR_TX, self.channel, "LOOPBACK", "RF_LOOPBACK_TXPAD")
                self.sdr.setSampleRate(SOAPY_SDR_RX, self.channel, self.sample_rate)
            else:
                self.sdr.writeSetting(SOAPY_SDR_TX, self.channel, "LOOPBACK", "NONE")
        elif self.driver == "hackrf" and enable_loopback:
            logger.warning("HackRF One lacks internal RF loopback switches. Use an external cable + attenuator (>=20dB).")

    def _apply_frequency_tuning(self, freq: float):
        """Internal helper to apply validated frequency tuning strategies."""
        if self.driver == "lime":
            logger.info("LimeSDR Custom Tuning: Tuning RF Synth to %.2f MHz | NCO to %.2f MHz", 
                        (freq - self.offset_hz) / 1e6, self.offset_hz / 1e6)
            self.sdr.setFrequency(SOAPY_SDR_TX, self.channel, "RF", freq - self.offset_hz)
            self.sdr.setFrequency(SOAPY_SDR_TX, self.channel, "BB", self.offset_hz)
        else:
            logger.info("HackRF Direct Tuning: Tuning RF Synth directly to %.2f MHz", freq / 1e6)
            self.sdr.setFrequency(SOAPY_SDR_TX, self.channel, freq)

    def start(self):
        """Activates the Tx hardware streaming pipeline."""
        if self.tx_stream is not None:
            logger.warning("Start called, but TX stream is already active.")
            return
            
        logger.info("Activating hardware TX pipeline...")
        self.tx_stream = self.sdr.setupStream(SOAPY_SDR_TX, SOAPY_SDR_CF32, [self.channel])
        self.sdr.activateStream(self.tx_stream)
        logger.info("Standalone carrier active.")

    def set_frequency(self, new_freq: float):
        """Changes the target frequency dynamically on-the-fly with bound checking."""
        f_min, f_max = self._limits["freq_range_hz"]
        if not (f_min <= new_freq <= f_max):
            logger.error("Rejected runtime frequency change: %.2f MHz is out of limits.", new_freq / 1e6)
            return

        self.target_freq = new_freq
        self._apply_frequency_tuning(new_freq)

    def set_gain(self, new_gain: float):
        """Changes the TX gain dynamically on-the-fly with bound checking."""
        g_min, g_max = self._limits["gain_range_db"]
        if not (g_min <= new_gain <= g_max):
            logger.error("Rejected runtime gain change: %.1f dB is out of limits.", new_gain)
            return

        self.gain = new_gain
        self.sdr.setGain(SOAPY_SDR_TX, self.channel, new_gain)
        logger.info("TX Gain dynamically updated to: %.1f dB", new_gain)

    def stop(self):
        """Gracefully tears down the stream and resets hardware pathways."""
        if self.tx_stream is not None:
            logger.info("Deactivating and flushing stream hardware...")
            self.sdr.deactivateStream(self.tx_stream)
            self.sdr.closeStream(self.tx_stream)
            self.tx_stream = None
            
            if self.driver == "lime":
                self.sdr.writeSetting(SOAPY_SDR_TX, self.channel, "LOOPBACK", "NONE")
            logger.info("Transmission terminated cleanly.")


# ==========================================
# Verified Verification Sandbox
# ==========================================
if __name__ == "__main__":
    sig_gen = SignalGenerator(driver="limesuiteng")
    
    try:
        # Example 1: Valid initial configuration
        sig_gen.configure(target_freq=434.0e6, sample_rate=5.0e6, gain=30.0, loopback=True)
        sig_gen.start()
        time.sleep(2)
        
        # Example 2: Dynamic runtime validation
        sig_gen.set_frequency(434.5e6)  # Allowed
        time.sleep(1)
        sig_gen.set_frequency(10.0e9)   # Logged as error and rejected safely!
        time.sleep(1)
        
    except SignalGeneratorError as err:
        logger.critical("Configuration Failed: %s", err)
    finally:
        sig_gen.stop()
