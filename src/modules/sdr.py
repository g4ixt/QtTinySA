#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Sep 18 15:33:12 2026

@author: mainly google ai
"""

import numpy as np
import SoapySDR
import logging
from SoapySDR import *
from PySide6.QtCore import QThread, Signal

class SDRSource(QThread):
    trace_ready = Signal(np.ndarray)

    def __init__(self, fft_size=1024):
        super().__init__()
        self.fft_size = fft_size
        self.running = True
        
        # Pre-calculate the window coefficients once at startup
        # Blackman-Harris offers exceptional sidelobe suppression (~92 dB drop)
        self.window = np.blackman_harris(self.fft_size)
        
        # Normalize the window so that it doesn't artificially attenuate the signal power
        self.window_gain_correction = 1.0 / np.sum(self.window)

    def run(self):
        # Initialize hardware directly using official bindings
        args = dict(driver="lime") # or rtlsdr, hackrf, sdrplay, etc.
        sdr = SoapySDR.Device(args)
        
        # Target center frequency requested by your QtTinySA GUI selection
        target_frequency = 433.92e6  # 433.92 MHz (Example)
        
        # --- AUTO ANTENNA ASSIGNMENT LOOP ---
        if target_frequency < 700e6:
            active_port = "LNAL"      # Below 700 MHz, use Low path
        elif target_frequency > 2000e6:
            active_port = "LNAH"      # Above 2.0 GHz, use High path
        else:
            active_port = "LNAW"      # Between 700 MHz and 2.0 GHz, use Wideband
        
        # Apply to RX Direction, Channel Index 0 (RX1_A or RX1_B port clusters)
        sdr.setAntenna(SOAPY_SDR_RX, 0, active_port)
        logging.info(f'LimeSDR configured. Antenna set to: {sdr.getAntenna(SOAPY_SDR_RX, 0)}')
        
        # Setup settings
        sdr.setDCOffsetMode(SOAPY_SDR_RX, 0, True) # auto hardware DC offset tracking (null DC spike)
        sdr.setSampleRate(SOAPY_SDR_RX, 0, 2.048e6)
        sdr.setFrequency(SOAPY_SDR_RX, 0, 433.92e6)
        
        # Setup active streaming buffer channels
        rx_stream = sdr.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32)
        sdr.activateStream(rx_stream)

        # Allocate space for incoming complex I/Q data
        iq_buffer = np.zeros(self.fft_size, dtype=np.complex64)

        while self.running:
            # Read streaming I/Q blocks directly out of the SDR hardware layer
            sr = sdr.readStream(rx_stream, [iq_buffer], self.fft_size)
            if sr.ret > 0:
                
                # Subtract the mean of the complex samples to null any remaining DC spike
                zero_mean_iq = iq_buffer - np.mean(iq_buffer)
                
                # Apply the window filter element-wise across the array
                # This smooths out the block edges to eliminate spectral leakage
                windowed_iq = zero_mean_iq * self.window
                # windowed_iq = iq_buffer * self.window
                
                # Compute the FFT
                fft_output = np.fft.fft(windowed_iq)
                fft_shifted = np.fft.fftshift(fft_output)
                
                # Apply normalization factor to keep absolute amplitude accurate
                fft_normalized = fft_shifted * self.window_gain_correction
                
                # Convert to dB Power Scale
                magnitude_db = 20 * np.log10(np.abs(fft_normalized) + 1e-10)
                
                # Emit clean spectrum payload straight to QtTinySA UI plot queue
                self.trace_ready.emit(magnitude_db)

        # Teardown connection safely
        sdr.deactivateStream(rx_stream)
        sdr.closeStream(rx_stream)

    def stop(self):
        self.running = False
        self.wait()

    def calculate_optimal_fft_size(sample_rate, target_rbw, window_type="blackman_harris"):
        # 1. Map the Window Factor (NENBW)
        window_factors = {
            "rectangular": 1.00,
            "hamming": 1.36,
            "hann": 1.50,
            "blackman_harris": 2.00
        }
        factor = window_factors.get(window_type, 2.00)
        
        # 2. Apply the inverse formula
        raw_size = (sample_rate / target_rbw) * factor
        
        # 3. Round up to the next highest power of 2 for optimal FFT performance
        optimal_size = int(2 ** np.ceil(np.log2(raw_size)))
        
        # Enforce safe minimum/maximum operational bounds for SDR stability
        return max(128, min(optimal_size, 16384))
    
    # Example Usage:
    # fft_size = calculate_optimal_fft_size(sample_rate=2048000, target_rbw=4000)
    # print(fft_size) -> Outputs: 1024