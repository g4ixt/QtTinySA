#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright 2026 Ian Jefferson G4IXT
# SPDX-License-Identifier: GPL-3.0-or-later

"""
Created on Mon Jan 12 11:23:46 2026
@author: ian

Helper functions

"""
import os
import sys
import logging
import numpy as np
from PySide6.QtCore import QObject


class Calc():

    @staticmethod
    def precision(frequencies, spotF):  # sets the marker indicated frequency precision
        span = frequencies[-1] - frequencies[0]
        HzPp = span / len(frequencies)  # Hz per point
        spotF = abs(spotF)  # delta markers can be 'negative' f
        if span > 0:
            if spotF < 1000:
                decimal = 0
            else:
                decimal = np.clip(int(np.log10(spotF)) - int(np.log10(HzPp)), 0, 6)  # number of decicimals
                logging.debug(f'fPrecision: pF = {HzPp} dp = {decimal}')
        else:
            decimal = 6
        return decimal

    @staticmethod
    def unit(spotF):
        index = int(np.log10(abs(spotF)))
        suffix = ['Hz', 'Hz', 'Hz', 'kHz', 'kHz', 'kHz', 'MHz', 'MHz', 'MHz', 'GHz', 'GHz']
        multiple = [1, 1, 1, 1e3, 1e3, 1e3, 1e6, 1e6, 1e6, 1e9, 1e9]
        return suffix[index], multiple[index]

    @staticmethod
    def maxMin(frequencies, levels, limits):  # finds the signal max/min (indexes) for setting markers
        maskFreq = limits[0]
        high = limits[1]
        low = limits[2]
        threshold = limits[3]

        # mask outside high/low freq boundary and level threshold lines
        levels = np.ma.masked_where(frequencies > high, levels)
        levels = np.ma.masked_where(frequencies < low, levels)
        levels = np.ma.masked_where(levels <= threshold, levels)

        maxi = [np.argmax(levels)]  # the index of the highest peak in the masked readings array
        mini = [np.argmin(levels)]  # the index of the deepest minimum in the masked readings array
        nextMax = nextMin = levels
        for i in range(8):
            # mask frequencies around detected peaks and find the NEXT 8 highest/lowest peaks
            nextMax = np.ma.masked_where(np.abs(frequencies[maxi[-1]] - frequencies) < maskFreq, nextMax)
            maxi.append(np.argmax(nextMax))
            nextMin = np.ma.masked_where(np.abs(frequencies[mini[-1]] - frequencies) < maskFreq, nextMin)
            mini.append(np.argmin(nextMin))
        return (list(frequencies[maxi]), list(frequencies[mini]))

    @staticmethod
    def kaiser_attenuation(beta):
        """
        Calculate the side-lobe suppression (attenuation in dB) from the Kaiser beta parameter.
        """
        if beta <= 0.0:
            return 21.0  # Base case for a rectangular-like window
            
        # At beta = 0.5842*(29)**0.4 + 0.07886*(29) ≈ 2.7845, switch to the linear >50 dB equation.
        beta_switch = 0.5842 * (29)**0.4 + 0.07886 * (29)
        
        if beta > beta_switch:
            # Attenuation > 50 dB (Linear relationship) beta = 0.1102 * (A - 8.7)
            A = (beta / 0.1102) + 8.7
            return A
        else:
            # 21 <= Attenuation <= 50 dB
            # beta = 0.5842 * (A - 21)**0.4 + 0.07886 * (A - 21)
            # Let x = (A - 21)**0.2. Then: 0.07886*x^5 + 0.5842*x^2 - beta = 0
            coefficients = [0.07886, 0.0, 0.0, 0.5842, 0.0, -beta]
            roots = np.roots(coefficients)
            
            # We only care about the real, positive root
            real_root = roots[np.isreal(roots) & (roots > 0)].real
            
            if len(real_root) > 0:
                x_val = real_root[0]
                A = (x_val ** 5) + 21.0
                return A
            else:
                return 21.0
    
    # @staticmethod()
    # def compute_power(fft, fft_size):
    #     # 20*log10 is correct here, because soapy_receiver sends the magnitude, not the squared magnitude
    #     power = 20 * np.log10(fft / fft_size + 1e-20)  # +1e-20 gives -400dBm if fft values are zero
    #     return power.astype(np.float32)


    @staticmethod
    def resample_spectrum(freq, levl, is_dBm=True):
        GUI_DISPLAY_COLUMNS = 1024
        """
        Resamples 'freq' (Hz) and 'levl' to exact 1,024-element arrays.
        
        Parameters:
        - freq: 1D NumPy array of frequencies in Hz.
        - levl: 1D NumPy array of levels (in dBm if is_dBm=True, or linear power if is_dBm=False).
        - is_dBm: Set True for tinySA (converts to linear mW before interpolating).
                  Set False for raw SDR FFT linear power (converts to dB at the end).
                  
        For SDR (pyFFTW): Pass raw linear squared magnitudes (np.abs(fft_shifted) ** 2) with is_dBm=False.
        It skips all linear conversions, max-pools the raw FFT power directly, tracks the exact peak frequency bins,
        and converts only the final 1,024 points to dB at the very end.
                  
        Returns:
        - out_freq: 1,024-element array of peak-aligned frequencies (Hz).
        - out_levl: 1,024-element array of levels in dBm.
        """
        num_points = len(levl)
    
        # Convert tinySA dBm to linear power (mW) if interpolating is needed
        # (Skip conversion if num_points is already an exact multiple of 1024)
        needs_interp = (num_points < GUI_DISPLAY_COLUMNS) or (num_points % GUI_DISPLAY_COLUMNS != 0)
        
        if is_dBm and needs_interp:
            linear_levl = 10.0 ** (levl / 10.0)
        else:
            linear_levl = levl
    
        # Exact Match of 1,024 points
        if num_points == GUI_DISPLAY_COLUMNS:
            out_freq = freq
            out_linear = linear_levl
    
        # Upsample if < 1,024 points, e.g., 450 tinySA points.  [should we prevent SDR FFT < 1024 to avoid upsampling?]
        elif num_points < GUI_DISPLAY_COLUMNS:
            old_x = np.linspace(0, 1, num_points)
            new_x = np.linspace(0, 1, GUI_DISPLAY_COLUMNS)
            
            out_freq = np.interp(new_x, old_x, freq)
            out_linear = np.interp(new_x, old_x, linear_levl)
    
        # Downsample if > 1,024 points
        else:
            # If size is an exact Power-of-2 multiple (e.g., 16,384 SDR points)
            if num_points % GUI_DISPLAY_COLUMNS == 0:
                work_freq = freq
                work_levl = linear_levl
                factor = num_points // GUI_DISPLAY_COLUMNS
    
            # If size is an odd point count (e.g., 1,521 tinySA points)
            # Stretch in linear power to the next multiple of 1,024 (e.g., 2,048) before downsampling
            else:
                factor = int(np.ceil(num_points / GUI_DISPLAY_COLUMNS))
                aligned_size = factor * GUI_DISPLAY_COLUMNS
                
                old_x = np.linspace(0, 1, num_points)
                aligned_x = np.linspace(0, 1, aligned_size)
                
                work_freq = np.interp(aligned_x, old_x, freq)
                work_levl = np.interp(aligned_x, old_x, linear_levl)
    
            # Reshape into (1024, factor) blocks and extract peaks
            levl_matrix = work_levl.reshape(GUI_DISPLAY_COLUMNS, factor)
            out_linear = levl_matrix.max(axis=1)
    
            # Find the exact column index of each peak within its block to identify freqs of peaks
            local_peak_idx = levl_matrix.argmax(axis=1)
            flat_peak_idx = (np.arange(GUI_DISPLAY_COLUMNS) * factor) + local_peak_idx
            out_freq = work_freq[flat_peak_idx]
    
        # Convert back to dBm if it started that way
        if is_dBm and not needs_interp:
            # Already in dBm and never converted to linear
            out_levl = out_linear
        elif is_dBm:
            # Convert linear power back to dB / dBm safely (avoiding log10(0))
            out_levl = 10.0 * np.log10(np.maximum(out_linear, 1e-20))
        else:
            out_levl = out_linear
    
        return out_freq, out_levl


class FakePortInfo():
    """Mimics serial.tools.list_ports_common.ListPortInfo using values supplied.  This code is from Claude ai"""

    def __init__(self, device, name=None, description="n/a", hwid="n/a",
                 vid=None, pid=None, serial_number=None, location=None,
                 manufacturer=None, product=None, interface=None):
        self.device = device
        self.name = name if name is not None else device.rsplit("/", 1)[-1]
        self.description = description
        self.hwid = hwid
        self.vid = vid
        self.pid = pid
        self.serial_number = serial_number
        self.location = location
        self.manufacturer = manufacturer
        self.product = product
        self.interface = interface

    def __str__(self):
        return "{} - {}".format(self.device, self.description)

    def __eq__(self, other):
        return isinstance(other, FakePortInfo) and self.device == other.device

    def __hash__(self):
        return hash(self.device)

    def __getitem__(self, index):
        return (self.device, self.description, self.hwid)[index]

def resource_path(filename: str) -> str:
    """Get resources path in a safe way to work on terminal AND in macOS app bundles."""
    if getattr(sys, 'frozen', False):
        base = sys._MEIPASS
    else:
        base = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(base, filename)
