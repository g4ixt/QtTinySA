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
            
        # Boundary check: At beta = 0.5842*(29)**0.4 + 0.07886*(29) ≈ 2.7845,
        # the formula switches over to the linear >50 dB equation.
        beta_switch = 0.5842 * (29)**0.4 + 0.07886 * (29)
        
        if beta > beta_switch:
            # Case 1: Attenuation > 50 dB (Linear relationship)
            # beta = 0.1102 * (A - 8.7)
            A = (beta / 0.1102) + 8.7
            return A
        else:
            # Case 2: 21 <= Attenuation <= 50 dB
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

    @staticmethod
    def cubic_spline_interp(x_data, y_data, x_new):
        """
        Interpolates 1D data using a Natural Cubic Spline algorithm powered strictly by NumPy.
        
        :param x_data: Array of measured calibration frequencies (must be strictly increasing).
        :param y_data: Array of measured path deltas (dB).
        :param x_new:  The target frequency or array of frequencies to calculate corrections for.
        :return:       Interpolated dB correction factor(s).
        """
        x = np.asarray(x_data, dtype=float)
        y = np.asarray(y_data, dtype=float)
        x_new = np.asarray(x_new, dtype=float)
        
        n = len(x)
        h = np.diff(x)
        
        # 1. Setup the tridiagonal system matrix components for natural boundary conditions
        A = np.zeros((n, n))
        B = np.zeros(n)
        
        A[0, 0] = 1.0
        A[n-1, n-1] = 1.0
        
        for i in range(1, n - 1):
            A[i, i - 1] = h[i - 1]
            A[i, i] = 2.0 * (h[i - 1] + h[i])
            A[i, i + 1] = h[i]
            B[i] = 3.0 * ((y[i + 1] - y[i]) / h[i] - (y[i] - y[i - 1]) / h[i - 1])
            
        # 2. Solve for the second derivatives (curvatures) at each node point
        c = np.linalg.solve(A, B)
        
        # 3. Derive the remaining spline coefficients (a, b, d) for each interval
        a = y[:-1]
        b = (y[1:] - y[:-1]) / h - h * (2.0 * c[:-1] + c[1:]) / 3.0
        d = (c[1:] - c[:-1]) / (3.0 * h)
        c_intervals = c[:-1]
        
        # 4. Vectorized interpolation evaluation loop
        # Vectorized search to find which interval bin each 'x_new' value belongs to
        indices = np.searchsorted(x, x_new) - 1
        indices = np.clip(indices, 0, n - 2) # Handle edge out-of-bounds inputs safely
        
        # Evaluate the piecewise polynomial: S_i(x) = a + b(x-x_i) + c(x-x_i)^2 + d(x-x_i)^3
        dx = x_new - x[indices]
        y_new = a[indices] + b[indices] * dx + c_intervals[indices] * (dx ** 2) + d[indices] * (dx ** 3)
        
        return y_new
    
    
    # ==========================================
    # Verification & Accuracy Evaluation Sandbox
    # ==========================================
    if __name__ == "__main__":
        # Measured calibration data points across an RF boundary (e.g., HF/VHF transition)
        cal_freqs = [0.5e6, 1.0e6, 5.0e6, 10.0e6, 20.0e6, 30.0e6, 50.0e6, 75.0e6, 100.0e6]
        cal_deltas = [18.5,  14.2,  8.1,   5.4,   3.2,   2.1,   1.5,   1.2,   1.0] # Smoothly curving dB drop
        
        # Target custom runtime channel frequency to evaluate
        target_freq = 15.5e6 
        
        # Calculate corrections using Cubic Spline vs standard Linear Interpolation
        spline_correction = numpy_cubic_spline_interp(cal_freqs, cal_deltas, target_freq)
        linear_correction = np.interp(target_freq, cal_freqs, cal_deltas)
        
        print(f"Target Operating Frequency: {target_freq / 1e6:.3f} MHz")
        print(f"NumPy Piecewise Linear Correction:  {linear_correction:.4f} dB")
        print(f"NumPy Natural Cubic Spline Correction: {spline_correction:.4f} dB")
        print(f"Calculated Curve Deviation Delta:      {abs(spline_correction - linear_correction):.4f} dB")


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
