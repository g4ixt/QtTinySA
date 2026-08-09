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


class Calc(QObject):
    def __init__(self, frequencies, spotF):
        super().__init__()

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

    def unit(spotF):
        index = int(np.log10(abs(spotF)))
        suffix = ['Hz', 'Hz', 'Hz', 'kHz', 'kHz', 'kHz', 'MHz', 'MHz', 'MHz', 'GHz', 'GHz']
        multiple = [1, 1, 1, 1e3, 1e3, 1e3, 1e6, 1e6, 1e6, 1e9, 1e9]
        return suffix[index], multiple[index]

    # def maxMin(frequencies, levels, maskFreq):  # finds the signal max/min (indexes) for setting markers
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

def resource_path(filename: str) -> str:
    """Get resources path in a safe way to work on terminal AND in macOS app bundles."""
    if getattr(sys, 'frozen', False):
        base = sys._MEIPASS
    else:
        base = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(base, filename)