#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright 2026 Ian Jefferson G4IXT
# SPDX-License-Identifier: GPL-3.0-or-later

"""
Created on Mon Jan 12 11:23:46 2026

@author: ian
"""
import logging
import numpy as np
from PySide6.QtCore import QObject


class Calc(QObject):
    def __init__(self, frequencies, spotF):
        super().__init__()

    def Precision(frequencies, spotF):  # sets the marker indicated frequency precision
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

    def Unit(spotF):
        index = int(np.log10(abs(spotF)))
        suffix = ['Hz', 'Hz', 'Hz', 'kHz', 'kHz', 'kHz', 'MHz', 'MHz', 'MHz', 'GHz', 'GHz']
        multiple = [1, 1, 1, 1e3, 1e3, 1e3, 1e6, 1e6, 1e6, 1e9, 1e9]
        return suffix[index], multiple[index]