#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Nov 26 15:42:02 2025

@author: ian
"""
import logging
import numpy as np
from PySide6.QtCore import QObject, Qt
from PySide6.QtGui import QLinearGradient
from PySide6.QtGraphs import QSurface3DSeries, QSurfaceDataProxy, QGraphsTheme
from PySide6.QtGraphsWidgets import Q3DSurfaceWidgetItem

import pyqtgraph

logging.basicConfig(format="%(message)s", level=logging.INFO)

# From https://github.com/Hagtronics/tinySA-Ultra-Phase-Noise
SHAPE_FACTOR = {0.2: 3.6, 1: -0.6, 3: -0.53, 10: 0, 30: 0, 100: 0, 300: 0, 600: 0, 850: 0}
# tinySA typical phase noise
PN_AT_10MHZ = np.loadtxt("10_baseline.txt")
PN_AT_1152MHZ = np.loadtxt("1152_baseline.txt")


class SurfaceGraph(QObject):

    def __init__(self, ui_widget, frequencies, readings):
        super().__init__()

        self.surface = Q3DSurfaceWidgetItem()
        self.surface.setWidget(ui_widget)

        self._dataProxy = QSurfaceDataProxy()
        self._dataSeries = QSurface3DSeries(self._dataProxy)

        self.setAxis(self.surface.axisX, "Frequency MHz")
        self.setAxis(self.surface.axisY, "Signal dBm")
        self.setAxis(self.surface.axisZ, "Sweep number")

        self.updater = SurfaceUpdater(frequencies, readings)
        self.surface.addSeries(self.updater._dataSeries)

    def setAxis(self, axis, name):
        ax = axis()
        ax.setTitle(name)
        ax.setTitleVisible(True)
        # ax.setLabelSize(1.5)

    def zoom(self, zoom):
        self.surface.setCameraZoomLevel(zoom)

    def rotateX(self, angle):
        self.surface.setCameraXRotation(angle)

    def rotateY(self, angle):
        self.surface.setCameraYRotation(angle)

    def setRange(self):
        axis = self.surface.axisY()
        maximum = np.round(axis.max(), decimals=-1)
        minimum = np.round(axis.min(), decimals=-1)
        axis.setMax(maximum)
        axis.setMin(minimum)


class SurfaceUpdater(QObject):

    def __init__(self, frequencies, readings):
        super().__init__()

        self._dataProxy = QSurfaceDataProxy()
        self._dataSeries = QSurface3DSeries(self._dataProxy)
        self.updateTimeSpectrum(frequencies, readings)

    def updateTimeSpectrum(self, frequencies, readings):
        startF = frequencies[0] / 1e6
        deltaF = (frequencies[1] - frequencies[0])/1e6
        self.setGradient()

        # re-populate the surface with the current readings
        self._dataProxy.resetArrayNp(startF, deltaF, 0, 1, readings)

    def setGradient(self):
        gr = QLinearGradient()
        gr.setColorAt(0.2, Qt.blue)
        gr.setColorAt(0.4, Qt.cyan)
        gr.setColorAt(0.6, Qt.magenta)
        gr.setColorAt(0.8, Qt.yellow)
        gr.setColorAt(0.9, Qt.red)

        self._dataSeries.setBaseGradient(gr)
        self._dataSeries.setColorStyle(QGraphsTheme.ColorStyle.RangeGradient)


class PhaseNoiseGraph(QObject):

    def __init__(self, ui_widget, name, frequencies, readings, rbw):
        super().__init__()
        self.create_plot(ui_widget, name)

    def create_plot(self, ui_widget, name):
        # ui_widget.addLegend()
        self.lowSB = ui_widget.plot([], [], name='lsb', width=1, padding=0, pen='y')
        self.upSB = ui_widget.plot([], [], name='usb', width=1, padding=0, pen='y')
        self.create_baseline(ui_widget)
        self.create_box(ui_widget)

    def create_baseline(self, ui_widget):
        self.base_noise = ui_widget.plot([], [], name='tinySA phase noise', width=1, padding=0, pen='g')

    def create_box(self, ui_widget):
        self.box = pyqtgraph.TextItem(text='', color='k', border='y', fill='y', anchor=(-8, -0.4))  # anchor y=vertical
        self.box.setParentItem(ui_widget.plotItem)
        self.box.setVisible(False)

    def update(self, index, frequencies, levels, rbw):  # index = marker index in 'frequencies'
        '''M1 is used in max tracking mode to find the frequency and level reference of the signal to be measured'''

        # Calculate Noise Power in 1Hz bandwidth, correcting for tinySA rbw filter shape and measured bandwidth
        eqnbw = SHAPE_FACTOR.get(rbw)
        factor = 10 * np.log10(rbw * 1e3)

        fStep = frequencies[1] - frequencies[0]
        mask = int((2 * rbw * 1e3) / fStep)

        # lsb freq points in the sideband below the marker excluding nearer than 2 * rbw kHz
        delta = levels[:index-mask] - levels[index]  # array of levels referenced to the marked carrier
        freqOffset = (frequencies[index] - frequencies[:index-mask])
        dBcHz = delta + eqnbw - factor
        self.lowSB.setData(freqOffset, dBcHz)

        # usb freq points in the sideband above the marker excluding nearer than 2 * rbw kHz
        delta = levels[index+mask:] - levels[index]  # array of levels referenced to the marked carrier
        freqOffset = (frequencies[index+mask:] - frequencies[index])
        dBcHz = delta + eqnbw - factor
        self.upSB.setData(freqOffset, dBcHz)

        # show signal frequency from Marker in label box
        self.box.setVisible(True)
        # decimal = M1.setPrecision(frequencies, frequencies[0])
        # unit, multiple = M1.setUnit(frequencies[0])  # set units
        # self.box.setText(f'{M1.line.value()/multiple:.{decimal}f}{unit}')
        self.box.setText(f'{frequencies[index]/1e6:.3f} MHz')

        # draw tinySA typical baseline phase noise on the graph
        if frequencies[0] < 100e6:
            baseline = np.interp(freqOffset, PN_AT_10MHZ[:, 0], PN_AT_10MHZ[:, 1])
        else:
            baseline = np.interp(freqOffset, PN_AT_1152MHZ[:, 0], PN_AT_1152MHZ[:, 1])
        self.base_noise.setData(freqOffset, baseline)
