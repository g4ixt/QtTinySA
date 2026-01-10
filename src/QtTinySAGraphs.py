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

logging.basicConfig(format="%(message)s", level=logging.INFO)
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
