#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright 2026 Ian Jefferson G4IXT
# SPDX-License-Identifier: GPL-3.0-or-later

"""
Created on Wed Nov 26 15:42:02 2025

@author: ian
"""
import logging
import numpy as np
from PySide6.QtWidgets import QGraphicsEllipseItem, QGraphicsLineItem, QGraphicsSimpleTextItem
from PySide6.QtCore import QObject, Qt, QElapsedTimer
from PySide6.QtGui import QLinearGradient, QBrush, QColor, QTransform
from PySide6.QtGraphs import QSurface3DSeries, QSurfaceDataProxy, QGraphsTheme
from PySide6.QtGraphsWidgets import Q3DSurfaceWidgetItem
import pyqtgraph

from modules.utility import Calc

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
        self.surface.setAmbientLightStrength(0.5)
        self.surface.setLightStrength(0.4)

        self._dataProxy = QSurfaceDataProxy()
        self._dataSeries = QSurface3DSeries(self._dataProxy)

        self.setAxis(self.surface.axisX, "Frequency MHz")
        self.setAxis(self.surface.axisY, "Signal dBm")
        self.setAxis(self.surface.axisZ, "Sweep number")

        self.updater = SurfaceUpdater(frequencies, readings)
        self.surface.addSeries(self.updater._dataSeries)
        self.surface.setShadowStrength(0)

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
        # self.updateTimeSpectrum(frequencies, readings)

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
        self._dataSeries.setDrawMode(QSurface3DSeries.DrawSurface)


class PhaseNoiseGraph(QObject):

    def __init__(self, ui_widget, frequencies, readings, rbw):
        super().__init__()
        self.create_plot(ui_widget)

    def create_plot(self, ui_widget):
        ui_widget.addLegend(offset=(30, 400))
        self.lowerSB = ui_widget.plot([], [], name='lsb', width=1, padding=0, pen='m')
        self.upperSB = ui_widget.plot([], [], name='usb', width=1, padding=0, pen='y')
        self.create_baseline(ui_widget)
        self.create_box(ui_widget)

    def create_baseline(self, ui_widget):
        self.base_noise = ui_widget.plot([], [], name='tinySA typical', width=1, padding=0, pen='g')

    def create_box(self, ui_widget):
        self.box = pyqtgraph.TextItem(text='', color='k', border='y', fill='y', anchor=(-1, -15))  # anchor y=vertical
        self.box.setParentItem(ui_widget.plotItem)
        self.box.setVisible(False)

    def update(self, index, frequencies, levels, rbw):  # index = marker index in 'frequencies'
        '''M1 is used in max tracking mode to find the frequency and level reference of the signal to be measured'''

        # Noise Power is defined in 1Hz bandwidth, so calc correction factors for tinySA filter shape and rbw
        eqnbw = SHAPE_FACTOR.get(rbw)
        factor = 10 * np.log10(rbw * 1e3)  # correction factor from rbw to 1Hz bandwidth
        fStep = frequencies[1] - frequencies[0]
        mask = int((2 * rbw * 1e3) / fStep)

        # lsb freq points in the sideband below the marker excluding nearer than 2 * rbw kHz
        delta = levels[:index-mask] - levels[index]  # array of levels referenced to the marked carrier
        freqOffset = (frequencies[index] - frequencies[:index-mask])
        dBcHz = delta + eqnbw - factor
        self.lowerSB.setData(freqOffset, dBcHz)

        # usb freq points in the sideband above the marker excluding nearer than 2 * rbw kHz
        delta = levels[index+mask+1:] - levels[index]  # array of levels referenced to the marked carrier
        freqOffset = (frequencies[index+mask+1:] - frequencies[index])
        dBcHz = delta + eqnbw - factor
        self.upperSB.setData(freqOffset, dBcHz)

        # show signal frequency from Marker in label box
        self.box.setVisible(True)
        decimal = Calc.precision(frequencies, frequencies[0])
        unit, multiple = Calc.unit(frequencies[0])
        self.box.setText(f'{frequencies[index]/multiple:.{decimal}f}{unit}')

        # draw tinySA typical baseline phase noise on the graph
        if frequencies[0] < 100e6:
            baseline = np.interp(freqOffset, PN_AT_10MHZ[:, 0], PN_AT_10MHZ[:, 1])
        else:
            baseline = np.interp(freqOffset, PN_AT_1152MHZ[:, 0], PN_AT_1152MHZ[:, 1])
        self.base_noise.setData(freqOffset, baseline)


class PolarGraph(QObject):
    def __init__(self, ui_widget, rings, radius):
        super().__init__()
        self.create_plot(ui_widget, rings, radius)
        self.runTimer = QElapsedTimer()
        self.samples = []
        self.sweeptime = []  # list of timestamps of each update
        self.amplitude = []  # list of dBm levels for the marked frequency at each update
        self.polar = ui_widget.plotwidget.plot([], [], name='pattern', width=1, padding=0)

    def create_plot(self, ui_widget, rings, radius):
        '''Draw concentric circles and radial lines to simulate polar axes.'''
        ui_widget.plotwidget.setAspectLocked(True)
        ui_widget.plotwidget.hideAxis('bottom')
        ui_widget.plotwidget.hideAxis('left')
        for i in range(0, rings + 1):
            r = i * radius / rings
            circle = QGraphicsEllipseItem(-r, -r, 2 * r, 2 * r)
            circle.setPen(pyqtgraph.mkPen('grey', width=0.3))
            label = QGraphicsSimpleTextItem(str(-40 + 10 * i))
            label.setBrush(QBrush(QColor('red')))
            label.setPos(0.5, r+2)
            label.setScale(0.2)
            label.setTransform(QTransform().scale(1, -1))  # otherwise the text is mysteriously inverted
            ui_widget.plotwidget.addItem(circle)
            ui_widget.plotwidget.addItem(label)
        r = radius - (radius/(rings*3))
        circle = QGraphicsEllipseItem(-r, -r, 2 * r, 2 * r)
        circle.setPen(pyqtgraph.mkPen('red', width=0.3))
        ui_widget.plotwidget.addItem(circle)

        # Add radial lines
        for angle_deg in range(0, 360, 15):
            angle_rad = np.deg2rad(angle_deg)
            x = radius * np.cos(angle_rad)
            y = radius * np.sin(angle_rad)
            line = QGraphicsLineItem(0, 0, x, y)
            line.setPen(pyqtgraph.mkPen('grey', width=0.5))
            ui_widget.plotwidget.addItem(line)

    def set_plot(self, ui_widget):
        ui_widget.progress.setValue(0)
        if ui_widget.manual.isChecked():
            self.samples = []
        else:
            self.sweeptime = []  # list of timestamps of each update
            self.amplitude = []  # list of dBm levels for the marked frequency at each update
        self.runTimer.start()

    def update_plot(self, ui_widget, index, levl):
        if self.runTimer.isValid():
            if ui_widget.manual.isChecked():
                self.amplitude.append(levl[index])
                self.sweeptime.append(ui_widget.heading.value())  # append current heading, not rotation time
                theta = np.divide((np.multiply(self.sweeptime, np.pi)), 180)  # convert heading degrees to radians
                self.runTimer.invalidate()
            else:
                if self.sweeptime == []:  # list is empty so auto-plot has just been started
                    self.sweeptime.append(0)
                else:
                    self.sweeptime.append(self.runTimer.elapsed() / 1000)
                self.amplitude.append(levl[index])
                if ui_widget.clockwise.isChecked():
                    # theta is the current heading in radians (based on rotation time set in GUI)
                    theta = np.divide((np.multiply(self.sweeptime, 2 * np.pi)), ui_widget.rotateTime.value())
                    ui_widget.heading.setValue(int(360*(theta[-1] / (2 * np.pi))))
                else:
                    theta = np.divide((np.multiply(self.sweeptime, -2 * np.pi)), ui_widget.rotateTime.value())
                    ui_widget.heading.setValue(360 + int(360*(theta[-1] / (2 * np.pi))))
                ui_widget.progress.setValue(int(100 * (abs(theta[-1]) / (2 * np.pi))))

            if ui_widget.manual.isChecked():
                peak = max(np.max(self.amplitude), ui_widget.refdBm.value())
            else:
                peak = np.max(self.amplitude)
                ui_widget.refdBm.setValue(peak)
            factor = 40 - peak  # correction to make the max signal amplitude read 40 units on the polar grid
            dBm = np.round(np.add(self.amplitude, factor), decimals=1)

            # clip to min 0 max 40dB and convert to cartesian co-ords in order to plot on simulated polar chart
            r = np.clip(dBm, 0, 40)
            x = np.multiply(r, np.sin(theta))
            y = np.multiply(r, np.cos(theta))
            self.polar.setData(x, y, pen='y')

            if self.sweeptime[-1] >= ui_widget.rotateTime.value():  # auto-plot is complete
                ui_widget.progress.setValue(100)
                self.runTimer.invalidate()
                if ui_widget.beamUp.isChecked() and not ui_widget.manual.isChecked():
                    self.rotate_plot(r, theta)

    def rotate_plot(self, r, theta):
        pkIndex = np.argmax(self.amplitude)  # find the array index of the maximum signal
        width = 0
        for i in range(pkIndex, len(self.amplitude) - 1):
            if self.amplitude[i] == self.amplitude[pkIndex]:
                width += 1
        pkIndex = pkIndex + int(width / 2)  # beam centre is half the width of a symetrical antenna main lobe
        pkBearing = 2 * np.pi * pkIndex / len(self.amplitude)

        # calculate new values to rotate display
        theta = np.subtract(theta, pkBearing)
        x = np.multiply(r, np.sin(theta))
        y = np.multiply(r, np.cos(theta))
        self.polar.setData(x, y, pen='y')


class SpectrumGraph(QObject):

    def __init__(self, s_widget, w_widget, h_widget, m_widget):
        super().__init__()
        self.create(s_widget, w_widget, h_widget, m_widget)
        self.count = 0  # the number of completed scans
        self.monitor_data = np.ndarray(2)
        self.wfall_data = np.ndarray(2)
        self.startF = None
        self.stopF = None
        self.points = None
        self.ps_markers = []

    def create(self, s_widget, w_widget, h_widget, m_widget):
        # create the main spectrum screen trace
        s_widget.addLegend(offset=(30, 400))
        self.trace = s_widget.plot([], [], width=1, padding=0)
        self.trace.is_visible = True

        # create four markers, all bound to this trace
        self.trace.m0 = Marker(s_widget, self, 'm1', 0.1)
        self.trace.m1 = Marker(s_widget, self, 'm2', 0.9)
        self.trace.m2 = Marker(s_widget, self, 'm3', 1.7)
        self.trace.m3 = Marker(s_widget, self, 'm4', 2.5)
        self.mkr_list = [self.trace.m0, self.trace.m1, self.trace.m2, self.trace.m3]

        # create the waterfall graph and its histogram
        self.waterfall = pyqtgraph.ImageItem(axisOrder='row-major')
        w_widget.addItem(self.waterfall)
        self.histogram = pyqtgraph.HistogramLUTItem(gradientPosition='right', orientation='vertical')
        self.histogram.setImageItem(self.waterfall)  # connects the histogram to the waterfall
        self.histogram.gradient.loadPreset('plasma')  # set the colour map
        self.waterfall.setLevels((-100, -25))  # needs to be here, after histogram is created
        h_widget.addItem(self.histogram)

        # create the signal level monitor graph (which uses the current marker 0 level for each spectrum)
        self.monitor = m_widget.addPlot(title='monitor')
        self.monitor.setAxisItems({'bottom': pyqtgraph.DateAxisItem()})
        self.monitor.showGrid(x=True, y=True)
        m_widget.nextRow()
        self.monitor.plot([], [])

    def enable(self, show=True):  # show or hide a trace
        if show:
            self.trace.show()
            self.trace.is_visible = True
        else:
            self.trace.hide()
            self.trace.is_visible = False
            for mkr in self.mkr_list:
                mkr.line.hide()

    def updateTrace(self, frequencies, levels):
        self.trace.setData(frequencies, levels)

    def set_colour(self, pen):
        self.pen = pen
        self.trace.setPen(pen)
        for mkr in self.mkr_list:
            mkr.line.setPen(pen, style=Qt.DashLine)
            mkr.delta.setPen(pen, style=Qt.DotLine)

    def fetch_data(self):
        '''return the (freq, level) data used to plot the trace'''
        arr = self.trace.getOriginalDataset()
        return arr[0], arr[1]

    def mkr_start(self, startF):  # set marker to the sweep start frequency
        for mkr in self.mkr_list:
            if mkr.mkr_type != 'Off':
                mkr.line.setValue(startF * 1e6)
                mkr.mkr_type()

    def update_monitor(self, frequencies, timeNow):  # called by updateGUI at the end of every scan
        self.monitor.clear()  # if it's not cleared the GUI runs slower and slower
        if self.trace.is_visible:
            self.monitor.show()
        else:
            self.monitor.hide()
            # self.runTimer.invalidate()
            return
        decimal = Calc.precision(frequencies, frequencies[0])  # set decimal places
        unit, multiple = Calc.unit(self.trace.m0.line.value())  # set units
        self.monitor.setTitle(f'{(self.trace.m0.line.value()/multiple):.{decimal}f} {unit}')

        time_points = np.size(self.monitor_data, 0)  # set in 'QtTinySA.scan'
        if self.count < time_points:
            # the array still has space for new readings, so write them in the next position (scan count)
            self.monitor_data[self.count, 0] = timeNow
            self.monitor_data[self.count, 1] = self.trace.m0.dBm
        else:
            # the array is full, so roll it left and write new readings to the end instead
            self.monitor_data = np.roll(self.monitor_data, -1, axis=0)
            self.monitor_data[-1, 0] = timeNow
            self.monitor_data[-1, 1] = self.trace.m0.dBm
        self.monitor.plot(self.monitor_data[:, 0], self.monitor_data[:, 1], pen=self.pen)

    def set_ps_mkr(self, ui_widget, freq, colour, name):  # sets preset freq markers
        mkr_pen = pyqtgraph.mkPen(colour, width=0.5, style=Qt.PenStyle.DashLine)
        marker = ui_widget.addLine(freq, 90, pen=mkr_pen, name=name)
        self.ps_markers.append(marker)
        return marker

    def del_ps_mkr(self, ui_widget):
        while len(self.ps_markers) > 0:
            for marker in self.ps_markers:
                ui_widget.removeItem(marker)
                self.ps_markers.remove(marker)

    def label_ps_mkr(self, marker, colour, rotate, band):
        if rotate:
            txt_angle = 90
        else:
            txt_angle = 0
        if band:
            pyqtgraph.InfLineLabel(marker, marker._name, movable=True, position=0.94,
                                   angle=txt_angle, color=colour)
        else:
            pyqtgraph.InfLineLabel(marker, marker._name, movable=True, position=0.08, anchors=(0.5, 0.5),
                                   angle=txt_angle, color=colour)


class Marker:
    def __init__(self, ui_widget, trace, name, box):
        self.dBm = -140
        self.create_lines(ui_widget, name, box)
        self.setSignals()
        self.spectrum = trace

    def create_lines(self, ui_widget, name, box):
        self.line = ui_widget.addLine(88, 90, movable=True, name=name,
                                      pen=pyqtgraph.mkPen('y', width=0.5), label=name)
        self.delta = ui_widget.addLine(0, 90, movable=True, name=name,
                                       pen=pyqtgraph.mkPen('y', width=0.5, style=Qt.PenStyle.DashLine),
                                       label=chr(916)+name)
        self.line.addMarker('^', 0, 10)
        self.deltaF = 0  # the delta marker frequency difference
        self.deltaRelative = True
        self.markerBox = pyqtgraph.TextItem(text='', border=None, anchor=(-0.7, -box), fill='k')  # box is vertical posn
        self.markerBox.setParentItem(ui_widget.plotItem)
        self.line.hide()
        self.delta.hide()

    # def setup(self, colour):
    #     '''restore the marker frequencies from the configuration database and set starting conditions'''
    #     self.line.setValue(numbers.tm.record(0).value(self.guiRef(2)))
    #     self.line.label.setColor(colour)
    #     self.line.label.setPosition(0.02)
    #     self.line.label.setMovable(True)
    #     self.line.setPen(color=colour, width=0.5)
    #     self.mType()
    #     # self.traceLink(self.guiRef(1).value())
    #     self.setLevel(self.guiRef(3).value())
    #     self.delta.hide()
    #     self.delta.setValue(0)
    #     self.deltaF = 0
    #     self.delta.label.setPosition(0.05)
    #     self.delta.label.setMovable(True)
    #     M2.tplot.setXLink(M1.tplot)
    #     M3.tplot.setXLink(M1.tplot)
    #     M4.tplot.setXLink(M1.tplot)

    def setSignals(self):
        self.line.sigPositionChanged.connect(self.set_delta)
        self.delta.sigPositionChanged.connect(self.dmkr_move)
        self.delta.sigClicked.connect(self.dmkr_click)
        self.line.sigClicked.connect(self.mkr_click)

    def to_start(self, startF):  # set marker to the sweep start frequency
        self.line.setValue(startF * 1e6)
        self.delta.hide()

    def spread(self, startF, stopF, gap):  # spread markers equally across scan range
        span = (stopF - startF) * 1e6
        if self.line.value() <= startF * 1e6 or self.line.value() > stopF * 1e6:
            self.line.setValue((startF * 1e6) + (0.2 * gap * span))
            self.delta.hide()

    def mkr_click(self):  # toggle visibility of associated delta marker
        freq = self.spectrum.fetch_data()[0]
        span = freq[-1] - freq[0]
        if self.delta.value() != 0:
            self.delta.hide()
            self.delta.setValue(0)
        else:
            self.delta.show()
            self.delta.setValue(self.line.value() + (span/20))
            self.deltaF = 0
            self.deltaF = self.delta.value() - self.line.value()

    def dmkr_click(self):  # toggle relative or absolute labelling
        if self.deltaRelative:
            self.deltaRelative = False
        else:
            self.deltaRelative = True

    def dmkr_move(self):  # set the delta freq offset
        self.deltaF = self.delta.value() - self.line.value()

    def set_type(self, m_type, m_track):
        self.mkr_type = m_type
        self.level = m_track
        if self.mkr_type == 'Off':
            self.line.hide()
            self.delta.hide()
            self.delta.setValue(0)
            self.deltaF = 0
        else:
            self.line.show()

    def set_delta(self):  # delta marker locking to reference marker
        self.delta.setValue(self.line.value() + self.deltaF)

    def mkr_update(self, limits, trace_on):
        # called by updateGUI when scanning or by a timer when not
        arr = self.spectrum.trace.getData()
        if arr[0] is None:
            return

        frequencies = arr[0]
        levels = arr[1]
        if frequencies[0] == frequencies[-1]:  # markers are not relevant in zero span
            return

        if self.mkr_type == 'Off' or not trace_on:
            self.line.hide()
            self.delta.hide()
            self.markerBox.setVisible(False)
            return
        else:
            self.line.show()
            self.markerBox.setVisible(True)

        if self.mkr_type in ('Max', 'Min'):
            maxmin = Calc.maxMin(frequencies, levels, limits)
            # maxmin is a tuple of lists where [0, x] are indices in the frequency array of the max
            # and [1, x] are min.  maskFreq represents an exclusion zone around the mkr, based on rbw
            if self.mkr_type == 'Max':
                self.line.setValue(maxmin[0][self.level - 1])
                if self.delta.value() != 0:
                    self.delta.setValue(maxmin[0][self.level - 1] + self.deltaF)  # needs to be index delta not F
            if self.mkr_type == 'Min':
                self.line.setValue(maxmin[1][self.level - 1])
                if self.delta.value() != 0:
                    self.delta.setValue(maxmin[1][self.level - 1] + self.deltaF)  # needs to be index delta not F

        # find closest value in freq array
        lineIndex = np.argmin(np.abs(frequencies - (self.line.value())))
        self.line.setValue(frequencies[lineIndex])
        self.dBm = levels[lineIndex]

        # set decimal places and units
        decimal = Calc.precision(frequencies, frequencies[0])
        unit, multiple = Calc.unit(frequencies[0])

        self.markerBox.setText(f'{self.line.name()} {self.line.value()/multiple:.{decimal}f}{unit} {self.dBm:.1f}dBm')

        if self.deltaF != 0:
            d_indx = np.argmin(np.abs(frequencies - (self.delta.value())))  # closest value in freq array
            if self.deltaRelative:
                deltadBm = levels[d_indx]
                dBm = deltadBm - self.dBm
                decimal = Calc.precision(frequencies, self.deltaF)  # deltaF can be negative
                unit, multiple = Calc.unit(self.deltaF)
                self.delta.label.setText(
                    f'{chr(916)}{self.line.name()} {dBm:.1f}dB\n{(self.deltaF / multiple):.{decimal}f}{unit}')
            else:
                dBm = levels[d_indx]
                self.delta.label.setText(
                    f'{chr(916)}{self.line.name()} {dBm:.1f}dBm\n{(self.delta.value()/multiple):.{decimal}f}{unit}')

    def setLevel(self, setting):
        self.level = setting - 1  # array indexes start at 0 not 1
