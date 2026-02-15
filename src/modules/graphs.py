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
from PySide6.QtCore import QObject, Qt
from PySide6.QtGui import QLinearGradient
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

        # Calculate Noise Power in 1Hz bandwidth, correcting for tinySA rbw filter shape and measured bandwidth
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


class Spectrum(QObject):

    def __init__(self, ui_widget):
        super().__init__()
        self.create_plot(ui_widget)

    def create_plot(self, ui_widget):
        ui_widget.addLegend(offset=(30, 400))
        self.trace = ui_widget.plot([], [], width=1, padding=0)

        # create 4 markers, bound to the trace
        self.trace.m0 = Marker(ui_widget, self.trace, 'm1', 0.1)
        self.trace.m1 = Marker(ui_widget, self.trace, 'm2', 0.9)
        self.trace.m2 = Marker(ui_widget, self.trace, 'm3', 1.7)
        self.trace.m3 = Marker(ui_widget, self.trace, 'm4', 2.5)
        self.mkr_list = [self.trace.m0, self.trace.m1, self.trace.m2, self.trace.m3]

    def enable(self, show=True):  # show or hide a trace
        if show:
            self.trace.show()
            # for mkr in self.mkr_list:
            #     mkr.line.show()
        else:
            self.trace.hide()
            for mkr in self.mkr_list:
                mkr.line.hide()

    def update(self, frequencies, levels):
        self.trace.setData(frequencies, levels)

    def set_colour(self, pen):
        self.trace.setPen(pen)
        for mkr in self.mkr_list:
            mkr.line.setPen(pen)
            mkr.delta.setPen(pen)

    def fetch_data(self):
        '''return the (freq, level) data used to plot the trace'''
        arr = self.trace.getOriginalDataset()
        return arr[0], arr[1]

    def mkr_start(self, startF):  # set marker to the sweep start frequency
        for mkr in self.mkr_list:
            if mkr.mkr_type != 'Off':
                mkr.line.setValue(startF * 1e6)
                mkr.mkr_type()


class Marker:
    def __init__(self, ui_widget, trace, name, box):
        # self.level = 1  # marker tracking level (min or max), set per marker from GUI
        # self.mkr_type = 'Normal'

        # self.fifo = queue.SimpleQueue()
        self.dBm = -140
        self.create_lines(ui_widget, name, box)
        # self.parent = trace
        # self.createMarkerTimePlot()
        # self.polar = pattern.ui.plotwidget.plot([], [], name=name, width=1, padding=0)
        # self.runTimer = QtCore.QElapsedTimer()  # for polar plot
        # self.sweeptime = []  # for polar plot
        # self.amplitude = []  # for polar plot
        # self.samples = []  # for polar plot

    def create_lines(self, ui_widget, name, box):
        self.line = ui_widget.addLine(88, 90, movable=True, name=name, pen=pyqtgraph.mkPen('y', width=0.5), label=name)
        self.delta = ui_widget.addLine(0, 90, movable=True, name=name,
                                       pen=pyqtgraph.mkPen('y', width=0.5, style=Qt.PenStyle.DashLine), label='d'+name)
        self.line.addMarker('^', 0, 10)
        self.deltaF = 0  # the delta marker frequency difference
        self.deltaRelative = True
        # self.delta.sigClicked.connect(self.dmr_click)
        self.line.sigClicked.connect(self.mkr_click)
        self.markerBox = pyqtgraph.TextItem(text='', border=None, anchor=(-0.7, -box), fill='k')  # box is vertical posn
        self.markerBox.setParentItem(ui_widget.plotItem)

    # def guiRef(self, opt):
    #     guiFields = ({'1': QtTSA.m1_type, '2': QtTSA.m2_type, '3': QtTSA.m3_type, '4': QtTSA.m4_type},
    #                  {'1': QtTSA.m1trace, '2': QtTSA.m2trace, '3': QtTSA.m3trace, '4': QtTSA.m4trace},
    #                  {'1': 'm1f', '2': 'm2f', '3': 'm3f', '4': 'm4f'},
    #                  {'1': QtTSA.m1track, '2': QtTSA.m2track, '3': QtTSA.m3track, '4': QtTSA.m4track})
    #     Ref = guiFields[opt].get(self.name)
    #     return Ref

   # def traceLink(self, setting):
   #      traces = {'1': T1, '2': T2, '3': T3, '4': T4}
   #      self.linked = traces.get(str(setting))
   #      tint = str("background-color: '" + self.linked.pen + "';")
   #      # self.guiRef(0).setStyleSheet(tint)
   #      self.guiRef(1).setStyleSheet(tint)
   #      checkboxes.dwm.submit()

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

    def to_start(self, startF):  # set marker to the sweep start frequency
        self.line.setValue(startF * 1e6)

    def spread(self, startF, stopF, gap):  # spread markers equally across scan range
        span = (stopF - startF) * 1e6
        if self.line.value() <= startF * 1e6 or self.line.value() > stopF * 1e6:
            self.line.setValue(startF * 1e6 + (0.2 * gap * span))

    def mkr_click(self):  # toggle visibility of associated delta marker
        delta = 1e5
        if self.delta.value() != 0:
            self.delta.hide()
            self.delta.setValue(0)
        else:
            self.delta.show()
            self.delta.setValue(self.line.value() + delta)
            self.deltaF = 0
            self.deltaF = self.delta.value() - self.line.value()

    def dmkr_click(self):  # toggle relative or absolute labelling
        if self.deltaRelative:
            self.deltaRelative = False
        else:
            self.deltaRelative = True

    def dmkr_move(self):  # set the delta freq offset
        self.deltaF = self.delta.value() - self.line.value()
        self.updateMarker()

    def set_type(self, m_type, m_track):
        self.mkr_type = m_type
        self.level = m_track
        if self.mkr_type == 'Off' or self.mkr_type == '':
            self.line.hide()
            self.delta.hide()
            self.delta.setValue(0)
            self.deltaF = 0
        else:
            self.line.show()

    def set_delta(self):  # delta marker locking to reference marker
        self.delta.setValue(self.line.value() + self.deltaF)
        self.update()

    def mkr_update(self, frequencies, levels, maskFreq):  # called by sweepComplete() and fifo timer
        if self.mkr_type == 'Off':
            self.markerBox.setVisible(False)
            return
        else:
            self.markerBox.setVisible(True)

        # flatten the arrays
        frequencies.reshape(-1)
        levels.reshape(-1)

        if self.mkr_type in ('Max', 'Min'):
            
            ### test # self.calcMaskFreq(frequencies)
            # self.level = 1
            ####
            
            maxmin = self.maxMin(frequencies, levels, maskFreq)
            # maxmin is a tuple of lists where [0, x] are indices in the frequency array of the max and [1, x] are min
            logging.debug(f'updateMarker(): maxmin = {maxmin}')
            if self.mkr_type == 'Max':
                self.line.setValue(maxmin[0][self.level])
                if self.delta.value() != 0:
                    self.delta.setValue(maxmin[0][self.level] + self.deltaF)  # needs to be index delta not F
            if self.mkr_type == 'Min':
                self.line.setValue(maxmin[1][self.level])
                if self.delta.value() != 0:
                    self.delta.setValue(maxmin[1][self.level] + self.deltaF)  # needs to be index delta not F

        lineIndex = np.argmin(np.abs(frequencies - (self.line.value())))  # find closest value in freq array
        logging.debug(f'updateMarker(): index={lineIndex} frequency={frequencies[lineIndex]}')
        self.line.setValue(frequencies[lineIndex])
        self.dBm = levels[lineIndex]

        decimal = Calc.precision(frequencies, frequencies[0])  # set decimal places
        unit, multiple = Calc.unit(frequencies[0])  # set units
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

    # def addFreqMarker(self, freq, colour, name, band=True):  # adds simple freq marker without full marker capability
    #     if QtTSA.presetLabel.isChecked():
    #         if band:
    #             self.marker = QtTSA.graphWidget.addLine(freq, 90, pen=pyqtgraph.mkPen(colour, width=0.5,
    #                                                     style=QtCore.Qt.PenStyle.DashLine), label=name,
    #                                                     labelOpts={'position': 0.97, 'color': (colour)})
    #         else:
    #             self.marker = QtTSA.graphWidget.addLine(freq, 90, pen=pyqtgraph.mkPen(colour, width=0.5,
    #                                                     style=QtCore.Qt.PenStyle.DashLine), label=name,
    #                                                     labelOpts={'position': 0.04, 'color': (colour),
    #                                                     'anchors': ((0, 0.2), (0, 0.2))})
    #         self.marker.label.setMovable(True)
    #     else:
    #         self.marker = QtTSA.graphWidget.addLine(freq, 90, pen=pyqtgraph.mkPen(colour, width=0.5,
    #                                                 style=QtCore.Qt.PenStyle.DashLine))
    #     self.fifo.put(self.marker)  # store the marker object in a queue
    #     logging.debug(f'addFreqMarker(): fifo size = {self.fifo.qsize()}')

    # def delFreqMarkers(self):
    #     for i in range(0, self.fifo.qsize()):
    #         QtTSA.graphWidget.removeItem(self.fifo.get())  # remove the marker and its corresponding object in the queue

    def maxMin(self, frequencies, levels, maskFreq):  # finds the signal max/min (indexes) for setting markers
        # # mask outside high/low freq boundary lines
        # levels = np.ma.masked_where(frequencies > highF.line.value(), levels)
        # levels = np.ma.masked_where(frequencies < lowF.line.value(), levels)

        # # mask readings below threshold line
        # levels = np.ma.masked_where(levels <= threshold.line.value(), levels)

        maxi = [np.argmax(levels)]  # the index of the highest peak in the masked readings array
        mini = [np.argmin(levels)]  # the index of the deepest minimum in the masked readings array
        nextMax = nextMin = levels
        for i in range(8):
            # mask frequencies around detected peaks and find the next 8 highest/lowest peaks
            nextMax = np.ma.masked_where(np.abs(frequencies[maxi[-1]] - frequencies) < maskFreq, nextMax)
            maxi.append(np.argmax(nextMax))
            nextMin = np.ma.masked_where(np.abs(frequencies[mini[-1]] - frequencies) < maskFreq, nextMin)
            mini.append(np.argmin(nextMin))
        return (list(frequencies[maxi]), list(frequencies[mini]))

    def setLevel(self, setting):
        self.level = setting - 1  # array indexes start at 0 not 1


        # def createMarkerTimePlot(self):
        #     '''multiplot is the name of the graphics layout grid widget in the fading dialogue window'''
        #     self.tplot = multiplot.addPlot(title='Marker ' + self.name)
        #     self.tplot.setAxisItems({'bottom': pyqtgraph.DateAxisItem()})
        #     self.tplot.showGrid(x=True, y=True)
        #     multiplot.nextRow()
        #     self.tplot.plot([], [])

        # def updateMarkerTimePlot(self, frequencies, timeNow):  # called by sweepComplete()
        #     self.tplot.clear()  # if it's not cleared the GUI runs slower and slower
        #     if self.mkr_type == 'Off':
        #         self.tplot.hide()
        #         self.runTimer.invalidate()
        #         return
        #     else:
        #         self.tplot.show()
        #     decimal = Calc.precision(frequencies, frequencies[0])  # set decimal places
        #     unit, multiple = Calc.unit(self.line.value())  # set units
        #     self.tplot.setTitle(f'Marker {self.name} = {(self.line.value()/multiple):.{decimal}f}' + unit)

        #     tinySA.timeMarkVals[tinySA.timeIndex, 0] = timeNow
        #     tinySA.timeMarkVals[tinySA.timeIndex, int(self.name)] = self.dBm
        #     self.tplot.plot(tinySA.timeMarkVals[:, 0], tinySA.timeMarkVals[:, int(self.name)], pen=self.linked.pen)

        #     if self.runTimer.isValid():  # polar pattern plot is active
        #         self.updatePolarPlot()

        # def setPolarPlot(self):
        #     if self.mkr_type != 'Off':
        #         pattern.ui.progress.setValue(0)
        #         if pattern.ui.manual.isChecked():
        #             self.samples = []
        #         else:
        #             self.sweeptime = []
        #             self.amplitude = []
        #         self.runTimer.start()

        # def updatePolarPlot(self):
        #     if pattern.ui.manual.isChecked():
        #         if len(self.samples) < pattern.scanCount.value():
        #             self.samples.append(self.dBm)
        #             pattern.ui.progress.setValue(int(100 * len(self.samples) / pattern.scanCount.value()))
        #             return
        #         else:
        #             if pattern.ui.max.isChecked():
        #                 self.amplitude.append(np.max(self.samples))
        #             if pattern.ui.avg.isChecked():
        #                 self.amplitude.append(np.average(self.samples))
        #             if pattern.ui.min.isChecked():
        #                 self.amplitude.append(np.min(self.samples))
        #             self.sweeptime.append(pattern.heading.value())  # append the current heading (instead of rotation time)
        #             self.runTimer.invalidate()
        #             theta = np.divide((np.multiply(self.sweeptime, np.pi)), 180)  # convert heading in degrees to radians
        #     else:
        #         if self.sweeptime == []:  # auto plot has just been started
        #             self.sweeptime.append(0)
        #         else:
        #             self.sweeptime.append(self.runTimer.elapsed() / 1000)
        #         self.amplitude.append(self.dBm)
        #         if pattern.ui.clockwise.isChecked():
        #             theta = np.divide((np.multiply(self.sweeptime, 2 * np.pi)), pattern.ui.rotateTime.value())
        #             pattern.ui.heading.setValue(int(360*(theta[-1] / (2 * np.pi))))
        #         else:
        #             theta = np.divide((np.multiply(self.sweeptime, -2 * np.pi)), pattern.ui.rotateTime.value())
        #             pattern.ui.heading.setValue(360 + int(360*(theta[-1] / (2 * np.pi))))
        #         pattern.ui.progress.setValue(int(100 * (abs(theta[-1]) / (2 * np.pi))))

        #     peak = max(np.max(self.amplitude), pattern.ui.refdBm.value())  # peak is maximum when antenna points at Tx
        #     factor = 40 - peak  # correction factor to make the max signal amplitude read 40 units on the polar grid
        #     dBm = np.round(np.add(self.amplitude, factor), decimals=1)

        #     r = np.clip(dBm, 0, 40)  # clip the signal vector to a max amplitude of 40 and minimum of 0
        #     x = np.multiply(r, np.sin(theta))
        #     y = np.multiply(r, np.cos(theta))
        #     self.polar.setData(x, y, pen=self.linked.pen)

        #     if self.sweeptime[-1] >= pattern.ui.rotateTime.value():  # rotation is complete
        #         self.runTimer.invalidate()
        #         if pattern.ui.beamUp.isChecked() and not pattern.ui.manual.isChecked():
        #             self.rotatePolarPlot(r, theta)

        # def rotatePolarPlot(self, r, theta):
        #     pkIndex = np.argmax(self.amplitude)  # find the array index of the maximum signal
        #     width = 0
        #     for i in range(pkIndex, len(self.amplitude) - 1):
        #         if self.amplitude[i] == self.amplitude[pkIndex]:
        #             width += 1
        #     pkIndex = pkIndex + int(width / 2)  # beam centre is half the width of a symetrical antenna main lobe
        #     pkBearing = 2 * np.pi * pkIndex / len(self.amplitude)

        #     # calculate new values to rotate display
        #     theta = np.subtract(theta, pkBearing)
        #     x = np.multiply(r, np.sin(theta))
        #     y = np.multiply(r, np.cos(theta))
        #     self.polar.setData(x, y, pen=self.linked.pen)