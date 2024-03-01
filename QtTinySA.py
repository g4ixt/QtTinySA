#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright 2024 Ian Jefferson G4IXT
# SPDX-License-Identifier: GPL-3.0-or-later

"""TinySA Ultra GUI programme using Qt5 and PyQt.

This code attempts to replicate some of the TinySA Ultra on-screen commands and to provide PC control.
Development took place on Kubuntu 22.04LTS with Python 3.9 and PyQt5 using Spyder in Anaconda.

TinySA and TinySA Ultra are trademarks of Erik Kaashoek and are used with permission.

TinySA commands are based on Erik's Python examples: http://athome.kaashoek.com/tinySA/python/

Serial communication commands are based on Martin's Python NanoVNA/TinySA Toolset: https://github.com/Ho-Ro"""

import os
import time
import logging
import numpy as np
import queue
import shutil
import platformdirs
import csv
from platform import system
from PyQt5 import QtWidgets, QtCore, QtGui
from PyQt5.QtWidgets import QMessageBox, QDataWidgetMapper, QFileDialog
from PyQt5.QtSql import QSqlDatabase, QSqlRelation, QSqlRelationalTableModel, QSqlRelationalDelegate
import pyqtgraph
import QtTinySpectrum  # the GUI
import QtTSApreferences  # the GUI preferences window
import struct
import serial
from serial.tools import list_ports

#  For 3D
import pyqtgraph.opengl as pyqtgl

os.environ['PYOPENGL_PLATFORM'] = 'egl'

# Defaults to non local configuration/data dirs - needed for packaging
if system() == "Linux":
    os.environ['XDG_CONFIG_DIRS'] = '/etc:/usr/local/etc'
    os.environ['XDG_DATA_DIRS'] = '/usr/share:/usr/local/share'

logging.basicConfig(format="%(message)s", level=logging.INFO)
threadpool = QtCore.QThreadPool()
basedir = os.path.dirname(__file__)

# pyqtgraph pens
red = pyqtgraph.mkPen(color='r', width=1.0)
yellow = pyqtgraph.mkPen(color='y', width=1.0)
white = pyqtgraph.mkPen(color='w', width=1.0)
magenta = pyqtgraph.mkPen(color='m', width=1.0)
cyan = pyqtgraph.mkPen(color='c', width=1.0)
red_dash = pyqtgraph.mkPen(color='r', width=0.5, style=QtCore.Qt.DashLine)
blue_dash = pyqtgraph.mkPen(color='b', width=0.5,  style=QtCore.Qt.DashLine)

###############################################################################
# classes


class analyser:
    def __init__(self):
        self.usb = None
        self.sweeping = False
        self.signals = WorkerSignals()
        self.signals.result.connect(self.sigProcess)
        self.signals.fullSweep.connect(self.updateGUI)
        self.signals.finished.connect(self.threadEnds)
        self.runTimer = QtCore.QElapsedTimer()  # debug
        self.scale = 174
        self.scanMemory = 50
        self.scan3D = False
        self.surface = None
        self.vGrid = None
        self.usbCheck = QtCore.QTimer()
        self.usbCheck.timeout.connect(self.isConnected)
        self.fifo = queue.SimpleQueue()
        self.fifoTimer = QtCore.QTimer()
        self.fifoTimer.timeout.connect(self.usbSend)
        self.tinySA4 = None
        self.resBW = ['0.2', '1', '3', '10', '30', '100', '300', '600', '850']

    def openPort(self):
        self.dev = None
        # Get tinysa device (port) automatically using hardware ID
        VID = 0x0483  # 1155
        PID = 0x5740  # 22336
        device_list = list_ports.comports()
        for x in device_list:
            if x.vid == VID and x.pid == PID:
                self.dev = x.device
                logging.info(f'Found TinySA on {self.dev}')
        if self.dev is None:
            ui.version.setText('TinySA not found')
            if not self.usbCheck.isActive():
                logging.info('TinySA not found')
        if self.dev and self.usb is None:  # TinySA was found but serial comms not open
            try:
                self.usb = serial.Serial(self.dev, baudrate=576000)
                logging.info(f'Serial port open: {self.usb.isOpen()}')
            except serial.SerialException:
                logging.info('serial port exception')
                popUp('Serial Port Exception', QMessageBox.Ok, QMessageBox.Critical)
        if self.dev and self.usb:
            self.initialise()

    def closePort(self):
        if self.usb:
            self.usb.close()
            logging.info(f'Serial port open: {self.usb.isOpen()}')
            self.usb = None

    def isConnected(self):
        # triggered by self.usbCheck QTimer - if tinySA wasn't found checks repeatedly for device, i.e.'hotplug'
        if self.dev is None:
            self.openPort()
        else:
            self.usbCheck.stop()

    def initialise(self):
        i = 0
        hardware = ''
        while hardware[:6] != 'tinySA' and i < 3:  # try 3 times to detect TinySA
            hardware = self.version()
            logging.info(f'{hardware[:16]}')
            i += 1
            time.sleep(0.5)
        # hardware = 'tinySA'  # used for testing
        if hardware[:7] == 'tinySA4':  # It's an Ultra
            self.tinySA4 = True
            ui.spur_box.setTristate(True)  # TinySA Ultra has 'auto', 'on' and 'off' setting for Spur
            ui.spur_box.setCheckState(QtCore.Qt.PartiallyChecked)  # auto
        else:
            self.tinySA4 = False
            self.scale = 128
            self.resBW = self.resBW[2:8]  # TinySA Basic has fewer resolution bandwidth filters
            ui.spur_box.setTristate(False)  # TinySA Basic has only 'on' and 'off' setting for Spur'
            ui.spur_box.setChecked(True)  # on
        self.spur()

        # Basic has no lna
        ui.lna_label.setVisible(self.tinySA4)
        ui.lna_box.setVisible(self.tinySA4)
        ui.lna_box.setEnabled(self.tinySA4)
        self.lna()

        # set the frequency band & rbw comboboxes to suit detected hardware
        setPreferences()
        self.resBW.insert(0, 'auto')
        ui.rbw_box.addItems(self.resBW)
        ui.rbw_box.setCurrentIndex(len(self.resBW)-4)

        # show hardware information in GUI
        ui.battery.setText(self.battery())
        ui.version.setText(hardware[:16])

        # connect the rbw & frequency boxes here or it causes startup index errors when they are populated
        ui.rbw_box.currentIndexChanged.connect(rbwChanged)
        ui.start_freq.editingFinished.connect(self.freq_changed)
        ui.stop_freq.editingFinished.connect(self.freq_changed)
        ui.centre_freq.valueChanged.connect(lambda: self.freq_changed(True))  # centre/span mode
        ui.span_freq.valueChanged.connect(lambda: self.freq_changed(True))  # centre/span mode
        ui.band_box.activated.connect(band_changed)

        self.fifoTimer.start(500)  # calls self.usbSend() every 500mS to execute serial commands whilst not scanning

    def restoreSettings(self):
        # update centre freq, span, auto points and graph for the start/stop freqs loaded from database
        self.freq_changed(False)  # start/stop mode
        pointsChanged()
        ui.graphWidget.setXRange(ui.start_freq.value(), ui.stop_freq.value())

        # update trace and marker settings from the database.  1 = last saved (default) settings
        S1.dLoad(1)
        S2.dLoad(1)
        S3.dLoad(1)
        S4.dLoad(1)

        #  set each marker to a different colour
        S1.vline.label.setColor('y')
        S2.vline.setPen(color='m', width=0.75, style=QtCore.Qt.DashLine)
        # S2.vline.setPen(color=QtGui.QColor('darkgreen'), width=0.75, style=QtCore.Qt.DashLine)
        S2.vline.label.setColor('m')
        S3.vline.setPen(color='c', width=0.75, style=QtCore.Qt.DashLine)
        S3.vline.label.setColor('c')
        S4.vline.setPen(color='w', width=0.75, style=QtCore.Qt.DashLine)
        S4.vline.label.setColor('w')

        setPreferences()

    def scan(self):  # called by 'run' button
        self.scan3D = ui.Enabled3D.isChecked()
        if self.usb is not None:
            if self.sweeping:  # if it's running, stop it
                self.sweeping = False  # tells the measurement thread to stop once current scan complete
                ui.scan_button.setEnabled(False)  # prevent repeat presses of 'stop'
                ui.run3D.setEnabled(False)
            else:
                try:  # start measurements
                    self.fifoTimer.stop()
                    self.scanCount = 1
                    self.clearBuffer()
                    self.setRBW()
                    self.runButton('Stop')
                    self.usbSend()
                    self.startMeasurement()  # runs measurement in separate thread
                except serial.SerialException:
                    logging.info('serial port exception')
                    self.dev = None
                    self.closePort()
        else:
            popUp('TinySA not found', QMessageBox.Ok, QMessageBox.Critical)

    def startMeasurement(self):
        frequencies = self.set_frequencies()
        self.usbSend()
        points = np.size(frequencies)
        readings = np.full((self.scanMemory, points), -100, dtype=float)
        self.sweep = Worker(self.measurement, frequencies, readings)  # workers are auto-deleted when thread stops
        self.sweeping = True
        self.createTimeSpectrum(frequencies, readings)
        self.reset3D()
        threadpool.start(self.sweep)

    def usbSend(self):
        try:
            self.usb.timeout = 1
        except AttributeError:  # don't know why this happens on second run of programme.  Temporary workaround.
            self.usb = serial.Serial(self.dev, baudrate=576000)
            logging.info(f'Serial port open: {self.usb.isOpen()}')
        while self.fifo.qsize() > 0:
            command = self.fifo.get(block=True, timeout=None)
            logging.debug(command)
            self.usb.write(command)
            self.usb.read_until(b'ch> ')  # skip command echo and prompt

    def serialQuery(self, command):
        self.usb.timeout = 1
        logging.debug(command)
        self.usb.write(command)
        self.usb.read_until(command + b'\n')  # skip command echo
        response = self.usb.read_until(b'ch> ')
        logging.debug(response)
        return response[:-6].decode()  # remove prompt

    def set_frequencies(self):  # creates a numpy array of equi-spaced freqs in Hz. Also called by measurement thread.
        startF = ui.start_freq.value()*1e6  # freq in Hz
        stopF = ui.stop_freq.value()*1e6
        points = self.setPoints()
        frequencies = np.linspace(startF, stopF, points, dtype=np.int64)
        logging.debug(f'frequencies = {frequencies}')
        return frequencies

    def freq_changed(self, centre=False):
        if centre:
            startF = ui.centre_freq.value()-ui.span_freq.value()/2
            stopF = ui.centre_freq.value()+ui.span_freq.value()/2
            ui.start_freq.setValue(startF)
            ui.stop_freq.setValue(stopF)
        else:
            startF = ui.start_freq.value()  # freq in MHz
            stopF = ui.stop_freq.value()
            if startF > stopF:
                stopF = startF
                ui.stop_freq.setValue(stopF)
            ui.centre_freq.setValue(startF + (stopF - startF) / 2)
            ui.span_freq.setValue(stopF - startF)
        ui.graphWidget.setXRange(startF, stopF)
        self.resume()

    def setRBW(self):  # may be called by measurement thread as well as normally
        rbw = ui.rbw_box.currentText()  # ui values are discrete ones in kHz
        logging.debug(f'rbw = {rbw}')
        command = f'rbw {rbw}\r'.encode()
        self.fifo.put(command)

    def setPoints(self):  # may be called by measurement thread as well as normally
        if ui.points_auto.isChecked():
            rbw = float(ui.rbw_box.currentText())
            points = preferences.rbw_x.value() * int((ui.span_freq.value()*1000)/(rbw))  # RBW multiplier * freq in kHz
            points = np.clip(points, preferences.minPoints.value(), preferences.maxPoints.value())  # limit points
        else:
            points = ui.points_box.value()
            logging.debug(f'setPoints: points = {ui.points_box.value()}')
        return points

    def clearBuffer(self):
        self.usb.timeout = 1
        while self.usb.inWaiting():
            self.usb.read_all()  # keep the serial buffer clean
            time.sleep(0.01)

    def sweepTimeout(self, frequencies):  # freqs are in Hz
        startF = frequencies[0]
        stopF = frequencies[-1]
        points = np.size(frequencies)
        if ui.rbw_box.currentIndex() == 0:  # rbw is auto
            # rbw auto setting from tinySA: ~7 kHz per 1 MHz scan frequency span
            rbw = (stopF - startF) * 7e-6
        else:
            rbw = float(ui.rbw_box.currentText())
        rbw = np.clip(rbw, float(self.resBW[1]), float(self.resBW[-1]))  # apply limits
        # timeout can be very long - use a heuristic approach
        # 1st summand is the scanning time, 2nd summand is the USB transfer overhead
        timeout = ((stopF - startF) / 20e3) / (rbw ** 2) + points / 500
        if (ui.spur_box.checkState() == 1 and stopF > 8 * 1e8) or ui.spur_box.checkState() == 2:
            timeout *= 2  # scan time doubles with spur on or spur auto above 800 MHz
        # transfer is done in blocks of 20 points, this is the timeout for one block
        timeout = timeout * 20 / points + 1  # minimum is 1 second
        logging.debug(f'sweepTimeout = {timeout:.2f} s')
        return timeout

    def measurement(self, frequencies, readings):  # runs in a separate thread
        points = np.size(readings, 1)
        self.threadRunning = True
        firstSweep = True
        self.scanCount = 1
        while self.sweeping:
            try:
                self.usb.timeout = self.sweepTimeout(frequencies)
                command = f'scanraw {int(frequencies[0])} {int(frequencies[-1])} {int(points)}\r'.encode()
                self.usb.write(command)
                index = 0
                # self.runTimer.start()  # debug
                self.usb.read_until(command + b'\n{')  # skip command echo
                dataBlock = ''
                while dataBlock != b'}ch' and index < points:  # if '}ch' it's reached the end of the scan points
                    dataBlock = (self.usb.read(3))  # read a block of 3 bytes of data
                    logging.debug(f'dataBlock: {dataBlock}\n')
                    if dataBlock != b'}ch':
                        # logging.debug(f'measurement: index {index} elapsed time = {self.runTimer.nsecsElapsed()/1e6}')
                        c, data = struct.unpack('<' + 'cH', dataBlock)
                        logging.debug(f'measurement: dataBlock: {dataBlock} data: {data}\n')
                        readings[0, index] = (data / 32) - self.scale  # scale 0..4095 -> -128..-0.03 dBm
                        if index // 20 == index / 20 or index == (points - 1):  # update traces every 20 readings
                            self.signals.result.emit(frequencies, readings)  # send readings to sigProcess()
                        index += 1
                    logging.debug(f'measurement: level = {(data / 32) - self.scale}dBm')
                self.usb.read(2)  # discard the command prompt
                if firstSweep:
                    readings = np.full((self.scanMemory, points), readings[0], dtype=float)
                    firstSweep = False
                self.scanCount += 1
                self.signals.fullSweep.emit(frequencies, readings)  # updateGUI() only once per sweep (performance)
                readings = np.roll(readings, 1, axis=0)  # readings row 0 is now full: roll it down 1 row
                readings[0] = readings[1]  # populate each sweep with previous sweep as default
                # logging.debug(f'elapsed time = {self.runTimer.nsecsElapsed()/1e6}')  # debug
                if self.fifo.qsize() > 0:  # a setting has changed
                    self.setRBW()
                    frequencies = self.set_frequencies()
                    points = np.size(frequencies)
                    readings = np.full((self.scanMemory, points), -100, dtype=float)
                    firstSweep = True
                    self.createTimeSpectrum(frequencies, readings)
                    self.scanCount = 1
                    self.usbSend()
            except serial.SerialException:
                logging.info('serial port exception')
                self.sweeping = False
        self.threadRunning = False
        self.signals.finished.emit()

    def threadEnds(self):
        self.runButton('Run')
        self.fifoTimer.start(500)

    def sigProcess(self, frequencies, readings):  # readings from the worker thread result signal every 20 measurements
        if ui.avgSlider.value() > self.scanCount:  # slice using use scanCount to stop default values swamping average
            readingsAvg = np.average(readings[:self.scanCount, ::], axis=0)
        else:
            readingsAvg = np.average(readings[:ui.avgSlider.value(), ::], axis=0)
        readingsMax = np.amax(readings[:self.scanMemory, ::], axis=0)
        readingsMin = np.amin(readings[:self.scanMemory, ::], axis=0)
        options = {'Normal': readings[0], 'Average': readingsAvg, 'Max': readingsMax, 'Min': readingsMin}
        S1.updateTrace(frequencies, options.get(S1.traceType))
        S2.updateTrace(frequencies, options.get(S2.traceType))
        S3.updateTrace(frequencies, options.get(S3.traceType))
        S4.updateTrace(frequencies, options.get(S4.traceType))

    def createTimeSpectrum(self, frequencies, readings):
        points = np.size(frequencies)
        x = np.arange(start=0, stop=self.scanMemory, step=1)  # the time axis depth
        y = np.arange(start=0, stop=points)  # the frequency axis width
        z = readings  # the measurement axis heights in dBm
        logging.debug(f'z = {z}')
        if self.surface:  # if 3D spectrum exists, clear it
            ui.openGLWidget.clear()
        self.surface = pyqtgl.GLSurfacePlotItem(x=-x, y=y, z=z, shader='heightColor',
                                                computeNormals=ui.glNormals.isChecked(), smooth=ui.glSmooth.isChecked())

        #  for each colour, map = pow(z * colorMap[0] + colorMap[1], colorMap[2])
        self.surface.shader()['colorMap'] = np.array([ui.rMulti.value(),      # red   [0]
                                                      ui.rConst.value(),      # red   [1]
                                                      ui.rExponent.value(),   # red   [2]
                                                      ui.gMulti.value(),      # green [3]
                                                      ui.gConst.value(),      # green [4]
                                                      ui.gExponent.value(),   # green [5]
                                                      ui.bMulti.value(),      # blue  [6]
                                                      ui.bConst.value(),      # blue  [7]
                                                      ui.gExponent.value()])  # blue  [8]

        self.surface.translate(16, -points/40, -8)  # front/back, left/right, up/down
        self.surface.scale(points/1250, 0.05, 0.1, local=True)
        ui.openGLWidget.addItem(self.surface)

        # Add a vertical grid to the 3D view
        self.vGrid = pyqtgl.GLGridItem(glOptions='translucent', color=(255, 255, 255, 70))
        self.vGrid.setSize(x=12, y=points/20, z=1)
        self.vGrid.rotate(90, 0, 1, 0)
        self.vGrid.setSpacing(1, 1, 2)
        ui.openGLWidget.addItem(self.vGrid)
        if ui.grid.isChecked():
            self.vGrid.show()
        else:
            self.vGrid.hide()

    def updateGUI(self, frequencies, readings):  # called once per scan by fullSweep signal from measurement() thread
        if ui.points_auto.isChecked():
            ui.points_box.setValue(np.size(frequencies))
        if ui.Enabled3D.isChecked():
            z = readings + 120  # Surface plot height shader needs positive numbers so convert from dBm to dBf
            logging.debug(f'z = {z}')
            self.surface.setData(z=z)  # update 3D graph
            params = ui.openGLWidget.cameraParams()
            logging.debug(f'camera {params}')
        fPeaks = self.peakDetect(frequencies, readings)
        S1.updateMarker(frequencies, readings[0, :], fPeaks)
        S2.updateMarker(frequencies, readings[0, :], fPeaks)
        S3.updateMarker(frequencies, readings[0, :], fPeaks)
        S4.updateMarker(frequencies, readings[0, :], fPeaks)

    def peakDetect(self, frequencies, readings):
        # find the signal peak values for setting peak markers
        Avg = np.average(readings[:ui.avgSlider.value(), ::], axis=0)
        # calculate a frequency width factor to use to mask readings above and below each peak frequency
        if ui.rbw_box.currentText() == 'auto':
            fWidth = preferences.rbw_x.value() * float(self.resBW[-1]) * 1e3
        else:
            fWidth = preferences.rbw_x.value() * float(ui.rbw_box.currentText()) * 1e3
        peaks = [np.argmax(Avg)]  # the index of the highest peak in the averaged readings array
        for i in range(3):
            # mask frequencies around detected peaks and find the next 3 highest peaks
            Avg = np.ma.masked_where(np.abs(frequencies[peaks[-1]] - frequencies) < fWidth, Avg)
            peaks.append(np.argmax(Avg))
        return list(frequencies[peaks])

    def orbit3D(self, sign, azimuth=True):  # orbits the camera around the 3D plot
        degrees = ui.rotateBy.value()
        if azimuth:
            ui.openGLWidget.orbit(sign*degrees, 0)  # sign controls direction and is +1 or -1
        else:
            ui.openGLWidget.orbit(0, sign*degrees)

    def axes3D(self, sign, axis):  # shifts the plot along one of its 3 axes - time, frequency, signal
        pixels = ui.panBy.value()
        options = {'X': (pixels*sign, 0, 0), 'Y': (0, pixels*sign, 0), 'Z': (0, 0, pixels*sign)}
        s = options.get(axis)
        ui.openGLWidget.pan(s[0], s[1], s[2], relative='global')

    def reset3D(self):  # sets the 3D view back to the starting point
        ui.openGLWidget.reset()
        self.orbit3D(135, 'X')
        ui.openGLWidget.pan(0, 0, -10, relative='global')
        self.zoom3D()

    def grid(self, sign):  # moves the grid backwards and forwards on the time axis
        step = ui.rotateBy.value()
        if ui.grid.isChecked():
            self.vGrid.translate(step*sign, 0, 0)

    def zoom3D(self):  # zooms the camera in and out
        zoom = ui.zoom.value()
        ui.openGLWidget.setCameraParams(distance=zoom)

    def runButton(self, action):
        # Update the Run/Stop buttons' text and colour
        ui.scan_button.setText(action)
        ui.run3D.setText(action)
        if action == 'Stopping':
            ui.scan_button.setStyleSheet('background-color: yellow')
            ui.run3D.setStyleSheet('background-color: yellow')
        else:
            ui.scan_button.setStyleSheet('background-color: white')
            ui.run3D.setStyleSheet('background-color: white')
            ui.scan_button.setEnabled(True)
            ui.run3D.setEnabled(True)

    def pause(self):
        # pauses the sweeping in either input or output mode
        command = 'pause\r'.encode()
        self.fifo.put(command)

    def resume(self):
        # resumes the sweeping in either input or output mode
        command = 'resume\r'.encode()
        self.fifo.put(command)

    def reset(self):
        # not yet found any detail for what is actually reset
        command = 'reset\r'.encode()
        self.fifo.put(command)

    def battery(self):
        command = 'vbat\r'.encode()
        vbat = self.serialQuery(command)
        return vbat

    def version(self):
        command = 'version\r'.encode()
        version = self.serialQuery(command)
        return version

    def spur(self):
        sType = ui.spur_box.checkState()
        if sType == 1:
            ui.spur_box.setText('Auto')
        else:
            ui.spur_box.setText('')
        options = {0: 'spur off\r'.encode(), 1: 'spur auto\r'.encode(), 2: 'spur on\r'.encode()}
        command = options.get(sType)
        self.fifo.put(command)

    def lna(self):
        if ui.lna_box.isChecked():
            command = 'lna on\r'.encode()
            ui.atten_auto.setEnabled(False)  # attenuator and lna are switched so mutually exclusive
            ui.atten_auto.setChecked(False)
            ui.atten_box.setEnabled(False)
            ui.atten_box.setValue(0)
        else:
            command = 'lna off\r'.encode()
            ui.atten_auto.setEnabled(True)
            ui.atten_auto.setChecked(True)
        self.fifo.put(command)


class display:
    def __init__(self, name, pen):
        self.name = name
        self.trace = ui.graphWidget.plot([], [], name=name, pen=pen, width=1)
        self.traceType = 'Normal'  # Normal, Average, Max, Min
        self.markerType = 'Normal'  # Normal, Delta; Peak
        self.vline = ui.graphWidget.addLine(88, 90, movable=True, name=name,
                                            pen=pyqtgraph.mkPen('y', width=0.5, style=QtCore.Qt.DashLine),
                                            label="{value:.2f}")
        self.hline = ui.graphWidget.addLine(y=0, movable=False, pen=red_dash, label='',
                                            labelOpts={'position': 0.025, 'color': ('r')})
        self.deltaF = 0  # the difference between this marker and Reference Marker (1)
        self.fifo = queue.SimpleQueue()

    def mStart(self):
        # set marker to the sweep start frequency
        if self.guiRef(0).isChecked():
            self.vline.setValue(ui.start_freq.value())
            self.mType()

    def mSpread(self):
        # spread markers equally across scan range
        if self.guiRef(0).isChecked():
            self.vline.setValue(ui.start_freq.value() + (0.2 * int(self.name) * ui.span_freq.value()))
            self.mType()

    def mType(self):
        self.markerType = self.guiRef(1).currentText()  # current combobox value from appropriate GUI field
        if self.markerType == 'Delta':
            self.deltaF = self.vline.value() - S1.vline.value()
            self.vline.label.setText(f'M{self.vline.name()} {chr(916)}{self.deltaF:.3f}MHz')
        if 'Peak' in (S1.markerType[:4], S2.markerType[:4], S3.markerType[:4], S4.markerType[:4]):
            S4.hline.show()  # the peak detection threshold line
        else:
            S4.hline.hide()

    def mDelta(self):  # delta marker locking to reference marker S1
        if self.markerType == 'Delta':
            self.vline.setValue(S1.vline.value() + self.deltaF)
            S1.vline.setPen(color='y', width=1.0)

    # The set of 4 functions below are needed until I understand how to make dataWidgetMapper work with comboboxes
    def mData(self, setting, saving=True):
        markers.tm.setFilter('display = ' + str(self.name) + ' AND setting = ' + str(setting))
        markers.tm.select()
        record = markers.tm.record(0)
        if saving:
            record.setValue('frequency', float(self.vline.value()))
            record.setValue('type', self.markerType)
            markers.tm.setRecord(0, record)
        else:
            self.vline.setValue(record.value('frequency'))
            self.markerType = record.value('type')
            self.guiRef(1).setCurrentText(self.markerType)
            logging.debug(f'marker f = {record.value("frequency")}')
            self.vline.label.setMovable(True)
            self.mEnable()
            self.mType()

    def tData(self, setting, saving=True):
        traces.tm.setFilter('display = ' + str(self.name) + ' AND setting = ' + str(setting))
        traces.tm.select()
        record = traces.tm.record(0)
        if saving:
            record.setValue('type', self.traceType)
            traces.tm.setRecord(0, record)
        else:
            self.traceType = record.value('type')
            self.guiRef(3).setCurrentText(self.traceType)
            S1.hEnable(preferences.neg25Line)
            S2.hEnable(preferences.zeroLine)
            S3.hEnable(preferences.plus6Line)

    def dSave(self, setting):
        self.tData(setting, True)
        self.mData(setting, True)  # true = saving

    def dLoad(self, setting):
        self.mData(setting, False)  # false = not saving but loading
        self.tData(setting, False)
        self.tEnable()
    # The set of 4 functions above are needed until understand how to make dataWidgetMapper work with comboboxes

    def guiRef(self, opt):
        guiFields = ({'1': ui.marker1, '2': ui.marker2, '3': ui.marker3, '4': ui.marker4},
                     {'1': ui.m1_type, '2': ui.m2_type, '3': ui.m3_type, '4': ui.m4_type},
                     {'1': ui.trace1, '2': ui.trace2, '3': ui.trace3, '4': ui.trace4},
                     {'1': ui.t1_type, '2': ui.t2_type, '3': ui.t3_type, '4': ui.t4_type})
        Ref = guiFields[opt].get(self.name)
        return Ref

    def tType(self):
        self.traceType = self.guiRef(3).currentText()  # 3 selects trace type comboboxes

    def mEnable(self):  # show or hide a marker
        if self.guiRef(0).isChecked():  # 0 selects marker checkboxes
            self.vline.show()
        else:
            self.vline.hide()
        checkboxes.dwm.submit()

    def hEnable(self, limit):  # show or hide the horizontal signal limit reminders
        if limit.isChecked():
            self.hline.show()
        else:
            self.hline.hide()

    def tEnable(self):  # show or hide a trace
        if self.guiRef(2).isChecked():  # 2 selects trace checkboxes
            self.trace.show()
        else:
            self.trace.hide()
        checkboxes.dwm.submit()

    def updateTrace(self, frequencies, readings):  # called by sigProcess() for every trace every 20 points
        self.trace.setData((frequencies/1e6), readings)
        if ui.grid.isChecked():
            tinySA.vGrid.show()
        else:
            tinySA.vGrid.hide()
        if not tinySA.sweeping:  # measurement thread is stopping
            ui.scan_button.setText('Stopping ...')
            ui.scan_button.setStyleSheet('background-color: orange')
            ui.run3D.setText('Stopping ...')
            ui.run3D.setStyleSheet('background-color: orange')

    def updateMarker(self, frequencies, readings, fPeaks):  # called by updateGUI()
        options = {'Peak1': fPeaks[0]/1e6, 'Peak2': fPeaks[1]/1e6, 'Peak3': fPeaks[2]/1e6,
                   'Peak4': fPeaks[3]/1e6, 'Normal': self.vline.value(), 'Delta': self.vline.value()}
        markerF = options.get(self.markerType)
        if markerF * 1e6 < np.min(frequencies) or markerF * 1e6 > np.max(frequencies):
            # marker is out of scan range so just show its frequency
            self.vline.label.setText(f'M{self.vline.name()} {self.vline.value():.3f}MHz')
        else:
            # marker is in scan range
            fIndex = np.argmin(np.abs(frequencies - (markerF * 1e6)))  # find closest value in freq array
            dBm = readings[fIndex]
            if dBm > S4.hline.value() or self.markerType[:4] != 'Peak':
                self.vline.setValue(frequencies[fIndex] / 1e6)  # set to the discrete value from frequencies[]
            if self.markerType == 'Delta':
                self.vline.label.setText(f'M{self.vline.name()} {chr(916)}{self.deltaF:.3f}MHz {dBm:.1f}dBm')
            else:
                self.vline.label.setText(f'M{self.vline.name()} {self.vline.value():.3f}MHz {dBm:.1f}dBm')

    def addFreqMarker(self, freq, colour, name):  # adds simple frequency marker without full marker capability
        if ui.presetLabel.isChecked():
            marker = ui.graphWidget.addLine(freq, 90, pen=pyqtgraph.mkPen(colour, width=0.5, style=QtCore.Qt.DashLine),
                                            label=name, labelOpts={'position': 0.05, 'color': (colour)})
            marker.label.setMovable(True)
        else:
            marker = ui.graphWidget.addLine(freq, 90, pen=pyqtgraph.mkPen(colour, width=0.5, style=QtCore.Qt.DashLine))

        self.fifo.put(marker)  # store the marker object in a queue

    def delFreqMarkers(self):
        for i in range(0, self.fifo.qsize()):
            ui.graphWidget.removeItem(self.fifo.get())  # remove the marker and its corresponding object in the queue

    def freqOffset(self, frequencies):  # for mixers external to TinySA # future feature
        if preferences.highLO.isChecked() and preferences.freqLO != 0:
            f = preferences.freqLO.value() - frequencies
        else:
            f = preferences.freqLO.value() + frequencies
        return f


class WorkerSignals(QtCore.QObject):
    error = QtCore.pyqtSignal(str)
    result = QtCore.pyqtSignal(np.ndarray, np.ndarray)
    fullSweep = QtCore.pyqtSignal(np.ndarray, np.ndarray)
    finished = QtCore.pyqtSignal()


class Worker(QtCore.QRunnable):
    '''Worker threads so that functions can run outside GUI event loop'''

    def __init__(self, fn, *args):
        super(Worker, self).__init__()
        self.fn = fn
        self.args = args
        self.signals = WorkerSignals()

    @QtCore.pyqtSlot()
    def run(self):
        '''Initialise the runner'''
        logging.info(f'{self.fn.__name__} thread running')
        self.fn(*self.args)
        logging.info(f'{self.fn.__name__} thread stopped')


class database():
    '''configuration data is stored in a SQLite database'''

    def __init__(self):
        self.db = None
        self.dbName = "QtTSAprefs.db"
        self.personalDir = platformdirs.user_config_dir(appname=app.applicationName(), ensure_exists=True)
        self.globalDir = platformdirs.site_config_dir(appname=app.applicationName())
        self.workingDirs = [os.path.dirname(__file__), os.path.dirname(os.path.realpath(__file__)), os.getcwd()]
        self.dbpath = self._getPersonalisedPath()

    def _getPersonalisedPath(self):
        returnpath = None
        # check if config database file exists in ~/.config/qttinysa/
        if os.path.exists(os.path.join(self.personalDir, self.dbName)):
            returnpath = self.personalDir
            logging.info(f'Personal configuration database found at {self.personalDir}')
        elif os.path.exists(os.path.join(self.globalDir, self.dbName)):
            logging.info(f'Personal configuration database not found in {self.personalDir}')
            c = shutil.copy(os.path.join(self.globalDir, self.dbName), self.personalDir)
            if os.path.exists(os.path.join(self.personalDir, self.dbName)):
                returnpath = self.personalDir
                logging.info(f'Global configuration database copied to {c}')
                popUp(f'Personal configuration database created at \n{c}', QMessageBox.Ok, QMessageBox.Information)
        if returnpath is None:
            # no config database file found in personal or global directories
            logging.info(f'No configuration database file exists in {self.personalDir} or {self.globalDir}')
            # Look for one in the current working folder and in the folder where the python file is stored (or linked to):
            # In case QtTinySA is called from outside the stored folder.
            for workingDir in self.workingDirs:
                if os.path.exists(os.path.join(workingDir, self.dbName)):
                    logging.info(f'Copying configuration database from {self.workingDir}')
                    c = shutil.copy(os.path.join(self.workingDir, self.dbName), self.personalDir)
            if os.path.exists(os.path.join(self.personalDir, self.dbName)):
                returnpath = self.personalDir
                logging.info(f'Personal configuration database created at {c}')
                popUp(f'Personal configuration database created at \n{c}', QMessageBox.Ok, QMessageBox.Information)
            else:
                raise FileNotFoundError("Unable to create a personal configuration database")
        return returnpath

    def connect(self):
        self.db = QSqlDatabase.addDatabase('QSQLITE')
        if QtCore.QFile.exists(os.path.join(self.dbpath, self.dbName)):
            self.db.setDatabaseName(os.path.join(self.dbpath, self.dbName))
            self.db.open()
            logging.info(f'Database open: {self.db.isOpen()}')
            self.db.exec('PRAGMA foreign_keys = ON')  # exec() is deprecated #
        else:
            logging.info('Database file is missing')
            popUp('Database file is missing', QMessageBox.Ok, QMessageBox.Critical)

    def disconnect(self):
        self.db.close()
        logging.info(f'Database open: {self.db.isOpen()}')
        QSqlDatabase.removeDatabase(QSqlDatabase.database().connectionName())


class modelView():
    '''set up and process data models bound to the GUI widgets'''

    def __init__(self, tableName):
        self.tableName = tableName
        self.tm = QSqlRelationalTableModel()
        self.dwm = QDataWidgetMapper()
        self.currentRow = 0

    def createTableModel(self):
        # add exception handling?
        self.tm.setTable(self.tableName)
        self.dwm.setModel(self.tm)
        self.dwm.setSubmitPolicy(QDataWidgetMapper.AutoSubmit)

    def addRow(self):  # adds a blank row to the frequency bands table widget
        self.tm.insertRow(self.currentRow + 1)
        self.currentRow += 1
        preferences.freqBands.selectRow(self.currentRow)

    def saveChanges(self):
        self.dwm.submit()

    def deleteRow(self):  # deletes row selected by the up/down arrows on the frequency bands table widget
        self.tm.removeRow(self.currentRow)
        self.tm.select()
        self.tm.layoutChanged.emit()
        self.dwm.submit()

    def tableClicked(self):
        self.currentRow = preferences.freqBands.currentIndex().row()  # the row index from the QModelIndexObject
        logging.debug(f'row {self.currentRow} clicked')

    def insertData(self, name, typeF, startF, stopF, colour):
        record = self.tm.record()
        record.setValue('name', name)
        record.setValue('startF', f'{startF:.6f}')
        record.setValue('stopF', f'{stopF:.6f}')
        bandstype.tm.setFilter('preset = "' + typeF + '"')  # using relation directly doesn't seem to work
        bandstype.tm.select()
        record.setValue('preset', bandstype.tm.record(0).value('ID'))
        colours.tm.setFilter('colour = "' + colour + '"')  # using relation directly doesn't seem to work
        colours.tm.select()
        record.setValue('colour', colours.tm.record(0).value('ID'))
        self.tm.insertRecord(-1, record)
        self.tm.select()
        self.tm.layoutChanged.emit()
        self.dwm.submit()

    def filterType(self, prefsDialog):
        if prefsDialog:
            if preferences.filterBox.currentText() == 'show all':
                bands.tm.setFilter('')
            else:
                bands.tm.setFilter('preset = "' + preferences.filterBox.currentText() + '"')  # SQL syntax
        else:
            if preferences.filterBox.currentText() == 'show all':
                bands.tm.setFilter('visible = "1"')
            else:
                bands.tm.setFilter('visible = "1" AND preset = "' + preferences.filterBox.currentText() + '"')

        if tinySA.tinySA4 is False:  # It's a tinySA basic with limited frequency range
            bands.tm.setFilter('preset = "' + preferences.filterBox.currentText() + '" AND startF <= "960"')

    def readCSV(self, fileName):
        with open(fileName, "r") as fileInput:
            header = None
            for row in csv.reader(fileInput):
                if not header:
                    header = row
                    logging.debug(f'header = {header}')
                    indx = self.findCols(header)
                    continue
                if len(indx) == 6:  # (name, preset(=type), startF, stopF, value(=visible), colour)
                    bands.insertData(row[indx[0]], row[indx[1]], float(row[indx[2]]), float(row[indx[3]]), row[indx[5]])
                elif len(indx) == 3:
                    bands.insertData(row[indx[0]], 'RF mic', float(row[indx[1]]) / 1e3, 0, row[indx[2]].lower())
                    logging.debug(f'colour = {row[indx[2]].lower()}')

    def writeCSV(self, fileName):
        header = []
        for i in range(1, self.tm.columnCount()):
            header.append(self.tm.record().fieldName(i))
        with open(fileName, "w") as fileOutput:
            output = csv.writer(fileOutput)
            output.writerow(header)
            for rowNumber in range(self.tm.rowCount()):
                fields = [self.tm.data(self.tm.index(rowNumber, columnNumber))
                          for columnNumber in range(1, 7)]
                output.writerow(fields)

    def findCols(self, header):
        indx = []
        try:
            for i in range(1, self.tm.columnCount()):  # start at 1 - don't include ID column
                indx.append(header.index(self.tm.record().fieldName(i)))  # Match to QtTinySA CSV export format
                logging.debug(f'i = {i} indx = {indx}')
        except ValueError:
            indx = []
            try:
                indx.append(header.index('Name'))  # Match to RF mic export format
                indx.append(header.index('Frequency'))
                indx.append(header.index('Colour'))
            except ValueError:
                popUp('CSV Import failed: Field headers were not identified', QMessageBox.Ok, QMessageBox.Critical)
                return
        return indx

###############################################################################
# respond to GUI signals


def band_changed():
    index = ui.band_box.currentIndex()
    if bands.tm.record(index).value('preset') == 'band':
        startF = bands.tm.record(index).value('StartF')
        stopF = bands.tm.record(index).value('StopF')
        ui.start_freq.setValue(startF)
        ui.stop_freq.setValue(stopF)
        tinySA.freq_changed(False)  # start/stop mode
    else:
        centreF = bands.tm.record(index).value('StartF')
        ui.centre_freq.setValue(centreF)
        ui.span_freq.setValue(1)
        tinySA.freq_changed(True)  # centre mode


def addBandPressed():
    if ui.marker1.isChecked() and ui.marker2.isChecked():
        if S1.vline.value() >= S2.vline.value():
            message = 'M1 frequency >= M2 frequency'
            popUp(message, QMessageBox.Ok, QMessageBox.Information)
            return
        name = 'M' + str(round(S1.vline.value(), 6))
        bands.insertData(name, S1.vline.value(), S2.vline.value(), '')
    else:
        message = 'M1 and M2 must both be enabled to add a new Band'
        popUp(message, QMessageBox.Ok, QMessageBox.Information)


def attenuate_changed():
    atten = ui.atten_box.value()
    if ui.atten_auto.isChecked():
        atten = 'auto'
        ui.atten_box.setEnabled(False)
    else:
        if not ui.lna_box.isChecked():  # attenuator and lna are switched so mutually exclusive
            ui.atten_box.setEnabled(True)
    command = f'attenuate {str(atten)}\r'.encode()
    tinySA.fifo.put(command)


def rbwChanged():
    if ui.rbw_box.currentIndex() == 0:  # can't calculate Points because we don't know what the RBW will be
        ui.points_auto.setChecked(False)
        ui.points_auto.setEnabled(False)
    else:
        ui.points_auto.setEnabled(True)
    tinySA.setRBW()  # if measurement thread is running, calling setRBW() will force it to update scan parameters


def pointsChanged():
    if ui.points_auto.isChecked():
        ui.points_box.setEnabled(False)
    else:
        ui.points_box.setEnabled(True)
    tinySA.resume()


def memChanged():
    depth = ui.memSlider.value()
    if depth < ui.avgSlider.value():
        ui.avgSlider.setValue(depth)
    tinySA.scanMemory = depth


def markerToStart():
    S1.mStart()
    S2.mStart()
    S3.mStart()
    S4.mStart()


def markerToCentre():
    S1.mSpread()
    S2.mSpread()
    S3.mSpread()
    S4.mSpread()


def mkr1_moved():
    if S2.markerType != 'Delta' and S3.markerType != 'Delta' and S4.markerType != 'Delta':
        S1.vline.setPen(color='y', width=0.75, style=QtCore.Qt.DashLine)
    else:
        S2.mDelta()
        S3.mDelta()
        S4.mDelta()


def setPreferences():
    checkboxes.dwm.submit()
    numbers.dwm.submit()
    bands.tm.submitAll()
    S4.hline.setValue(preferences.peakThreshold.value())
    bands.filterType(False)
    if ui.presetMarker.isChecked():
        S1.delFreqMarkers()
        S2.delFreqMarkers()
        freqMarkers()


def dialogPrefs():
    bands.filterType(True)
    bands.tm.select()
    bands.currentRow = 0
    preferences.freqBands.selectRow(bands.currentRow)
    pwindow.show()


def about():
    message = ('TinySA Ultra GUI programme using Qt5 and PyQt\nAuthor: Ian Jefferson G4IXT\n\nVersion: {} \nConfig: {}'
               .format(app.applicationVersion(), config.dbpath))
    popUp(message, QMessageBox.Ok, QMessageBox.Information)

##############################################################################
# other methods


def exit_handler():
    if tinySA.dev is not None:
        # save the current displayed marker and trace settings as the default
        S1.dSave(1)
        S2.dSave(1)
        S3.dSave(1)
        S4.dSave(1)
        numbers.dwm.submit()
        checkboxes.dwm.submit()
        # stop sweeping
        if tinySA.sweeping:
            tinySA.sweeping = False  # tell the measurement thread to stop
            while tinySA.threadRunning:
                time.sleep(0.1)  # wait for measurements to stop
        tinySA.resume()
        tinySA.closePort()  # close USB connection
    config.disconnect()  # close database
    logging.info('QtTinySA Closed')


def popUp(message, button, icon):
    # icon can be = QMessageBox.Warning, QMessageBox.Information, QMessageBox.Critical, QMessageBox.Question
    msg = QMessageBox(parent=(window))
    msg.setIcon(icon)
    msg.setText(message)
    msg.setStandardButtons(button)
    msg.exec_()


def freqMarkers():
    if ui.presetMarker.isChecked():
        for i in range(0, bands.tm.rowCount()):
            startF = bands.tm.record(i).value('StartF')
            colour = bands.tm.record(i).value('colour')
            name = bands.tm.record(i).value('name')
            S1.addFreqMarker(startF, colour, name)
            stopF = bands.tm.record(i).value('StopF')
            if bands.tm.record(i).value('preset') == 'band':
                S2.addFreqMarker(stopF, colour, '')
    else:
        S1.delFreqMarkers()
        S2.delFreqMarkers()


def freqMarkerLabel():
    S1.delFreqMarkers()
    S2.delFreqMarkers()
    freqMarkers()


def exportData():
    filename = QFileDialog.getSaveFileName(caption="Save As", filter="Comma Separated Values (*.csv)")
    logging.info(f'filename {filename}')
    if filename[0] != '':
        bands.writeCSV(filename[0])


def importData():
    filename = QFileDialog.getOpenFileName(caption="Select File to Import", filter="Comma Separated Values (*.csv)")
    logging.info(f'filename {filename}')
    if filename[0] != '':
        bands.readCSV(filename[0])


###############################################################################
# Instantiate classes

tinySA = analyser()

app = QtWidgets.QApplication([])  # create QApplication for the GUI
app.setApplicationName('QtTinySA')
app.setApplicationVersion(' v0.10.1')
window = QtWidgets.QMainWindow()
ui = QtTinySpectrum.Ui_MainWindow()
ui.setupUi(window)

pwindow = QtWidgets.QDialog()  # pwindow is the preferences dialogue box
preferences = QtTSApreferences.Ui_Preferences()
preferences.setupUi(pwindow)

# Traces & markers
S1 = display('1', yellow)
S2 = display('2', magenta)
S3 = display('3', cyan)
S4 = display('4', white)

# Data models for configuration settings
config = database()
config.connect()
bands = modelView('frequencies')
checkboxes = modelView('checkboxes')
numbers = modelView('numbers')
markers = modelView('marker')
traces = modelView('trace')
tracetext = modelView('combo')
markertext = modelView('combo')
bandstype = modelView('freqtype')
colours = modelView('SVGColour')


###############################################################################
# GUI settings

# pyqtgraph settings for spectrum display
ui.graphWidget.disableAutoRange()  # supposed to make pyqtgraph plot faster
ui.graphWidget.setYRange(-110, 5)
ui.graphWidget.setXRange(87.5, 108)
ui.graphWidget.setBackground('k')  # black
ui.graphWidget.showGrid(x=True, y=True)

ui.graphWidget.setLabel('left', 'Signal', 'dBm')
ui.graphWidget.setLabel('bottom', 'Frequency MHz')

# marker label positions
S1.vline.label.setPosition(0.99)
S2.vline.label.setPosition(0.95)
S3.vline.label.setPosition(0.90)
S4.vline.label.setPosition(0.85)

# signal limit lines
S1.hline.setValue(-25)
S1.hline.label.setText('best')
S2.hline.label.setText('max')
S3.hline.setValue(6)
S3.hline.setPen('r')

# marker peak threshold line
S4.hline.setPen(red_dash, width=0.5)
S4.hline.setMovable(True)
S4.hline.label.setFormat("{value:.1f}")

###############################################################################
# Connect signals from buttons and sliders.  Connections for freq and rbw boxes are in 'initialise' Fn

ui.scan_button.clicked.connect(tinySA.scan)
ui.run3D.clicked.connect(tinySA.scan)
ui.atten_box.valueChanged.connect(attenuate_changed)
ui.atten_auto.clicked.connect(attenuate_changed)
ui.spur_box.clicked.connect(tinySA.spur)
ui.lna_box.clicked.connect(tinySA.lna)
ui.memSlider.sliderMoved.connect(memChanged)
ui.points_auto.stateChanged.connect(pointsChanged)
ui.points_box.editingFinished.connect(pointsChanged)

# marker dragging
S1.vline.sigPositionChanged.connect(mkr1_moved)
S2.vline.sigPositionChanged.connect(S2.mType)
S3.vline.sigPositionChanged.connect(S3.mType)
S4.vline.sigPositionChanged.connect(S4.mType)

# marker setting within span range
ui.mkr_start.clicked.connect(markerToStart)
ui.mkr_centre.clicked.connect(markerToCentre)

# marker checkboxes
ui.marker1.clicked.connect(S1.mEnable)
ui.marker2.clicked.connect(S2.mEnable)
ui.marker3.clicked.connect(S3.mEnable)
ui.marker4.clicked.connect(S4.mEnable)

# marker type changes
ui.m1_type.activated.connect(S1.mType)
ui.m2_type.activated.connect(S2.mType)
ui.m3_type.activated.connect(S3.mType)
ui.m4_type.activated.connect(S4.mType)

# frequency band markers
ui.presetMarker.stateChanged.connect(freqMarkers)
ui.presetLabel.stateChanged.connect(freqMarkerLabel)
ui.mToBand.clicked.connect(addBandPressed)

# trace checkboxes
ui.trace1.stateChanged.connect(S1.tEnable)
ui.trace2.stateChanged.connect(S2.tEnable)
ui.trace3.stateChanged.connect(S3.tEnable)
ui.trace4.stateChanged.connect(S4.tEnable)

# trace type changes
ui.t1_type.activated.connect(S1.tType)
ui.t2_type.activated.connect(S2.tType)
ui.t3_type.activated.connect(S3.tType)
ui.t4_type.activated.connect(S4.tType)

# 3D graph controls
ui.orbitL.clicked.connect(lambda: tinySA.orbit3D(1, True))
ui.orbitR.clicked.connect(lambda: tinySA.orbit3D(-1, True))
ui.orbitU.clicked.connect(lambda: tinySA.orbit3D(-1, False))
ui.orbitD.clicked.connect(lambda: tinySA.orbit3D(1, False))
ui.timeF.clicked.connect(lambda: tinySA.axes3D(-1, 'X'))
ui.timeR.clicked.connect(lambda: tinySA.axes3D(1, 'X'))
ui.freqR.clicked.connect(lambda: tinySA.axes3D(-1, 'Y'))
ui.freqL.clicked.connect(lambda: tinySA.axes3D(1, 'Y'))
ui.signalUp.clicked.connect(lambda: tinySA.axes3D(-1, 'Z'))
ui.signalDown.clicked.connect(lambda: tinySA.axes3D(1, 'Z'))
ui.gridF.clicked.connect(lambda: tinySA.grid(1))
ui.gridR.clicked.connect(lambda: tinySA.grid(-1))
ui.zoom.sliderMoved.connect(tinySA.zoom3D)
ui.reset3D.clicked.connect(tinySA.reset3D)

# preferences
preferences.neg25Line.stateChanged.connect(lambda: S1.hEnable(preferences.neg25Line))
preferences.zeroLine.stateChanged.connect(lambda: S2.hEnable(preferences.zeroLine))
preferences.plus6Line.stateChanged.connect(lambda: S3.hEnable(preferences.plus6Line))
preferences.addRow.clicked.connect(bands.addRow)
preferences.deleteRow.clicked.connect(bands.deleteRow)
preferences.freqBands.clicked.connect(bands.tableClicked)
preferences.filterBox.currentTextChanged.connect(lambda: bands.filterType(True))
ui.actionPreferences.triggered.connect(dialogPrefs)  # open preferences dialogue when its menu is clicked
ui.actionAbout_QtTinySA.triggered.connect(about)
pwindow.finished.connect(setPreferences)  # update database checkboxes table on dialogue window close
preferences.exportButton.pressed.connect(exportData)
preferences.importButton.pressed.connect(importData)


###############################################################################
# set up the application
logging.info(f'{app.applicationName()}{app.applicationVersion()}')

# table models - read/write views of the configuration data
bands.createTableModel()  # relational
bands.tm.setSort(3, QtCore.Qt.AscendingOrder)
bands.tm.setHeaderData(5, QtCore.Qt.Horizontal, 'visible')

bands.tm.setRelation(5, QSqlRelation('boolean', 'ID', 'value'))  # set 'view' column to a True/False choice combo box
boolean = QSqlRelationalDelegate(preferences.freqBands)
preferences.freqBands.setItemDelegate(boolean)

bands.tm.setRelation(6, QSqlRelation('SVGColour', 'ID', 'colour'))  # set 'marker' column to a colours choice combo box
colour = QSqlRelationalDelegate(preferences.freqBands)
preferences.freqBands.setItemDelegate(colour)

bands.tm.setRelation(2, QSqlRelation('freqtype', 'ID', 'preset'))  # set 'type' column to a freq type choice combo box
fType = QSqlRelationalDelegate(preferences.freqBands)
preferences.freqBands.setItemDelegate(fType)

colHeader = preferences.freqBands.horizontalHeader()
colHeader.setSectionResizeMode(QtWidgets.QHeaderView.ResizeToContents)

#
bandstype.createTableModel()
colours.createTableModel()
colours.tm.select()
#

# populate the presets combobox
ui.band_box.setModel(bands.tm)
ui.band_box.setModelColumn(1)
bands.tm.select()  # initially select the data in the model

# populate the preferences filter combo box
preferences.filterBox.setModel(bandstype.tm)
preferences.filterBox.setModelColumn(1)
bandstype.dwm.addMapping(preferences.filterBox, 1)
bandstype.tm.select()

# connect the preferences dialogue box freq band widget to the data model
preferences.freqBands.setModel(bands.tm)
preferences.freqBands.hideColumn(0)  # ID
rowHeader = preferences.freqBands.verticalHeader()

#  Map database tables to preferences dialogue box fields and to main GUI
#  ** lines need to be in this order and here or the mapping doesn't work **
checkboxes.createTableModel()
checkboxes.dwm.addMapping(preferences.rbw_x, 3)
checkboxes.dwm.addMapping(preferences.neg25Line, 4)
checkboxes.dwm.addMapping(preferences.zeroLine, 5)
checkboxes.dwm.addMapping(preferences.plus6Line, 6)
checkboxes.dwm.addMapping(ui.trace1, 7)
checkboxes.dwm.addMapping(ui.trace2, 8)
checkboxes.dwm.addMapping(ui.trace3, 9)
checkboxes.dwm.addMapping(ui.trace4, 10)
checkboxes.dwm.addMapping(ui.marker1, 11)
checkboxes.dwm.addMapping(ui.marker2, 12)
checkboxes.dwm.addMapping(ui.marker3, 13)
checkboxes.dwm.addMapping(ui.marker4, 14)
checkboxes.dwm.addMapping(ui.lna_box, 15)
checkboxes.dwm.addMapping(ui.points_auto, 16)
checkboxes.tm.select()
checkboxes.dwm.setCurrentIndex(0)  # 0 = (last used) default settings

# The models for saving number, marker and trace settings
numbers.createTableModel()
numbers.dwm.addMapping(preferences.minPoints, 3)
numbers.dwm.addMapping(preferences.maxPoints, 4)
numbers.dwm.addMapping(ui.start_freq, 5)
numbers.dwm.addMapping(ui.stop_freq, 6)
numbers.dwm.addMapping(preferences.peakThreshold, 7)
numbers.tm.select()
numbers.dwm.setCurrentIndex(0)
markers.createTableModel()
traces.createTableModel()
traces.tm.select()

# populate the trace comboboxes
tracetext.createTableModel()
tracetext.tm.setFilter('type = "trace"')
ui.t1_type.setModel(tracetext.tm)
ui.t2_type.setModel(tracetext.tm)
ui.t3_type.setModel(tracetext.tm)
ui.t4_type.setModel(tracetext.tm)
tracetext.tm.select()

# populate the marker comboboxes
markertext.createTableModel()
markertext.tm.setFilter('type = "marker"')
ui.m1_type.setModel(markertext.tm)
ui.m2_type.setModel(markertext.tm)
ui.m3_type.setModel(markertext.tm)
ui.m4_type.setModel(markertext.tm)
markertext.tm.select()

# set GUI fields using values from the configuration database
tinySA.restoreSettings()

# try to open a USB connection to the TinySA hardware
tinySA.openPort()
if tinySA.dev is None:
    tinySA.usbCheck.start(500)  # check again every 500mS

window.show()
window.setWindowTitle(app.applicationName() + app.applicationVersion())
window.setWindowIcon(QtGui.QIcon(os.path.join(basedir, 'tinySAsmall.png')))

###############################################################################
# run the application until the user closes it

try:
    app.exec()
finally:
    exit_handler()  # close cleanly
