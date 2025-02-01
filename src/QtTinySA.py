#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright 2025 Ian Jefferson G4IXT
# SPDX-License-Identifier: GPL-3.0-or-later

# Compilation mode, support OS-specific options
# nuitka-project-if: {OS} in ("Windows", "Linux", "Darwin", "FreeBSD"):
#    nuitka-project: --onefile
#    nuitka-project: --enable-plugin=pyqt5
#    nuitka-project: --include-qt-plugins=sqldrivers
#    nuitka-project: --include-data-file=./QtTSAprefs.db=./
# nuitka-project-else:
#    nuitka-project: --standalone

"""TinySA Ultra GUI programme using Qt5 and PyQt.

This code attempts to replicate some of the TinySA Ultra on-screen commands and to provide PC control.
Development took place on Kubuntu 24.04LTS with Python 3.11 and PyQt5 using Spyder in Anaconda.

TinySA, TinySA Ultra and the tinysa icon are trademarks of Erik Kaashoek and are used with permission.

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
from PyQt5 import QtWidgets, QtCore
from PyQt5.QtWidgets import QMessageBox, QDataWidgetMapper, QFileDialog, QInputDialog, QLineEdit
from PyQt5.QtSql import QSqlDatabase, QSqlRelation, QSqlRelationalTableModel, QSqlRelationalDelegate
from PyQt5.QtGui import QPixmap, QIcon
from datetime import datetime
import pyqtgraph
import QtTinySpectrum  # the main GUI
import QtTSApreferences  # the preferences GUI
import QtTSAfilebrowse  # the tinySA SD card browser GUI
import struct
import serial
from serial.tools import list_ports
from io import BytesIO

#  For 3D
import pyqtgraph.opengl as pyqtgl

# Defaults to non local configuration/data dirs - needed for packaging
if system() == "Linux":
    os.environ['XDG_CONFIG_DIRS'] = '/etc:/usr/local/etc'
    os.environ['XDG_DATA_DIRS'] = '/usr/share:/usr/local/share'
# Fix 3D Spectrum Rendering not working on Windows using DirectX by default
elif system() == "Windows":
    # force Qt to use OpenGL rather than DirectX for Windows OS
    QtCore.QCoreApplication.setAttribute(QtCore.Qt.AA_UseDesktopOpenGL)

logging.basicConfig(format="%(message)s", level=logging.INFO)
threadpool = QtCore.QThreadPool()
basedir = os.path.dirname(__file__)

# pyqtgraph pens
red_dash = pyqtgraph.mkPen(color='r', width=0.5, style=QtCore.Qt.DashLine)
blue_dash = pyqtgraph.mkPen(color='b', width=0.5,  style=QtCore.Qt.DashLine)


###############################################################################
# classes


class analyser:
    def __init__(self):
        self.usb = None
        self.surface = None
        self.vGrid = None
        self.tinySA4 = None
        self.directory = None
        self.firmware = None
        self.sweeping = False
        self.threadRunning = False
        self.signals = WorkerSignals()
        self.signals.result.connect(self.updateGUI)
        self.signals.finished.connect(self.threadEnds)
        self.signals.saveResults.connect(saveFile)
        self.signals.resetGUI.connect(self.resetGUI)
        self.runTimer = QtCore.QElapsedTimer()
        self.scale = 174
        self.scanMemory = 50
        self.usbCheck = QtCore.QTimer()
        self.usbCheck.timeout.connect(self.isConnected)
        self.fifo = queue.SimpleQueue()
        self.fifoTimer = QtCore.QTimer()
        self.fifoTimer.timeout.connect(self.usbSend)
        self.maxF = 6000
        self.memF = BytesIO()
        self.ports = []

    def openPort(self):  # called by isConnected() triggered by the self.usbCheck QTimer at startup
        # Get tinySA comport using hardware ID
        VID = 0x0483  # 1155
        PID = 0x5740  # 22336
        usbPorts = list_ports.comports()
        for port in usbPorts:
            if port.vid == VID and port.pid == PID:
                if port not in self.ports:
                    preferences.deviceBox.addItem(self.identify(port) + " on " + port.device)
                    self.ports.append(port)
        if len(self.ports) == 1:  # found only one device so just test it
            self.usbCheck.stop()
            self.testPort(self.ports[0])
            return
        if len(self.ports) > 1:  # several devices found
            preferences.deviceBox.insertItem(0, "Select device")
            preferences.deviceBox.setCurrentIndex(0)
            popUp("Several devices detected.  Choose device in Settings > Preferences",
                  QMessageBox.Ok, QMessageBox.Information)
            self.usbCheck.stop()

    def testPort(self, port):  # tests comms and initialises tinySA if found
        try:
            self.usb = serial.Serial(port.device, baudrate=576000)
            logging.info(f'Serial port {port.device} open: {self.usb.isOpen()}')
        except serial.SerialException:
            logging.info('Serial port exception. A possible cause is that your username is not in the "dialout" group.')
            popUp('Serial port exception',
                  QMessageBox.Ok, QMessageBox.Critical)
        if self.usb:
            for i in range(4):  # try 3 times to communicate with tinySA over USB serial
                firmware = self.version()
                if firmware[:6] == 'tinySA':
                    logging.info(f'{port.device} test {i} : {firmware[:16]}')
                    break
                else:
                    time.sleep(1)
            # split firmware into a list of [device, major version number, minor version number, other stuff]
            self.firmware = firmware.replace('_', '-').split('-')
            if firmware[:6] == 'tinySA':
                if float(self.firmware[1][-3:] + self.firmware[2]) < 1.4177:
                    logging.info('for fastest possible scan speed, upgrade firmware to v1.4-177 or later')
                if self.firmware[0] in ('tinySA4',  'tinySA_') and self.firmware[1][0] == "v":
                    self.initialise(self.firmware)
                if self.firmware[1][0] != "v":
                    logging.info(f'{port.device} test found firmware {firmware}. Expected to find tinySA_vn.n-nnn')
            else:
                logging.info(f'firmware {firmware} for {self.identify(port)} on {port.device} is not a tinySA')

    def identify(self, port):
        # Windows returns no information to pySerial list_ports.comports()
        if system() == 'Linux' or system() == 'Darwin':
            return port.product
        else:
            return 'USB device'

    def closePort(self):
        if self.usb:
            self.usb.close()
            logging.info(f'Serial port open: {self.usb.isOpen()}')
            self.usb = None

    def isConnected(self):
        # triggered by self.usbCheck QTimer - if tinySA wasn't found checks repeatedly for device, i.e.'hotplug'
        if len(self.ports) == 0:
            self.openPort()
        else:
            for i in range(len(self.ports)):
                if self.identify(self.ports[i])[:6] in ('tinySA', 'USB de'):
                    self.usbCheck.stop()
                else:
                    self.openPort()

    def initialise(self, product):
        # product = 'tinySA'  # used for testing
        logging.debug('initialise: started')
        if product[0] == 'tinySA4':  # It's an Ultra
            self.tinySA4 = True
            self.maxF = preferences.maxFreqBox.value()
            self.scale = 174
            ui.spur_box.setTristate(True)  # TinySA Ultra has 'auto', 'on' and 'off' setting for Spur
            ui.spur_box.setCheckState(checkboxes.tm.record(0).value("spur"))
        else:
            self.tinySA4 = False  # It's a Basic
            self.maxF = 960
            self.scale = 128
            rbwtext.tm.setFilter('type = "rbw" and value != "0.2" and value != "1" and value != "850"')  # fewer RBWs
            ui.spur_box.setTristate(False)  # TinySA Basic has only 'on' and 'off' setting for Spur'
            ui.spur_box.setChecked(True)  # on
        self.spur()

        # Basic has no lna
        ui.lna_box.setVisible(self.tinySA4)
        ui.lna_box.setEnabled(self.tinySA4)
        self.lna()

        # show device information in GUI
        ui.battery.setText(self.battery())
        ui.version.setText(product[0] + " " + product[1] + " " + product[2])

        self.setTime()
        self.setAbort(True)
        self.fifoTimer.start(500)  # calls self.usbSend() every 500mS to execute serial commands whilst not scanning

        # Traces
        T1.setup()
        T2.setup()
        T3.setup()
        T4.setup()

        # Markers
        M1.setLevel(ui.m1track.value())
        M2.setLevel(ui.m2track.value())
        M3.setLevel(ui.m3track.value())
        M4.setLevel(ui.m4track.value())
        M1.setup('yellow', 'm1f')
        M2.setup('yellow', 'm2f')
        M3.setup('yellow', 'm3f')
        M4.setup('yellow', 'm4f')

        # set various defaults
        ui.waterfallSize.setValue(ui.waterfallSize.value() + 1)  # sets the height of the waterfall widget
        pointsChanged()
        setPreferences()
        band = ui.band_box.currentText()
        bandselect.filterType(False, ui.filterBox.currentText())  # setting the filter overwrites the band

        # connect GUI controls that send messages to tinySA
        connectActive()

        # restore the band
        ui.band_box.setCurrentText(band)
        logging.debug('initialise: finished')

    def scan(self):  # called by 'run' button
        logging.debug(f'scan: self.usb = {self.usb}')
        if self.usb is not None:
            if self.sweeping:  # if it's running, stop it
                self.sweeping = False  # tells the measurement thread to stop once current scan complete
                logging.debug('scan: stop measurement thread')
                ui.scan_button.setEnabled(False)  # prevent repeat presses of 'stop'
                ui.run3D.setEnabled(False)
            else:
                try:  # start measurements
                    self.fifoTimer.stop()
                    self.clearBuffer()
                    self.setRBW()
                    # self.sampleRep()  # doesn't work with scanraw
                    self.runButton('Stop')
                    self.usbSend()
                    self.startMeasurement()  # runs measurement in separate thread
                except serial.SerialException:
                    logging.info('serial port exception')
                    self.ports = []
                    self.closePort()
        else:
            popUp('TinySA not found', QMessageBox.Ok, QMessageBox.Critical)

    def startMeasurement(self):
        frequencies, readings, maxima, minima = self.set_arrays()
        self.sweep = Worker(self.measurement, frequencies, readings, maxima, minima)  # workers deleted when thread ends
        self.sweeping = True
        self.createTimeSpectrum(frequencies, readings)
        self.createWaterfall(frequencies, readings)
        self.reset3D()
        threadpool.start(self.sweep)

    def usbSend(self):
        try:
            self.usb.timeout = 1
        except (serial.SerialException, AttributeError):
            self.usbCheck.start()
            return
        while self.fifo.qsize() > 0:
            command = self.fifo.get(block=True, timeout=None)
            logging.debug(command)
            self.serialWrite(command)

    def serialQuery(self, command):
        self.usb.write(command.encode())
        self.usb.read_until(command.encode() + b'\n')  # skip command echo
        response = self.usb.read_until(b'ch> ')  # until prompt
        logging.debug(response)
        return response[:-6].decode()  # remove prompt

    def serialWrite(self, command):
        self.usb.timeout = 1
        logging.debug(command)
        self.usb.write(command.encode())
        self.usb.read_until(b'ch> ')  # skip command echo and prompt

    def set_arrays(self):
        startF = ui.start_freq.value() * 1e6  # freq in Hz
        stopF = ui.stop_freq.value() * 1e6
        points = self.setPoints()
        maxima = np.full(points, -200, dtype=float)
        minima = np.full(points, 0, dtype=float)
        frequencies = np.linspace(startF, stopF, points, dtype=np.int64)
        # logging.info(f'set_arrays: frequencies = {frequencies}')
        readings = np.full((self.scanMemory, points), None, dtype=float)
        readings[0] = -200
        return frequencies, readings, maxima, minima

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
        ui.graphWidget.setXRange(startF * 1e6, stopF * 1e6)
        if ui.span_freq.value() != 0:
            lowF.line.setValue((startF + ui.span_freq.value()/20) * 1e6)
            highF.line.setValue((stopF - ui.span_freq.value()/20) * 1e6)
        self.resume()  # puts a message in the fifo buffer so the measurement thread spots it and updates its settings

    def freqOffset(self, frequencies):  # for mixers or LNBs external to TinySA
        startF = frequencies[0]
        spanF = frequencies[-1] - startF
        loF = preferences.freqLO.value() * 1e6
        if preferences.highLO.isChecked() and preferences.freqLO != 0:
            scanF = (loF - startF - spanF, loF - startF)
        else:
            scanF = (startF - loF, startF - loF + spanF)
        if min(scanF) < 0:
            self.sweeping = False
            scanF = (88 * 1e6, 108 * 1e6)
            logging.info('LO frequency offset error, check preferences')
            popUp("LO frequency offset error, check preferences", QMessageBox.Ok, QMessageBox.Critical)
        logging.debug(f'freqOffset(): scanF = {scanF}')
        return scanF

    def setRBW(self):  # may be called by measurement thread as well as normally
        rbw = ui.rbw_box.currentText()  # ui values are discrete ones in kHz
        logging.debug(f'rbw = {rbw}')
        command = f'rbw {rbw}\r'
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
        if ui.rbw_auto.isChecked():
            # rbw auto setting from tinySA: ~7 kHz per 1 MHz scan frequency span
            rbw = (stopF - startF) * 7e-6
        else:
            rbw = float(ui.rbw_box.currentText())
        rbw = np.clip(rbw, 0.2, 850)  # apply limits
        # timeout can be very long - use a heuristic approach
        # 1st summand is the scanning time, 2nd summand is the USB transfer overhead
        timeout = ((stopF - startF) / 20e3) / (rbw ** 2) + points / 500
        if (ui.spur_box.checkState() == 1 and stopF > 8 * 1e8) or ui.spur_box.checkState() == 2:
            timeout *= 2  # scan time doubles with spur on or spur auto above 800 MHz
        # transfer is done in blocks of 20 points, this is the timeout for one block
        timeout = timeout * 20 / points + 1  # minimum is 1 second
        logging.debug(f'sweepTimeout = {timeout:.2f} s')
        return timeout

    def measurement(self, frequencies, readings, maxima, minima):  # runs in a separate thread
        sweepCount = 0
        updateTimer = QtCore.QElapsedTimer()
        points = np.size(frequencies)
        self.threadRunning = True
        firstRun = True
        version = int(self.firmware[2])  # just the firmware version number
        # self.runTimer.start()  # debug
        # logging.debug(f'elapsed time = {self.runTimer.nsecsElapsed()/1e6:.3f}mS')  # debug
        updateTimer.start()  # used to trigger the signal that sends measurements to updateGUI()

        while self.sweeping:
            if preferences.freqLO != 0:
                startF, stopF = self.freqOffset(frequencies)
                command = f'scanraw {int(startF)} {int(stopF)} {int(points)} 3\r'
            else:
                command = f'scanraw {int(frequencies[0])} {int(frequencies[-1])} {int(points)} 3\r'
            self.usb.timeout = self.sweepTimeout(frequencies)

            # firmware versions before 4.177 don't support auto-repeating scanraw so command must be sent each sweep
            if version < 177 or firstRun:
                try:
                    self.usb.write(command.encode())
                    self.usb.read_until(command.encode() + b'\n{')  # skip command echo
                    dataBlock = ''
                except serial.SerialException:
                    logging.info('serial port exception')
                    self.sweeping = False
                    break

            # read the measurement data from the tinySA
            for point in range(points):
                dataBlock = (self.usb.read(3))  # read a block of 3 bytes of data
                logging.debug(f'dataBlock: {dataBlock}\n')
                if dataBlock == b'}':  # from FW165 jog button press returns different value
                    logging.info('screen touched or jog button pressed')
                    self.sweeping = False
                    break
                try:
                    c, data = struct.unpack('<' + 'cH', dataBlock)
                except struct.error:
                    logging.info('data error')
                    self.sweeping = False
                    break
                readings[0, point] = (data / 32) - self.scale  # scale 0..4095 -> -128..-0.03 dBm

                # If it's the final point of this sweep, set up for the next sweep
                if point == points - 1:
                    readingsMax = np.nanmax(readings[:self.scanMemory], axis=0)
                    readingsMin = np.nanmin(readings[:self.scanMemory], axis=0)
                    maxima = np.fmax(maxima, readingsMax)
                    minima = np.fmin(minima, readingsMin)
                    readings[-1] = readings[0]  # populate last row with current sweep before rolling
                    readings = np.roll(readings, 1, axis=0)  # readings row 0 is now full: roll it down 1 row
                    if version >= 177:
                        if self.usb.read(2) != b'}{':  # the end of scan marker character is '}{'
                            logging.info('QtTinySA display is out of sync with tinySA frequency')
                            self.sweeping = False
                            break
                        sweepCount += 1
                        firstRun = False
                        if sweepCount == self.scanMemory:  # array is full so trigger CSV data file save
                            self.signals.saveResults.emit(frequencies, readings)
                            sweepCount = 0

                # If a sweep setting has been changed by the user, the sweep must be re-started (+ new recording start)
                if self.fifo.qsize() > 0 or not self.sweeping:
                    self.serialWrite('abort\r')
                    self.clearBuffer()
                    firstRun = True
                    self.setRBW()  # reads GUI rbw box value
                    frequencies, readings, maxima, minima = self.set_arrays()  # reads GUI values
                    points = np.size(frequencies)
                    self.signals.resetGUI.emit(frequencies, readings)
                    self.usbSend()  # send all the queued commands in the FIFO buffer to the TinySA
                    updateTimer.start()
                    break

                timeElapsed = updateTimer.nsecsElapsed()  # how long this batch of measurements has been running, nS

                # Send the sesults to updateGUI if an update is due
                if timeElapsed/1e6 > preferences.intervalBox.value():
                    self.signals.result.emit(frequencies, readings, maxima, minima, timeElapsed)  # send to updateGUI()
                    updateTimer.start()

        self.usb.read(2)  # discard the command prompt that the timySA sends when sweeping ends
        self.threadRunning = False
        self.signals.finished.emit()

    def threadEnds(self):
        if int(self.firmware[2]) >= 177:  # the firmware version number
            self.serialWrite('abort\r')
        self.runButton('Run')
        self.fifoTimer.start(500)  # start watching for commands

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

    def createWaterfall(self, frequencies, readings):
        self.waterfall = pyqtgraph.ImageItem(axisOrder='row-major')
        self.waterfall.setAutoDownsample(True)
        ui.waterfall.addItem(self.waterfall)
        ui.waterfall.setYRange(0, ui.memBox.value())

        # Histogram associated with waterfall
        self.histogram = pyqtgraph.HistogramLUTItem(gradientPosition='right', orientation='vertical')
        self.histogram.setImageItem(self.waterfall)  # connects the histogram to the waterfall
        self.histogram.gradient.loadPreset('viridis')  # set the colour map
        self.waterfall.setLevels((-110, -20))  # needs to be here, after histogram is created
        ui.histogram.addItem(self.histogram)

    def updateWaterfall(self, readings):
        self.waterfall.setImage(readings, autoLevels=False)
        # sigma = np.std(readings)
        # mean = np.mean(readings)
        # readings_min = mean - 2*sigma  # save to window state
        # readings_max = mean + 2*sigma
        # self.waterfall.setLevels((readings_min, readings_max))
        # self.histogram.setLevels((readings_min, readings_max))

    def resetGUI(self, frequencies, readings):
        self.waterfall.clear()
        ui.waterfall.setXRange(np.size(readings, axis=1), 0)
        self.updateWaterfall(readings)
        self.createTimeSpectrum(frequencies, readings)

    def updateGUI(self, frequencies, readings, maxima, minima, runtime):  # called by signal from measurement() thread
        # for LNB/Mixer mode when LO is above measured freq the scan is reversed, i.e. low TinySA freq = high meas freq
        if preferences.highLO.isChecked() and preferences.freqLO != 0:
            frequencies = frequencies[::-1]  # reverse the array
            np.fliplr(readings)
            ui.waterfall.invertX(True)
        else:
            ui.waterfall.invertX(False)

        # update graph axes if in zero span
        if frequencies[0] == frequencies[-1]:
            ui.graphWidget.setLabel('bottom', 'Time')
            frequencies = np.arange(1, len(frequencies) + 1, dtype=int)
            ui.graphWidget.setXRange(frequencies[0], frequencies[-1])

        # update the swept traces
        readingsAvg = np.nanmean(readings[0:ui.avgBox.value()], axis=0)
        options = {'Normal': readings[0], 'Average': readingsAvg, 'Max': maxima, 'Min': minima}
        T1.trace.setData(frequencies, options.get(T1.traceType))
        T2.trace.setData(frequencies, options.get(T2.traceType))
        T3.trace.setData(frequencies, options.get(T3.traceType))
        T4.trace.setData(frequencies, options.get(T4.traceType))

        if ui.waterfallSize.value() != 0:
            self.updateWaterfall(readings)

        # update markers (if not in zero span, where they are not relevant)  # being called too often?
        if frequencies[0] != frequencies[-1]:
            ui.graphWidget.setLabel('bottom', units='Hz')
            M1.updateMarker(frequencies, readings, maxima, minima)
            M2.updateMarker(frequencies, readings, maxima, minima)
            M3.updateMarker(frequencies, readings, maxima, minima)
            M4.updateMarker(frequencies, readings, maxima, minima)

        # update 3D graph if enabled
        if ui.stackedWidget.currentWidget() == ui.View3D:
            z = readings + 120  # Surface plot height shader needs positive numbers so convert from dBm to dBf
            logging.debug(f'z = {z}')
            self.surface.setData(z=z)  # update 3D graph
            params = ui.openGLWidget.cameraParams()
            logging.debug(f'camera {params}')
        if ui.grid.isChecked():
            tinySA.vGrid.show()
        else:
            tinySA.vGrid.hide()

        # other updates
        if ui.points_auto.isChecked():
            ui.points_box.setValue(np.size(frequencies))

        ui.updates.setText(str(int(1/(runtime/1e9))))  # the display update frequency indicator

        if not tinySA.sweeping:  # measurement thread is stopping
            ui.scan_button.setText('Stopping ...')
            ui.scan_button.setStyleSheet('background-color: orange')
            ui.run3D.setText('Stopping ...')
            ui.run3D.setStyleSheet('background-color: orange')

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
            ui.scan_button.setStyleSheet('background-color: ')
            ui.run3D.setStyleSheet('background-color: ')
            ui.scan_button.setEnabled(True)
            ui.run3D.setEnabled(True)

    def pause(self):
        self.fifo.put('pause\r')

    def resume(self):
        self.fifo.put('resume\r')

    def reset(self):
        self.fifo.put('reset\r')

    def battery(self):
        vbat = self.serialQuery('vbat\r')
        return vbat

    def setAbort(self, on=True):
        if on:
            command = 'abort on\r'
        else:
            command = 'abort off\r'
        self.fifo.put(command)

    def abort(self):
        self.serialWrite('abort\r')
        self.clearBuffer()

    def version(self):
        version = self.serialQuery('version\r')
        return version

    def spur(self):
        sType = ui.spur_box.checkState()
        options = {0: 'Spur Off', 1: 'Spur Auto', 2: 'Spur On'}
        ui.spur_label.setText(options.get(sType))
        options = {0: 'spur off\r', 1: 'spur auto\r', 2: 'spur on\r'}
        command = options.get(sType)
        self.fifo.put(command)

    def lna(self):
        if ui.lna_box.isChecked():
            command = 'lna on\r'
            ui.atten_auto.setEnabled(False)  # attenuator and lna are switched so mutually exclusive
            ui.atten_auto.setChecked(False)
            ui.atten_box.setEnabled(False)
            ui.atten_box.setValue(0)
            self.fifo.put('attenuate 0\r')
        else:
            command = 'lna off\r'
            ui.atten_auto.setEnabled(True)
            ui.atten_auto.setChecked(True)
            self.fifo.put('attenuate auto\r')
        self.fifo.put(command)

    def setTime(self):
        if self.tinySA4 and preferences.syncTime.isChecked():
            dt = datetime.now()
            y = dt.year - 2000
            command = f'time b 0x{y}{dt.month:02d}{dt.day:02d} 0x{dt.hour:02d}{dt.minute:02d}{dt.second:02d}\r'
            self.fifo.put(command)

    def example(self):
        self.fifo.put('example\r')

    def setSweep(self, start, stop):  # only used to set a default on the tinySA
        if start is not None:
            self.serialWrite("sweep start %d\r" % start)
        if stop is not None:
            self.serialWrite("sweep stop %d\r" % stop)

    def sampleRep(self):
        # sets the number of repeat measurements at each frequency point to the value in the GUI
        command = f'repeat {ui.sampleRepeat.value()}\r'
        self.fifo.put(command)

    def fPrecision(self, frequencies):  # sets the marker indicated frequency precision
        fInc = frequencies[1] - frequencies[0]
        if fInc > 0:
            self.dp = np.clip(int(np.log10(frequencies[0] / fInc)), 0, 5)  # number of decicimal places required
            logging.info(f'fPrecision: fInc = {fInc} dp = {self.dp}')
        else:
            self.dp = 6

    def listSD(self):
        if self.usb:
            self.clearBuffer()  # clear the USB serial buffer
            ls = self.serialQuery('sd_list\r')
            return ls

    def readSD(self, fileName):
        command = ('sd_read %s\r' % fileName)
        self.usb.write(command.encode())
        self.usb.readline()  # discard empty line
        format_string = "<1i"  # little-endian single integer of 4 bytes
        self.usb.timeout = None
        buffer = self.usb.read(4)
        size = struct.unpack(format_string, buffer)
        size = size[0]
        data = self.usb.read(size)
        self.usb.timeout = 1
        return data

    def dialogBrowse(self):
        if self.threadRunning:
            popUp("Cannot browse tinySA whilst a scan is running", QMessageBox.Ok, QMessageBox.Information)
            return
        elif self.usb:
            SD = self.listSD()
            filebrowse.listWidget.clear()
            ls = []
            for i in range(len(SD.splitlines())):
                ls.append(SD.splitlines()[i].split(" ")[0])
            filebrowse.listWidget.insertItems(0, ls)
            fwindow.show()
        else:
            popUp('TinySA not found', QMessageBox.Ok, QMessageBox.Critical)

    def fileDownload(self):
        selected = filebrowse.listWidget.currentItem().text()  # the file selected in the list widget
        if self.directory:  # already saved a file so use the same folder path as the default
            folder = os.path.join(self.directory, selected)
            fileName = QFileDialog.getSaveFileName(caption="Save As", directory=folder)
        else:
            fileName = QFileDialog.getSaveFileName(caption="Save As", directory=selected)
        with open(str(fileName[0]), "wb") as file:
            file.write(self.memF.getvalue())
        self.directory = os.path.dirname(fileName[0])
        filebrowse.downloadInfo.setText(self.directory)  # show the path where the file was saved

    def fileShow(self):
        self.memF.seek(0, 0)  # set the memory buffer pointer to the start
        self.memF.truncate()  # clear down the memory buffer to the pointer
        filebrowse.picture.clear()
        fileName = filebrowse.listWidget.currentItem().text()
        self.clearBuffer()  # clear the tinySA serial buffer
        self.memF.write(self.readSD(fileName))  # read the file from the tinySA memory card and store in memory buffer
        if fileName[-3:] == 'bmp':
            pixmap = QPixmap()
            pixmap.loadFromData(self.memF.getvalue())
            filebrowse.picture.setPixmap(pixmap)

    # def sweepTime(self, seconds):
    #     #  0.003 to 60S
    #     command = f'sweeptime {seconds}\r'
    #     self.fifo.put(command)

    def mouseScaled(self):
        # find the current limits of the (frequency axis) viewbox and set the sweep to them
        xaxis = (ui.graphWidget.getAxis('bottom').range)
        startF = float(xaxis[0]/1e6)
        stopF = float(xaxis[1]/1e6)
        logging.debug(f'mouseScaled: start = {startF} stop = {stopF}')
        ui.start_freq.setValue(startF)
        ui.stop_freq.setValue(stopF)
        self.freq_changed(False)


class trace:
    def __init__(self, name, pen):
        self.name = name
        self.pen = pen
        self.trace = ui.graphWidget.plot([], [], name=name, pen=self.pen, width=1, padding=0)
        self.fifo = queue.SimpleQueue()

    def guiRef(self, opt):
        guiFields = ({'1': ui.m1_type, '2': ui.m2_type, '3': ui.m3_type, '4': ui.m4_type},
                     {'1': ui.m1trace, '2': ui.m2trace, '3': ui.m3trace, '4': ui.m4trace},
                     {'1': ui.trace1, '2': ui.trace2, '3': ui.trace3, '4': ui.trace4},
                     {'1': ui.t1_type, '2': ui.t2_type, '3': ui.t3_type, '4': ui.t4_type})
        Ref = guiFields[opt].get(self.name)
        return Ref

    def tType(self):
        self.traceType = self.guiRef(3).currentText()  # '3' selects trace type, 'guiRef' is replaced by the option

    def enable(self):  # show or hide a trace
        if self.guiRef(2).isChecked():  # 2 selects trace checkboxes
            tint = str("background-color: '" + self.pen + "';")
            self.guiRef(3).setStyleSheet(tint)
            self.trace.show()
        else:
            self.guiRef(3).setStyleSheet("background-color: ;")
            self.trace.hide()
        checkboxes.dwm.submit()

    def setup(self):
        self.tType()
        self.enable()


class limit:
    def __init__(self, pen, x, y, movable):  # x = None, horizontal.  y = None, vertical
        self.pen = pen
        self.x = x
        self.y = y
        self.movable = movable

    def create(self, dash=False, mark='', posn=0.99):
        label = ''
        if self.y:
            label = '{value:.1f}'
        self.line = ui.graphWidget.addLine(self.x, self.y, movable=self.movable, pen=self.pen, label=label,
                                           labelOpts={'position': 0.98, 'color': (self.pen), 'movable': True})
        self.line.addMarker(mark, posn, 10)
        if dash:
            self.line.setPen(self.pen, width=0.5, style=QtCore.Qt.DashLine)

    def visible(self, show=True):
        if show:
            self.line.show()
        else:
            self.line.hide()


class marker:
    def __init__(self, name, box):
        self.name = name
        self.linked = T1  # which trace data the marker uses as default
        self.level = 1  # marker tracking level (min or max), set per marker from GUI
        self.markerType = 'Normal'
        self.line = ui.graphWidget.addLine(88, 90, movable=True, name=name,
                                           pen=pyqtgraph.mkPen('y', width=0.5), label=self.name)
        self.deltaline = ui.graphWidget.addLine(0, 90, movable=True, name=name,
                                                pen=pyqtgraph.mkPen('y', width=0.5, style=QtCore.Qt.DashLine),
                                                label=self.name)
        self.line.addMarker('^', 0, 10)
        self.deltaF = 0  # the delta marker frequency difference
        self.deltaRelative = True
        self.deltaline.sigClicked.connect(self.deltaClicked)
        self.line.sigClicked.connect(self.lineClicked)
        self.markerBox = pyqtgraph.TextItem(text='', border=None, anchor=(-0.5, -box))  # box is vertical posn
        self.markerBox.setParentItem(ui.graphWidget.plotItem)
        self.fifo = queue.SimpleQueue()

    def traceLink(self, setting):
        traces = {'1': T1, '2': T2, '3': T3, '4': T4}
        self.linked = traces.get(str(setting))
        tint = str("background-color: '" + self.linked.pen + "';")
        self.guiRef(1).setStyleSheet(tint)
        checkboxes.dwm.submit()

    def setup(self, colour, freqField):
        # restore the marker frequencies from the configuration database and set starting conditions
        self.line.setValue(numbers.tm.record(0).value(freqField))
        self.line.label.setColor(colour)
        self.line.label.setPosition(0.02)
        self.line.label.setMovable(True)
        self.line.setPen(color=colour, width=0.5)
        self.mType()
        self.traceLink(self.linked.name)
        self.deltaline.hide()
        self.deltaline.setValue(0)
        self.deltaF = 0
        self.deltaline.label.setPosition(0.05)
        self.deltaline.label.setMovable(True)

    def start(self):  # set marker to the sweep start frequency
        if self.markerType != 'Off':
            self.line.setValue(ui.start_freq.value() * 1e6)
            self.mType()

    def spread(self):  # spread markers equally across scan range
        if self.markerType != 'Off':
            self.line.setValue(ui.start_freq.value() * 1e6 + (0.2 * int(self.name) * ui.span_freq.value() * 1e6))
            self.mType()

    def lineClicked(self):  # toggle visibility of associated delta marker
        if self.deltaline.value() != 0:
            self.deltaline.hide()
            self.deltaline.setValue(0)
        else:
            self.deltaline.show()
            self.deltaline.setValue(self.line.value() + ui.span_freq.value() * 2.5e4)
            self.deltaF = 0

    def deltaClicked(self):  # toggle relative or absolute labelling
        if self.deltaRelative:
            self.deltaRelative = False
        else:
            self.deltaRelative = True

    def deltaMoved(self):  # set the delta freq offset
        self.deltaF = self.deltaline.value() - self.line.value()

    def mType(self):
        self.markerType = self.guiRef(0).currentText()  # current combobox value from appropriate GUI field
        if self.markerType == 'Off' or self.markerType == '':
            self.line.hide()
            self.deltaline.hide()
            self.deltaline.setValue(0)
            self.deltaF = 0
        else:
            self.line.show()

        # if self.markerType in ('Max', 'Min'):
        #     threshold.line.show()  # the peak detection threshold line
        #     lowF.line.show()  # the boundary markers
        #     highF.line.show()
        # else:
        #     threshold.line.hide()
        #     lowF.line.hide()
        #     highF.line.hide()

    def setDelta(self):  # delta marker locking to reference marker
        self.deltaline.setValue(self.line.value() + self.deltaF)

    def guiRef(self, opt):
        guiFields = ({'1': ui.m1_type, '2': ui.m2_type, '3': ui.m3_type, '4': ui.m4_type},
                     {'1': ui.m1trace, '2': ui.m2trace, '3': ui.m3trace, '4': ui.m4trace},
                     {'1': ui.trace1, '2': ui.trace2, '3': ui.trace3, '4': ui.trace4},
                     {'1': ui.t1_type, '2': ui.t2_type, '3': ui.t3_type, '4': ui.t4_type})
        Ref = guiFields[opt].get(self.name)
        return Ref

    def updateMarker(self, frequencies, readings, maxima, minima):  # called by updateGUI()
        if self.markerType == 'Off':
            self.markerBox.setVisible(False)
            return
        else:
            self.markerBox.setVisible(True)

        if self.markerType in ('Max', 'Min'):
            maxmin = self.maxMin(frequencies, readings, maxima, minima)
            # maxmin is a tuple of lists where [0, x] are indices in the frequency array of the max and [1, x] are min
            logging.debug(f'updateMarker: maxmin = {maxmin}')
            if self.markerType == 'Max':
                self.line.setValue(maxmin[0][self.level])
                if self.deltaline.value != 0:
                    self.deltaline.setValue(maxmin[0][self.level] + self.deltaF)  # needs to be index delta not F
            if self.markerType == 'Min':
                self.line.setValue(maxmin[1][self.level])
                if self.deltaline.value != 0:
                    self.deltaline.setValue(maxmin[1][self.level] + self.deltaF)  # needs to be index delta not F

        options = {'Normal': readings[0],
                   'Average': np.nanmean(readings[:ui.avgBox.value()], axis=0),
                   'Max': maxima,
                   'Min': minima}

        levels = options.get(self.linked.traceType)

        lineIndex = np.argmin(np.abs(frequencies - (self.line.value())))  # find closest value in freq array
        linedBm = levels[lineIndex]
        self.markerBox.setText(f'M{self.line.name()} {self.line.value()/1e6:.{4}f} {linedBm:.1f}dBm')

        if self.deltaline.value != 0:
            deltaLineIndex = np.argmin(np.abs(frequencies - (self.deltaline.value())))  # closest value in freq array
            if self.deltaRelative:
                deltaLinedBm = levels[deltaLineIndex]
                dBm = deltaLinedBm - linedBm
                self.deltaline.label.setText(f'{chr(916)}{self.line.name()} {dBm:.1f}dBm\n{self.deltaF/1e6:.{3}f}MHz')
            else:
                dBm = levels[deltaLineIndex]
                self.deltaline.label.setText(
                    f'{chr(916)}{self.line.name()} {dBm:.1f}dBm\n{self.deltaline.value()/1e6:.{3}f}MHz')

    def addFreqMarker(self, freq, colour, name, band=True):  # adds simple freq marker without full marker capability
        if ui.presetLabel.isChecked():
            if band:
                self.marker = ui.graphWidget.addLine(freq, 90, pen=pyqtgraph.mkPen(colour, width=0.5,
                                                     style=QtCore.Qt.DashLine), label=name, labelOpts={'position': 0.97,
                                                     'color': (colour)})
            else:
                self.marker = ui.graphWidget.addLine(freq, 90, pen=pyqtgraph.mkPen(colour, width=0.5,
                                                     style=QtCore.Qt.DashLine), label=name, labelOpts={'position': 0.04,
                                                     'color': (colour), 'anchors': ((0, 0.2), (0, 0.2))})
            self.marker.label.setMovable(True)
        else:
            self.marker = ui.graphWidget.addLine(freq, 90,
                                                 pen=pyqtgraph.mkPen(colour, width=0.5, style=QtCore.Qt.DashLine))
        self.fifo.put(self.marker)  # store the marker object in a queue

    def delFreqMarkers(self):
        for i in range(0, self.fifo.qsize()):
            ui.graphWidget.removeItem(self.fifo.get())  # remove the marker and its corresponding object in the queue

    def maxMin(self, frequencies, readings, maxima, minima):  # finds the signal max/min (indexes) for setting markers
        options = {'Normal': readings[0],
                   'Average': np.nanmean(readings[:ui.avgBox.value()], axis=0),
                   'Max': maxima,
                   'Min': minima}
        levels = options.get(self.linked.traceType)

        logging.debug(f'maxmin: linked tracetype = {self.linked.traceType}')

        # mask outside high/low freq boundary lines
        levels = np.ma.masked_where(frequencies < lowF.line.value(), levels)
        levels = np.ma.masked_where(frequencies > highF.line.value(), levels)

        # mask readings below threshold line
        levels = np.ma.masked_where(levels <= threshold.line.value(), levels)

        # calculate a frequency width factor to use to mask readings near each max/min frequency
        if ui.rbw_auto.isChecked():
            fWidth = preferences.rbw_x.value() * 850 * 1e3
        else:
            fWidth = preferences.rbw_x.value() * float(ui.rbw_box.currentText()) * 1e3

        maxi = [np.argmax(levels)]  # the index of the highest peak in the masked readings array
        mini = [np.argmin(levels)]  # the index of the deepest minimum in the masked readings array
        nextMax = nextMin = levels
        for i in range(8):
            # mask frequencies around detected peaks and find the next 8 highest/lowest peaks
            nextMax = np.ma.masked_where(np.abs(frequencies[maxi[-1]] - frequencies) < fWidth, nextMax)
            maxi.append(np.argmax(nextMax))
            nextMin = np.ma.masked_where(np.abs(frequencies[mini[-1]] - frequencies) < fWidth, nextMin)
            mini.append(np.argmin(nextMin))
        return (list(frequencies[maxi]), list(frequencies[mini]))

    def setLevel(self, setting):
        self.level = setting - 1  # array indexes start at 0 not 1


class WorkerSignals(QtCore.QObject):
    error = QtCore.pyqtSignal(str)
    result = QtCore.pyqtSignal(np.ndarray, np.ndarray, np.ndarray, np.ndarray, float)
    fullSweep = QtCore.pyqtSignal(np.ndarray, np.ndarray)
    saveResults = QtCore.pyqtSignal(np.ndarray, np.ndarray)
    resetGUI = QtCore.pyqtSignal(np.ndarray, np.ndarray)
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


class modelView():
    '''set up and process data models bound to the GUI widgets'''

    def __init__(self, tableName, dbName):
        self.tableName = tableName
        self.tm = QSqlRelationalTableModel(db=dbName)
        self.dwm = QDataWidgetMapper()
        self.currentRow = 0

    def createTableModel(self):
        self.tm.setTable(self.tableName)
        self.dwm.setModel(self.tm)
        self.dwm.setSubmitPolicy(QDataWidgetMapper.AutoSubmit)

    def addRow(self):  # adds a blank row to the frequency bands table widget above current row
        if self.currentRow == 0:
            self.tm.insertRow(0)
        else:
            self.tm.insertRow(self.currentRow)
            bands.tm.select()
            bandstype.tm.select()
        preferences.freqBands.selectRow(self.currentRow)

    def saveChanges(self):
        self.dwm.submit()

    def deleteRow(self, single=True):  # deletes rows in the frequency bands table widget
        if single:
            logging.info(f'deleteRow: current row = {self.currentRow}')
            self.tm.removeRow(self.currentRow)
        else:
            for i in range(0, self.tm.rowCount()):
                self.tm.removeRow(i)
        self.tm.select()
        self.tm.layoutChanged.emit()
        self.dwm.submit()

    def tableClicked(self):
        self.currentRow = preferences.freqBands.currentIndex().row()  # the row index from the QModelIndexObject
        logging.debug(f'row {self.currentRow} clicked')

    def insertData(self, **data):
        record = self.tm.record()
        logging.debug(f'insertData: record = {record}')
        for key, value in data.items():
            logging.info(f'insertData: key = {key} value={value}')
            record.setValue(str(key), value)
        self.tm.insertRecord(-1, record)
        self.tm.layoutChanged.emit()
        self.dwm.submit()

    def filterType(self, prefsDialog, boxText):
        sql = 'preset = "' + boxText + '"'
        if prefsDialog:
            if boxText == 'show all':
                sql = ''
            self.tm.setFilter(sql)
        else:
            sql = 'visible = "1" AND preset = "' + boxText + '"'
            if boxText == 'show all':
                sql = 'visible = "1"'
            if tinySA.tinySA4 is False:  # It's a tinySA basic with limited frequency range
                sql = sql + ' AND startF <= "960000000"'
            index = ui.band_box.currentIndex()
            self.tm.setFilter(sql)
            ui.band_box.setCurrentIndex(index)

    def readCSV(self, fileName):
        with open(fileName, "r") as fileInput:
            reader = csv.DictReader(fileInput)
            for row in reader:
                record = self.tm.record()
                for key, value in row.items():
                    logging.debug(f'readCSV: key = {key} value = {value}')
                    # don't understand how to make relation work for these fields
                    if key == 'preset':
                        value = presetID(value)
                    if key == 'colour':
                        value = colourID(value)
                    if key == 'value':
                        value = int(eval(value))
                    # to match RF mic CSV files
                    if key == 'Frequency':
                        key = 'startF'
                        value = str(float(value) / 1e3)
                    if key != 'ID':  # ID is the table primary key and is auto-populated
                        record.setValue(str(key), value)
                if record.value('value') not in (0, 1):  # because it's not present in RF mic CSV files
                    record.setValue('value', 1)
                if record.value('preset') == '':  # preset missing so use current preferences filterbox text
                    record.setValue('preset', presetID(preferences.filterBox.currentText()))
                self.tm.insertRecord(-1, record)
        self.tm.select()
        self.tm.layoutChanged.emit()
        self.dwm.submit()

    def writeCSV(self, fileName):
        header = []
        for i in range(1, self.tm.columnCount()):
            header.append(self.tm.record().fieldName(i))
        with open(fileName, "w") as fileOutput:
            output = csv.writer(fileOutput)
            output.writerow(header)
            for rowNumber in range(self.tm.rowCount()):
                fields = [self.tm.data(self.tm.index(rowNumber, columnNumber))
                          for columnNumber in range(1, 8)]
                output.writerow(fields)

    def mapWidget(self, modelName):  # maps the widget combo-box fields to the database tables, using the mapping table
        maps.tm.setFilter('model = "' + modelName + '"')
        for index in range(0, maps.tm.rowCount()):
            gui = maps.tm.record(index).value('gui')
            column = maps.tm.record(index).value('column')
            self.dwm.addMapping(eval(gui), int(column))

###############################################################################
# respond to GUI signals


def band_changed():
    index = ui.band_box.currentIndex()
    startF = bandselect.tm.record(index).value('StartF')
    stopF = bandselect.tm.record(index).value('StopF')
    if stopF not in (0, ''):
        ui.start_freq.setValue(startF / 1e6)
        ui.stop_freq.setValue(stopF / 1e6)
        tinySA.freq_changed(False)  # start/stop mode
    else:
        centreF = startF / 1e6
        ui.centre_freq.setValue(centreF)
        ui.span_freq.setValue(int(centreF / 10))  # default span to a tenth of the centre freq
        tinySA.freq_changed(True)  # centre mode
    freqMarkers()


def addBandPressed():  # this is not going to work now ################################################################
    if ui.m1_type.currentText() == 'Off':
        message = 'Please enable Marker 1'
        popUp(message, QMessageBox.Ok, QMessageBox.Information)
        return
    if ui.m1_type.currentText() != 'Off' and ui.m2_type.currentText() != 'Off':  # Two markers to set a band limit
        if M1.line.value() >= M2.line.value():
            message = 'M1 frequency >= M2 frequency'
            popUp(message, QMessageBox.Ok, QMessageBox.Information)
            return
        ID = presetID(str(ui.filterBox.currentText()))
        title = "New Frequency Band"
        message = "Enter a name for the new band."
        bandName, ok = QInputDialog.getText(None, title, message, QLineEdit.Normal, "")
        bands.insertData(name=bandName, type=ID, startF=f'{M1.line.value()}',
                         stopF=f'{M2.line.value()}', visible=1, colour=colourID('green'))  # colourID(value)
    else:  # If only Marker 1 is enabled then this creates a spot Frequency marker
        title = "New Spot Frequency Marker"
        message = "Enter a name for the Spot Frequency"
        spotName, ok = QInputDialog.getText(None, title, message, QLineEdit.Normal, "")
        bands.insertData(name=spotName, type=12, startF=f'{M1.line.value()}',
                         stopF='', visible=1, colour=colourID('orange'))  # preset 12 is Marker (spot frequency).
    bands.tm.select()
    bandselect.tm.select()


def attenuate_changed():
    atten = ui.atten_box.value()
    if ui.atten_auto.isChecked():
        atten = 'auto'
        ui.atten_box.setEnabled(False)
    else:
        if not ui.lna_box.isChecked():  # attenuator and lna are switched so mutually exclusive
            ui.atten_box.setEnabled(True)
    command = f'attenuate {str(atten)}\r'
    tinySA.fifo.put(command)


def rbwChanged():
    if ui.rbw_auto.isChecked():  # can't calculate Points because we don't know what the RBW will be
        ui.rbw_box.setEnabled(False)
        ui.points_auto.setChecked(False)
        ui.points_auto.setEnabled(False)
    else:
        ui.rbw_box.setEnabled(True)
        ui.points_auto.setEnabled(True)
    tinySA.setRBW()  # if measurement thread is running, calling setRBW() will force it to update scan parameters


def pointsChanged():
    if ui.points_auto.isChecked():
        ui.points_box.setEnabled(False)
        ui.rbw_box.setEnabled(True)
    else:
        ui.points_box.setEnabled(True)
    tinySA.resume()  # without this command, the trace doesn't update


def memChanged():
    depth = ui.memBox.value()
    if depth < ui.avgBox.value():
        ui.avgBox.setValue(depth)
    tinySA.scanMemory = depth


def markerToStart():
    M1.start()
    M2.start()
    M3.start()
    M4.start()


def markerToCentre():
    M1.spread()
    M2.spread()
    M3.spread()
    M4.spread()


def markerLevel():
    M1.setLevel(ui.m1track.value())
    M2.setLevel(ui.m2track.value())
    M3.setLevel(ui.m3track.value())
    M4.setLevel(ui.m4track.value())


def setPreferences():  # called when the preferences window is closed
    checkboxes.dwm.submit()
    bands.tm.submitAll()
    threshold.line.setValue(preferences.peakThreshold.value())
    best.visible(preferences.neg25Line.isChecked())
    maximum.visible(preferences.zeroLine.isChecked())
    damage.visible(preferences.plus6Line.isChecked())

    if ui.presetMarker.isChecked():
        freqMarkers()
    isMixerMode()


def dialogPrefs():  # called by clicking on the setup > preferences menu
    bands.filterType(True, preferences.filterBox.currentText())
    bands.currentRow = 0
    preferences.freqBands.selectRow(bands.currentRow)
    pwindow.show()


def about():
    message = ('TinySA Ultra GUI programme using Qt5 and PyQt\nAuthor: Ian Jefferson G4IXT\n\nVersion: {} \nConfig: {}'
               .format(app.applicationVersion(), config.databaseName()))
    popUp(message, QMessageBox.Ok, QMessageBox.Information)


def clickEvent():
    logging.info('clickEvent')


def testComPort():
    index = preferences.deviceBox.currentIndex()
    tinySA.testPort(tinySA.ports[index - 1])  # allow for 'select device' entry


##############################################################################
# other methods

def saveFile(frequencies, readings):
    if preferences.saveSweep.isChecked():
        timeStamp = time.strftime('%Y-%m-%d-%H%M%S')
        saver = Worker(writeSweep, timeStamp, frequencies, readings)  # workers deleted when thread ends
        threadpool.start(saver)


def writeSweep(timeStamp, frequencies, readings):
    array = np.insert(readings, 0, frequencies, axis=0)  # insert the measurement freqs at the top of the readings array
    dBm = np.transpose(np.round(array, decimals=2))  # transpose columns and rows
    fileName = str(timeStamp + '_RBW' + ui.rbw_box.currentText() + '.csv')
    with open(fileName, "w", newline='') as fileOutput:
        output = csv.writer(fileOutput)
        for rowNumber in range(0, np.shape(dBm)[0]):
            fields = [dBm[rowNumber, columnNumber] for columnNumber in range(0, np.shape(dBm)[1])]
            output.writerow(fields)


def getPath(dbName):
    # check if a personal database file exists already
    personalDir = platformdirs.user_config_dir(appname=app.applicationName(), appauthor=False)
    if not os.path.exists(personalDir):
        os.mkdir(personalDir)

    if os.path.isfile(os.path.join(personalDir, dbName)):
        logging.info(f'Database {dbName} found at {personalDir}')
        return personalDir

    # if not, then check if a global database file exists
    globalDir = platformdirs.site_config_dir(appname=app.applicationName(), appauthor=False)
    if os.path.isfile(os.path.join(globalDir, dbName)):
        shutil.copy(os.path.join(globalDir, dbName), personalDir)
        logging.info(f'Database {dbName} copied from {globalDir} to {personalDir}')
        return personalDir

    # If not, then look in current working folder & where the python file is stored/linked from
    workingDirs = [os.path.dirname(__file__), os.path.dirname(os.path.realpath(__file__)), os.getcwd()]
    for directory in workingDirs:
        if os.path.isfile(os.path.join(directory, dbName)):
            shutil.copy(os.path.join(directory, dbName), personalDir)
            logging.info(f'{dbName} copied from {directory} to {personalDir}')
            return personalDir
    raise FileNotFoundError("Unable to find the database {self.dbName}")


def connect(dbFile, con):

    db = QSqlDatabase.addDatabase('QSQLITE', connectionName=con)
    dbPath = getPath(dbFile)
    if QtCore.QFile.exists(os.path.join(dbPath, dbFile)):
        db.setDatabaseName(os.path.join(dbPath, dbFile))
        db.open()
        logging.info(f'{dbFile} open: {db.isOpen()}  Connection = "{db.connectionName()}"')
        logging.debug(f'tables available = {db.tables()}')
        # database.setConnectOptions('PRAGMA foreign_keys = ON;')
    else:
        logging.info('Database file {dbPath}{dbFile} is missing')
        popUp('Database file is missing', QMessageBox.Ok, QMessageBox.Critical)
        return
    return db


def disconnect(db):
    db.close()
    logging.info(f'Database {db.databaseName()} open: {db.isOpen()}')
    QSqlDatabase.removeDatabase(db.databaseName())


def exit_handler():
    if len(tinySA.ports) != 0:
        # save the marker frequencies
        record = numbers.tm.record(0)
        record.setValue('m1f', float(M1.line.value()))
        record.setValue('m2f', float(M2.line.value()))
        record.setValue('m3f', float(M3.line.value()))
        record.setValue('m4f', float(M4.line.value()))
        numbers.tm.setRecord(0, record)
        # save the gui field values and checkbox states
        if tinySA.sweeping:
            tinySA.sweeping = False  # tell the measurement thread to stop
            while tinySA.threadRunning:
                time.sleep(0.05)  # wait for measurements to stop
        tinySA.resume()
        tinySA.usbSend()
        tinySA.closePort()  # close USB connection

    checkboxes.dwm.submit()
    disconnect(config)

    logging.info('QtTinySA Closed')


def popUp(message, button, icon):
    # icon can be = QMessageBox.Warning, QMessageBox.Information, QMessageBox.Critical, QMessageBox.Question
    msg = QMessageBox(parent=(window))
    msg.setIcon(icon)
    msg.setText(message)
    msg.setStandardButtons(button)
    msg.exec_()


def freqMarkers():
    M1.delFreqMarkers()
    M2.delFreqMarkers()
    for i in range(0, presetmarker.tm.rowCount()):
        try:
            startF = presetmarker.tm.record(i).value('StartF')
            stopF = presetmarker.tm.record(i).value('StopF')
            colour = presetmarker.tm.record(i).value('colour')
            name = presetmarker.tm.record(i).value('name')
            if ui.presetMarker.isChecked() and presetmarker.tm.record(i).value('visible') and stopF in (0, ''):
                M1.addFreqMarker(startF, colour, name, band=False)
                if ui.presetLabel.isChecked() and ui.presetLabel.checkState() == 2:
                    M1.marker.label.setAngle(90)
                    M1.marker.label.setPosition(0)
            if ui.presetMarker.isChecked() and stopF not in (0, ''):  # it's a band marker
                M1.addFreqMarker(startF, colour, name)
                M2.addFreqMarker(stopF, colour, name)
        except ValueError:
            continue


def freqMarkerLabel():
    freqMarkers()


def exportData():
    filename = QFileDialog.getSaveFileName(caption="Save As", filter="Comma Separated Values (*.csv)")
    logging.info(f'export filename is {filename[0]}')
    if filename[0] != '':
        bands.writeCSV(filename[0])


def importData():
    filename = QFileDialog.getOpenFileName(caption="Select File to Import", filter="Comma Separated Values (*.csv)")
    logging.info(f'import filename is {filename[0]}')
    if filename[0] != '':
        bands.readCSV(filename[0])


def isMixerMode():
    if preferences.freqLO.value() == 0:
        ui.mixerMode.setVisible(False)
        ui.start_freq.setStyleSheet('background-color:None')
        ui.stop_freq.setStyleSheet('background-color:None')
        ui.centre_freq.setStyleSheet('background-color:None')
        ui.start_freq.setMaximum(tinySA.maxF)
        ui.centre_freq.setMaximum(tinySA.maxF)
        ui.stop_freq.setMaximum(tinySA.maxF)
    else:
        ui.mixerMode.setVisible(True)
        ui.start_freq.setStyleSheet('background-color:lightGreen')
        ui.stop_freq.setStyleSheet('background-color:lightGreen')
        ui.centre_freq.setStyleSheet('background-color:lightGreen')
        ui.start_freq.setMaximum(100000)
        ui.centre_freq.setMaximum(100000)
        ui.stop_freq.setMaximum(100000)


def presetID(typeF):  # using the QSQLRelation directly doesn't work for preset.  Can't see why.
    for i in range(0, bandstype.tm.rowCount()):
        preset = bandstype.tm.record(i).value('preset')
        if preset == typeF:
            ID = bandstype.tm.record(i).value('ID')
            return ID
    return 1


def colourID(shade):  # using the QSQLRelation directly doesn't work for colour.  Can't see why.
    for i in range(0, colours.tm.rowCount()):
        colour = colours.tm.record(i).value('colour')
        if colour == shade.lower():
            ID = colours.tm.record(i).value('ID')
            return ID
    return 1


def setWaterfall():
    ui.waterfall.setMaximumSize(QtCore.QSize(16777215, ui.waterfallSize.value()))


def connectActive():
    # Connect signals from controls that send messages to tinySA.  Called by 'initialise'.

    ui.atten_box.valueChanged.connect(attenuate_changed)
    ui.atten_auto.clicked.connect(attenuate_changed)
    ui.spur_box.clicked.connect(tinySA.spur)
    ui.lna_box.clicked.connect(tinySA.lna)
    ui.memBox.valueChanged.connect(memChanged)
    ui.points_auto.stateChanged.connect(pointsChanged)
    ui.points_box.editingFinished.connect(pointsChanged)
    ui.setRange.clicked.connect(tinySA.mouseScaled)
    ui.band_box.currentIndexChanged.connect(band_changed)
    ui.band_box.activated.connect(band_changed)
    ui.rbw_box.currentIndexChanged.connect(rbwChanged)
    ui.rbw_auto.clicked.connect(rbwChanged)

    # frequencies
    ui.start_freq.editingFinished.connect(tinySA.freq_changed)
    ui.stop_freq.editingFinished.connect(tinySA.freq_changed)
    ui.centre_freq.valueChanged.connect(lambda: tinySA.freq_changed(True))  # centre/span mode
    ui.span_freq.valueChanged.connect(lambda: tinySA.freq_changed(True))  # centre/span mode

    ui.sampleRepeat.valueChanged.connect(tinySA.sampleRep)

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
    ui.timeSpectrum.clicked.connect(lambda: ui.stackedWidget.setCurrentWidget(ui.View3D))
    ui.analyser.clicked.connect(lambda: ui.stackedWidget.setCurrentWidget(ui.ViewNormal))

    # preferences
    ui.actionAbout_QtTinySA.triggered.connect(about)

    # filebrowse
    ui.actionBrowse_TinySA.triggered.connect(tinySA.dialogBrowse)
    filebrowse.download.clicked.connect(tinySA.fileDownload)
    filebrowse.listWidget.itemClicked.connect(tinySA.fileShow)

    # Quit
    ui.actionQuit.triggered.connect(app.closeAllWindows)

    # Sweep time
    # ui.sweepTime.valueChanged.connect(lambda: tinySA.sweepTime(ui.sweepTime.value()))


def connectPassive():
    # Connect signals from GUI controls that don't cause messages to go to the tinySA

    ui.scan_button.clicked.connect(tinySA.scan)
    ui.run3D.clicked.connect(tinySA.scan)

    # marker dragging
    M1.line.sigPositionChanged.connect(M1.setDelta)
    M2.line.sigPositionChanged.connect(M2.setDelta)
    M3.line.sigPositionChanged.connect(M3.setDelta)
    M4.line.sigPositionChanged.connect(M4.setDelta)
    M1.deltaline.sigPositionChanged.connect(M1.deltaMoved)
    M2.deltaline.sigPositionChanged.connect(M2.deltaMoved)
    M3.deltaline.sigPositionChanged.connect(M3.deltaMoved)
    M4.deltaline.sigPositionChanged.connect(M4.deltaMoved)

    # marker setting within span range
    ui.mkr_start.clicked.connect(markerToStart)
    ui.mkr_centre.clicked.connect(markerToCentre)

    # marker tracking level
    ui.m1track.valueChanged.connect(lambda: M1.setLevel(ui.m1track.value()))
    ui.m2track.valueChanged.connect(lambda: M2.setLevel(ui.m2track.value()))
    ui.m3track.valueChanged.connect(lambda: M3.setLevel(ui.m3track.value()))
    ui.m4track.valueChanged.connect(lambda: M4.setLevel(ui.m4track.value()))

    # marker type changes
    ui.m1_type.activated.connect(M1.mType)
    ui.m2_type.activated.connect(M2.mType)
    ui.m3_type.activated.connect(M3.mType)
    ui.m4_type.activated.connect(M4.mType)

    # frequency band markers
    ui.presetMarker.clicked.connect(freqMarkers)
    ui.presetLabel.clicked.connect(freqMarkerLabel)
    ui.mToBand.clicked.connect(addBandPressed)
    ui.filterBox.currentTextChanged.connect(freqMarkers)

    # trace checkboxes
    ui.trace1.stateChanged.connect(T1.enable)
    ui.trace2.stateChanged.connect(T2.enable)
    ui.trace3.stateChanged.connect(T3.enable)
    ui.trace4.stateChanged.connect(T4.enable)

    # trace type changes
    ui.t1_type.activated.connect(T1.tType)
    ui.t2_type.activated.connect(T2.tType)
    ui.t3_type.activated.connect(T3.tType)
    ui.t4_type.activated.connect(T4.tType)

    # preferences
    preferences.addRow.clicked.connect(bands.addRow)
    preferences.deleteRow.clicked.connect(lambda: bands.deleteRow(True))
    preferences.deleteAll.clicked.connect(lambda: bands.deleteRow(False))
    preferences.freqBands.clicked.connect(bands.tableClicked)
    preferences.filterBox.currentTextChanged.connect(lambda: bands.filterType(True, preferences.filterBox.currentText()))
    pwindow.finished.connect(setPreferences)  # update database checkboxes table on dialogue window close
    preferences.exportButton.pressed.connect(exportData)
    preferences.importButton.pressed.connect(importData)
    preferences.deviceBox.activated.connect(testComPort)
    ui.filterBox.currentTextChanged.connect(lambda: bandselect.filterType(False, ui.filterBox.currentText()))
    ui.actionPreferences.triggered.connect(dialogPrefs)  # open preferences dialogue when its menu is clicked

    # Waterfall
    ui.waterfallSize.valueChanged.connect(setWaterfall)


###############################################################################
# Instantiate classes

tinySA = analyser()

# create QApplication for the GUI
app = QtWidgets.QApplication([])
app.setApplicationName('QtTinySA')
app.setApplicationVersion(' v1.0.2')
window = QtWidgets.QMainWindow()
ui = QtTinySpectrum.Ui_MainWindow()
ui.setupUi(window)

# pwindow is the preferences dialogue box
pwindow = QtWidgets.QDialog()
preferences = QtTSApreferences.Ui_Preferences()
preferences.setupUi(pwindow)

# fwindow is the filebrowse dialogue box
fwindow = QtWidgets.QDialog()
filebrowse = QtTSAfilebrowse.Ui_Filebrowse()
filebrowse.setupUi(fwindow)

# Traces
T1 = trace('1', 'lightblue')
T2 = trace('2', 'indianred')
T3 = trace('3', 'orange')
T4 = trace('4', 'yellow')

# Markers
M1 = marker('1', 0.1)
M2 = marker('2', 0.6)
M3 = marker('3', 1.1)
M4 = marker('4', 1.7)

# limit lines
best = limit('gold', None, -25, movable=False)
maximum = limit('red', None, 0, movable=False)
damage = limit('red', None, 6, movable=False)
threshold = limit('cyan', None, preferences.peakThreshold.value(), movable=True)
lowF = limit('cyan', (ui.start_freq.value() + ui.span_freq.value()/20)*1e6, None, movable=True)
highF = limit('cyan', (ui.stop_freq.value() - ui.span_freq.value()/20)*1e6, None, movable=True)
best.create(True, '|>', 0.99)
maximum.create(True, '|>', 0.99)
damage.create(False, '|>', 0.99)
threshold.create(True, '<|', 0.99)
lowF.create(True, '|>', 0.01)
highF.create(True, '<|', 0.01)

# Database and models for configuration settings
config = connect("QtTSAprefs.db", "settings")

checkboxes = modelView('checkboxes', config)
numbers = modelView('numbers', config)
markers = modelView('marker', config)
traces = modelView('trace', config)
tracetext = modelView('combo', config)
markertext = modelView('combo', config)
rbwtext = modelView('combo', config)
bandstype = modelView('freqtype', config)
colours = modelView('SVGColour', config)
maps = modelView('mapping', config)
bands = modelView('frequencies', config)
presetmarker = modelView('frequencies', config)
bandselect = modelView('frequencies', config)

# Database and models for recording and playback (can't get multiple databases to work)
# saveData = connect("QtTSArecording.db", "measurements")

# data = modelView('data', saveData)
# settings = modelView('settings', saveData)
# data.tm.select()
# settings.tm.select()


###############################################################################
# GUI settings

# pyqtgraph settings for spectrum display
ui.graphWidget.setYRange(-112, -20)
ui.graphWidget.setDefaultPadding(padding=0)
ui.graphWidget.showGrid(x=True, y=True)
ui.graphWidget.setLabel('bottom', '', units='Hz')

# pyqtgraph settings for waterfall and histogram display
ui.waterfall.setDefaultPadding(padding=0)
ui.waterfall.getPlotItem().hideAxis('bottom')
ui.waterfall.setLabel('left', 'Time', **{'color': '#FFF', 'font-size': '3pt'})
ui.waterfall.invertY(True)
ui.histogram.setDefaultPadding(padding=0)
ui.histogram.plotItem.invertY(True)
ui.histogram.getPlotItem().hideAxis('bottom')
ui.histogram.getPlotItem().hideAxis('left')

###############################################################################
# set up the application
logging.info(f'{app.applicationName()}{app.applicationVersion()}')

# table models - read/write views of the configuration data

# field mapping of the checkboxes from the database
maps.createTableModel()
maps.tm.select()

# to populate the preset bands and markers relational table in the preferences dialogue
bands.createTableModel()
bands.tm.select()
bands.tm.setSort(3, QtCore.Qt.AscendingOrder)
bands.tm.setHeaderData(5, QtCore.Qt.Horizontal, 'visible')
bands.tm.setHeaderData(7, QtCore.Qt.Horizontal, 'LO')
bands.tm.setEditStrategy(QSqlRelationalTableModel.OnFieldChange)
bands.tm.setRelation(2, QSqlRelation('freqtype', 'ID', 'preset'))  # set 'type' column to a freq type choice combo box
bands.tm.setRelation(5, QSqlRelation('boolean', 'ID', 'value'))  # set 'view' column to a True/False choice combo box
bands.tm.setRelation(6, QSqlRelation('SVGColour', 'ID', 'colour'))  # set 'marker' column to a colours choice combo box
presets = QSqlRelationalDelegate(preferences.freqBands)
preferences.freqBands.setItemDelegate(presets)
colHeader = preferences.freqBands.horizontalHeader()
colHeader.setSectionResizeMode(QtWidgets.QHeaderView.ResizeToContents)

# for the filter combo box in the preferences dialogue
bandstype.createTableModel()
bandstype.tm.select()

# to lookup the preset bands and markers colours because can't get the relationships to work
colours.createTableModel()
colours.tm.select()

# for the preset markers, which need different filtering to the preferences dialogue
presetmarker.createTableModel()
presetmarker.tm.setRelation(6, QSqlRelation('SVGColour', 'ID', 'colour'))
presetmarker.tm.setRelation(2, QSqlRelation('freqtype', 'ID', 'preset'))
presetmarker.tm.setSort(3, QtCore.Qt.AscendingOrder)
presetmarker.tm.select()

# populate the ui band selection combo box, which needs different filter to preferences dialogue and preset markers
bandselect.createTableModel()
bandselect.tm.setRelation(2, QSqlRelation('freqtype', 'ID', 'preset'))
bandselect.tm.setRelation(5, QSqlRelation('boolean', 'ID', 'value'))
bandselect.tm.setRelation(6, QSqlRelation('SVGColour', 'ID', 'colour'))
bandselect.tm.setSort(3, QtCore.Qt.AscendingOrder)
ui.band_box.setModel(bandselect.tm)
ui.band_box.setModelColumn(1)
bandselect.tm.select()

# populate the preferences dialogue and ui filter combo boxes
preferences.filterBox.setModel(bandstype.tm)
preferences.filterBox.setModelColumn(1)
ui.filterBox.setModel(bandstype.tm)
ui.filterBox.setModelColumn(1)

# connect the preferences dialogue box freq band table widget to the data model
preferences.freqBands.setModel(bands.tm)
preferences.freqBands.hideColumn(0)  # ID
preferences.freqBands.verticalHeader().setVisible(True)

# connect GUI controls that set marker associated trace - has to go here so that colours initialise
ui.m1trace.valueChanged.connect(lambda: M1.traceLink(ui.m1trace.value()))
ui.m2trace.valueChanged.connect(lambda: M2.traceLink(ui.m2trace.value()))
ui.m3trace.valueChanged.connect(lambda: M3.traceLink(ui.m3trace.value()))
ui.m4trace.valueChanged.connect(lambda: M4.traceLink(ui.m4trace.value()))

# Map database tables to preferences/GUI fields * lines need to be in this order and here or the mapping doesn't work *
checkboxes.createTableModel()
checkboxes.mapWidget('checkboxes')  # uses mapping table from database
checkboxes.tm.select()
checkboxes.dwm.setCurrentIndex(0)  # 0 = (last used) default settings

# populate the rbw combobox
rbwtext.createTableModel()
rbwtext.tm.setFilter('type = "rbw"')
ui.rbw_box.setModel(rbwtext.tm)
rbwtext.tm.select()

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

# The models for saving number, marker and trace settings
markers.createTableModel()
traces.createTableModel()
traces.tm.select()
numbers.createTableModel()
numbers.mapWidget('numbers')  # uses mapping table from database
numbers.tm.select()
numbers.dwm.setCurrentIndex(0)

window.show()
window.setWindowTitle(app.applicationName() + app.applicationVersion())
window.setWindowIcon(QIcon(os.path.join(basedir, 'tinySAsmall.png')))

# connect GUI controls that send messages to tinySA
connectPassive()

# try to open a USB connection to the TinySA hardware
tinySA.usbCheck.start(500)  # check again every 500mS

###############################################################################
# run the application until the user closes it

try:
    app.exec()
finally:
    exit_handler()  # close cleanly
