#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright 2024 Ian Jefferson G4IXT
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
from PyQt5 import QtWidgets, QtCore
from PyQt5.QtWidgets import QMessageBox, QDataWidgetMapper, QFileDialog, QInputDialog, QLineEdit
from PyQt5.QtSql import QSqlDatabase, QSqlRelation, QSqlRelationalTableModel, QSqlRelationalDelegate
from PyQt5.QtGui import QPixmap
from datetime import datetime
import pyqtgraph
import QtTinySpectrum  # the GUI
import QtTSApreferences  # the GUI preferences window
import QtTSAfilebrowse
import struct
import serial
from serial.tools import list_ports
from io import BytesIO

#  For 3D
import pyqtgraph.opengl as pyqtgl

# os.environ['PYOPENGL_PLATFORM'] = 'egl'

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

    def testPort(self, port):
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
            # split it into a list of [device, major version number, minor version number, other stuff]
            self.firmware = firmware.replace('_', '-').split('-')
            if float(self.firmware[1][-3:] + self.firmware[2]) < 1.4177:
                logging.info('for fastest possible scan speed, upgrade firmware to v1.4-177 or later')
            if self.firmware[0] in ('tinySA4',  'tinySA_') and self.firmware[1][0] == "v":
                self.initialise(self.firmware)
            if self.firmware[1][0] != "v":
                logging.info(f'{port.device} serial command found version {firmware}. Expected to find tinySA_vn.n-nnn')

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
        self.setSweep(ui.start_freq.value() * 1e6, ui.stop_freq.value() * 1e6)  # set the tinySA default scan range
        # product = 'tinySA'  # used for testing
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

        # connect the rbw & frequency boxes here or it causes startup index errors when they are populated
        # ui.rbw_box.currentIndexChanged.connect(rbwChanged)
        # ui.rbw_auto.clicked.connect(rbwChanged)
        ui.start_freq.editingFinished.connect(self.freq_changed)
        ui.stop_freq.editingFinished.connect(self.freq_changed)
        ui.centre_freq.valueChanged.connect(lambda: self.freq_changed(True))  # centre/span mode
        ui.span_freq.valueChanged.connect(lambda: self.freq_changed(True))  # centre/span mode

        self.setAbort(True)

        self.fifoTimer.start(500)  # calls self.usbSend() every 500mS to execute serial commands whilst not scanning

    def restoreSettings(self):
        # update centre freq, span, auto points and graph for the start/stop freqs loaded from database
        self.freq_changed(False)  # start/stop mode
        pointsChanged()
        ui.graphWidget.setXRange(ui.start_freq.value() * 1e6, ui.stop_freq.value() * 1e6, padding=0)
        logging.debug(f'restoreSettings(): band = {numbers.tm.record(0).value("band")}')

        # update trace and marker settings from the database.  1 = last saved (default) settings
        S1.dLoad(1)
        S2.dLoad(1)
        S3.dLoad(1)
        S4.dLoad(1)
        S1.vline.setValue(numbers.tm.record(0).value('m1f'))
        S2.vline.setValue(numbers.tm.record(0).value('m2f'))
        S3.vline.setValue(numbers.tm.record(0).value('m3f'))
        S4.vline.setValue(numbers.tm.record(0).value('m4f'))
        #  set each marker to a different colour
        S1.vline.label.setColor('y')
        S2.vline.setPen(color='m', width=0.75, style=QtCore.Qt.DashLine)
        S2.vline.label.setColor('m')
        S3.vline.setPen(color='c', width=0.75, style=QtCore.Qt.DashLine)
        S3.vline.label.setColor('c')
        S4.vline.setPen(color='w', width=0.75, style=QtCore.Qt.DashLine)
        S4.vline.label.setColor('w')

        setPreferences()
        ui.updates.setText(str(int(1000/(preferences.intervalBox.value()))))  # the display update frequency indicator
        ui.band_box.setCurrentText(numbers.tm.record(0).value("band"))  # this shouldn't be needed but it is

    def scan(self):  # called by 'run' button
        if self.usb is not None:
            if self.sweeping:  # if it's running, stop it
                self.sweeping = False  # tells the measurement thread to stop once current scan complete
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
        frequencies, readings, maxima = self.set_arrays()
        self.sweep = Worker(self.measurement, frequencies, readings, maxima)  # workers auto-deleted when thread stops
        self.sweeping = True
        self.createTimeSpectrum(frequencies, readings)
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
        maxima = np.full(points, -120, dtype=float)
        frequencies = np.linspace(startF, stopF, points, dtype=np.int64)
        # logging.info(f'set_arrays: frequencies = {frequencies}')
        readings = np.full((self.scanMemory, points), None, dtype=float)
        readings[0] = -120
        return frequencies, readings, maxima

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
            S1.bline.setValue((startF + ui.span_freq.value()/20) * 1e6)
            S2.bline.setValue((stopF - ui.span_freq.value()/20) * 1e6)
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
            logging.info('frequency offset error, check preferences')
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

    def measurement(self, frequencies, readings, maxima):  # runs in a separate thread
        updateTimer = QtCore.QElapsedTimer()
        points = np.size(frequencies)
        self.threadRunning = True
        firstRun = True
        version = int(self.firmware[2])  # just the firmware version number
        # self.runTimer.start()  # debug
        # logging.info(f'elapsed time = {self.runTimer.nsecsElapsed()/1e6:.3f}mS')  # debug

        updateTimer.start()  # used to trigger the signal that sends measurements to updateGUI()
        while self.sweeping:
            if preferences.freqLO != 0:
                startF, stopF = self.freqOffset(frequencies)
                command = f'scanraw {int(startF)} {int(stopF)} {int(points)} 3\r'
            else:
                command = f'scanraw {int(frequencies[0])} {int(frequencies[-1])} {int(points)} 3\r'
            self.usb.timeout = self.sweepTimeout(frequencies)
            if version < 177 or firstRun:
                # firmware versions before 4.177 don't support auto-repeating scanraw so command must be sent each sweep
                try:
                    self.usb.write(command.encode())
                    self.usb.read_until(command.encode() + b'\n{')  # skip command echo
                    dataBlock = ''
                except serial.SerialException:
                    logging.info('serial port exception')
                    self.sweeping = False
                    break
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
                if point == points - 1:  # it's the final point of this sweep
                    readingsMax = np.nanmax(readings[:self.scanMemory], axis=0)
                    maxima = np.fmax(maxima, readingsMax)
                    readings[-1] = readings[0]  # populate last row with current sweep before rolling
                    readings = np.roll(readings, 1, axis=0)  # readings row 0 is now full: roll it down 1 row
                    if version >= 177:
                        firstRun = False
                        if self.usb.read(2) != b'}{':  # the end of scan marker character is '}{'
                            logging.info('QtTinySA display is out of sync with tinySA frequency')
                            self.sweeping = False
                            break
                if self.fifo.qsize() > 0 or not self.sweeping:  # a setting has been changed by the user
                    self.serialWrite('abort\r')
                    self.clearBuffer()
                    firstRun = True
                    self.setRBW()
                    frequencies, readings, maxima = self.set_arrays()
                    points = np.size(frequencies)
                    self.createTimeSpectrum(frequencies, readings)
                    self.usbSend()  # send all the queued commands in the FIFO buffer to the TinySA
                    updateTimer.start()
                    break
                timeElapsed = updateTimer.nsecsElapsed()  # how long the thread has been running, nS
                if timeElapsed/1e6 > preferences.intervalBox.value():
                    self.signals.result.emit(frequencies, readings, maxima, timeElapsed)  # send to updateGUI()
                    updateTimer.start()
        self.usb.read(2)  # discard the command prompt
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

    def updateGUI(self, frequencies, readings, maxima, runtime):  # called by a signal from the measurement() thread
        # for LNB/Mixer mode when LO is above measured freq the scan is reversed, i.e. low TinySA freq = high meas freq
        if preferences.highLO.isChecked() and preferences.freqLO != 0:
            frequencies = frequencies[::-1]
            np.fliplr(readings)

        # calculate the average and min trace values
        readingsAvg = np.nanmean(readings[0:ui.avgBox.value()], axis=0)
        readingsMin = np.nanmin(readings[:self.scanMemory], axis=0)
        logging.debug(f'sigProcess: averages={readingsAvg}')

        # update graph axes if in zero span
        if frequencies[0] == frequencies[-1]:
            ui.graphWidget.setLabel('bottom', 'Time')
            frequencies = np.arange(1, len(frequencies) + 1, dtype=int)
            ui.graphWidget.setXRange(frequencies[0], frequencies[-1])

        # update the swept traces
        options = {'Normal': readings[0], 'Average': readingsAvg, 'Max': maxima, 'Min': readingsMin}
        S1.trace.setData(frequencies, options.get(S1.traceType))
        S2.trace.setData(frequencies, options.get(S2.traceType))
        S3.trace.setData(frequencies, options.get(S3.traceType))
        S4.trace.setData(frequencies, options.get(S4.traceType))

        # update markers if not in zero span (where they are not relevant)
        if frequencies[0] != frequencies[-1]:
            ui.graphWidget.setLabel('bottom', units='Hz')
            maxmin = self.maxMin(frequencies, readings)
            S1.updateMarker(frequencies, readings[0, :], maxmin)
            S2.updateMarker(frequencies, readings[0, :], maxmin)
            S3.updateMarker(frequencies, readings[0, :], maxmin)
            S4.updateMarker(frequencies, readings[0, :], maxmin)

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

    def maxMin(self, frequencies, readings):  # finds the signal max/min values for setting markers
        avg = np.nanmean(readings[:ui.avgBox.value()], axis=0)
        avg = np.ma.masked_where(frequencies < S1.bline.value(), avg)
        avg = np.ma.masked_where(frequencies > S2.bline.value(), avg)
        avg = np.ma.masked_where(avg <= S4.hline.value(), avg)  # mask all below threshold
        avgMin = avgMax = avg
        # calculate a frequency width factor to use to mask readings near each max/min frequency
        if ui.rbw_auto.isChecked():
            fWidth = preferences.rbw_x.value() * 850 * 1e3
        else:
            fWidth = preferences.rbw_x.value() * float(ui.rbw_box.currentText()) * 1e3
        maxi = [np.argmax(avgMax)]  # the index of the highest peak in the masked averaged readings array
        mini = [np.argmin(avgMin)]  # the index of the deepest minimum in the masked averaged readings array
        for i in range(3):
            # mask frequencies around detected peaks and find the next 3 highest/lowest peaks
            avgMax = np.ma.masked_where(np.abs(frequencies[maxi[-1]] - frequencies) < fWidth, avgMax)
            maxi.append(np.argmax(avgMax))
            avgMin = np.ma.masked_where(np.abs(frequencies[mini[-1]] - frequencies) < fWidth, avgMin)
            mini.append(np.argmin(avgMin))
        return (list(frequencies[maxi]), list(frequencies[mini]))

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


class display:
    def __init__(self, name, pen):
        self.name = name
        self.trace = ui.graphWidget.plot([], [], name=name, pen=pen, width=1, padding=0)
        self.traceType = 'Normal'  # Normal, Average, Max, Min
        self.markerType = 'Normal'  # Normal, Delta; Max, Min
        self.vline = ui.graphWidget.addLine(88, 90, movable=True, name=name,
                                            pen=pyqtgraph.mkPen('y', width=0.5, style=QtCore.Qt.DashLine),
                                            label="{value:.5f}")
        self.hline = ui.graphWidget.addLine(y=0, movable=False, pen=red_dash, label='',
                                            labelOpts={'position': 0.025, 'color': ('r')})
        self.bline = ui.graphWidget.addLine(-10, -10, movable=True, name=name,
                                            pen=pyqtgraph.mkPen('r', width=0.5, style=QtCore.Qt.DashLine),
                                            label="bound", labelOpts={'position': 0.025, 'color': ('r'), 'movable': True})
        self.deltaF = 0  # the difference between this marker and Reference Marker (1)
        self.fifo = queue.SimpleQueue()
        self.vline.sigClicked.connect(self.mClicked)

    def mStart(self):
        # set marker to the sweep start frequency
        if self.guiRef(0).isChecked():
            self.vline.setValue(ui.start_freq.value() * 1e6)
            self.mType()

    def mSpread(self):
        # spread markers equally across scan range
        if self.guiRef(0).isChecked():
            self.vline.setValue(ui.start_freq.value() * 1e6 + (0.2 * int(self.name) * ui.span_freq.value() * 1e6))
            self.mType()

    # def mSpread(self):
        # spread markers across scan range
        # mOn = [ui.marker1.isChecked(), ui.marker2.isChecked(), ui.marker3.isChecked(), ui.marker4.isChecked()]
        # mcount = np.count_nonzero(mOn)
        # for i in range(4):
        #     if mOn[i + 1] and mcount == 1

        # if self.guiRef(0).isChecked():
        #     self.vline.setValue(ui.start_freq.value() + (0.5 * ui.span_freq.value())/mcount)
        #     self.mType()

    def mClicked(self):
        ui.centre_freq.setValue(self.vline.value() / 1e6)
        tinySA.freq_changed(True)

    def mType(self):
        self.markerType = self.guiRef(1).currentText()  # current combobox value from appropriate GUI field
        if self.markerType == 'Delta':
            self.deltaF = self.vline.value() - S1.vline.value()
            self.vline.label.setText(f'M{self.vline.name()} {chr(916)}{self.deltaF:.5f}MHz')
        if {'Max', 'Min'}.intersection({S1.markerType[:3], S2.markerType[:3], S3.markerType[:3], S4.markerType[:3]}):
            S4.hline.show()  # the peak detection threshold line
            S1.bline.show()  # the boundary markers
            S2.bline.show()
        else:
            S4.hline.hide()
            S1.bline.hide()
            S2.bline.hide()

    def mDelta(self):  # delta marker locking to reference marker S1
        if self.markerType == 'Delta':
            self.vline.setValue(S1.vline.value() + self.deltaF)
            S1.vline.setPen(color='y', width=1.0)

    def dLoad(self, setting):
        self.vline.label.setMovable(True)
        self.mEnable()
        self.mType()
        self.tType()
        self.tEnable()
        S1.hEnable(preferences.neg25Line)
        S2.hEnable(preferences.zeroLine)
        S3.hEnable(preferences.plus6Line)

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

    def updateMarker(self, frequencies, readings, maxmin):  # called by updateGUI()
        options = {'Max1': maxmin[0][0], 'Max2': maxmin[0][1], 'Max3': maxmin[0][2],
                   'Max4': maxmin[0][3], 'Normal': self.vline.value(), 'Delta': self.vline.value(),
                   'Min1': maxmin[1][0], 'Min2': maxmin[1][1], 'Min3': maxmin[1][2],
                   'Min4': maxmin[1][3]}
        markerF = options.get(self.markerType)
        if markerF < np.min(frequencies) or markerF > np.max(frequencies):
            # marker is out of scan range so just show its frequency
            self.vline.label.setText(f'M{self.vline.name()} {self.vline.value()/1e6:.{5}f}')
        else:
            # marker is in scan range
            fIndex = np.argmin(np.abs(frequencies - (markerF)))  # find closest value in freq array
            dBm = readings[fIndex]
            if dBm > S4.hline.value() or self.markerType[:4] == 'Normal' or self.markerType[:4] == 'Delta':
                self.vline.setValue(frequencies[fIndex])  # set to the discrete value from frequencies[]
            if self.markerType == 'Delta':
                self.vline.label.setText(f'M{self.vline.name()} {chr(916)}{self.deltaF/1e6:.{5}f} {dBm:.1f}dBm')
            else:
                self.vline.label.setText(f'M{self.vline.name()} {self.vline.value()/1e6:.{5}f} {dBm:.1f}dBm')

    def addFreqMarker(self, freq, colour, name, band=True):  # adds simple freq marker without full marker capability
        if ui.presetLabel.isChecked():
            if band:
                self.marker = ui.graphWidget.addLine(freq, 90, pen=pyqtgraph.mkPen(colour, width=0.5,
                                                     style=QtCore.Qt.DashLine), label=name, labelOpts={'position': 0.97,
                                                     'color': (colour)})
            else:
                self.marker = ui.graphWidget.addLine(freq, 90, pen=pyqtgraph.mkPen(colour, width=0.5,
                                                     style=QtCore.Qt.DashLine), label=name, labelOpts={'position': 0.06,
                                                     'color': (colour), 'anchors': ((0, 0.2), (0, 0.2))})
            self.marker.label.setMovable(True)
        else:
            self.marker = ui.graphWidget.addLine(freq, 90, pen=pyqtgraph.mkPen(colour, width=0.5, style=QtCore.Qt.DashLine))
        self.fifo.put(self.marker)  # store the marker object in a queue

    def delFreqMarkers(self):
        for i in range(0, self.fifo.qsize()):
            ui.graphWidget.removeItem(self.fifo.get())  # remove the marker and its corresponding object in the queue


class WorkerSignals(QtCore.QObject):
    error = QtCore.pyqtSignal(str)
    result = QtCore.pyqtSignal(np.ndarray, np.ndarray, np.ndarray, float)
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

    def __init__(self, dbName):
        self.db = None
        self.dbName = dbName
        self.personalDir = platformdirs.user_config_dir(appname=app.applicationName(), appauthor=False)
        self.globalDir = platformdirs.site_config_dir(appname=app.applicationName(), appauthor=False)
        self.workingDirs = [os.path.dirname(__file__), os.path.dirname(os.path.realpath(__file__)), os.getcwd()]
        self.dbpath = self._getPersonalisedPath()

    def _getPersonalisedPath(self):
        if os.path.exists(os.path.join(self.personalDir, self.dbName)):  # check if personal config database file exists
            logging.info(f'Database {self.dbName} found at {self.personalDir}')
            return self.personalDir
        if not os.path.exists(self.personalDir):
            os.mkdir(self.personalDir)
        if os.path.exists(os.path.join(self.globalDir, self.dbName)):
            logging.info(f'Database {self.dbName} found at {self.globalDir}')
            shutil.copy(os.path.join(self.globalDir, self.dbName), self.personalDir)
            logging.info(f'Database {self.dbName} copied from {self.globalDir} to {self.personalDir}')
            return self.personalDir
        logging.info(f'No database file {self.dbName} exists in {self.personalDir} or {self.globalDir}')
        # Look in current working folder & where the python file is stored/linked from
        if self.dbName == "QtTSAprefs.db":
            for workingDir in self.workingDirs:
                if os.path.exists(os.path.join(workingDir, self.dbName)):
                    shutil.copy(os.path.join(workingDir, self.dbName), self.personalDir)
                    logging.info(f'{self.dbName} copied from {workingDir} to {self.personalDir}')
                    return self.personalDir
            raise FileNotFoundError("Unable to find the database {self.dbName}")

    def connect(self):
        self.db = QSqlDatabase.addDatabase('QSQLITE')
        if QtCore.QFile.exists(os.path.join(self.dbpath, self.dbName)):
            self.db.setDatabaseName(os.path.join(self.dbpath, self.dbName))
            self.db.open()
            logging.info(f'{self.dbName} open: {self.db.isOpen()}')
            # self.db.setConnectOptions('PRAGMA foreign_keys = ON;')
        else:
            logging.info('Database file {self.dbpath}{self.dbName} is missing')
            popUp('Database file is missing', QMessageBox.Ok, QMessageBox.Critical)

    def disconnect(self):
        self.db.close()
        logging.info(f'Database {self.dbName} open: {self.db.isOpen()}')
        QSqlDatabase.removeDatabase(QSqlDatabase.database().connectionName())


class modelView():
    '''set up and process data models bound to the GUI widgets'''

    def __init__(self, tableName):
        self.tableName = tableName
        self.tm = QSqlRelationalTableModel()
        self.dwm = QDataWidgetMapper()
        self.currentRow = 0
        self.currentBand = 0

    def createTableModel(self):
        self.tm.setTable(self.tableName)
        self.dwm.setModel(self.tm)
        self.dwm.setSubmitPolicy(QDataWidgetMapper.AutoSubmit)

    def addRow(self):  # adds a blank row to the frequency bands table widget above current row
        if self.currentRow == 0:
            self.tm.insertRow(0)
        else:
            self.tm.insertRow(self.currentRow)
        preferences.freqBands.selectRow(self.currentRow)

    def saveChanges(self):
        self.dwm.submit()

    def deleteRow(self, single=True):  # deletes rows in the frequency bands table widget
        if single:
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


def addBandPressed():
    if not ui.marker1.isChecked():
        message = 'Please enable Marker 1'
        popUp(message, QMessageBox.Ok, QMessageBox.Information)
        return
    if ui.marker1.isChecked() and ui.marker2.isChecked():  # Two markers are to set the band limits of a new band
        if S1.vline.value() >= S2.vline.value():
            message = 'M1 frequency >= M2 frequency'
            popUp(message, QMessageBox.Ok, QMessageBox.Information)
            return
        ID = presetID(str(ui.filterBox.currentText()))
        title = "New Frequency Band"
        message = "Enter a name for the new band."
        bandName, ok = QInputDialog.getText(None, title, message, QLineEdit.Normal, "")
        bands.insertData(name=bandName, preset=ID, startF=f'{S1.vline.value()}',
                         stopF=f'{S2.vline.value()/1e6:.6f}', value=1, colour=colourID('green'))  # colourID(value)
    else:  # If only Marker 1 is enabled then this creates a spot Frequency marker
        title = "New Spot Frequency Marker"
        message = "Enter a name for the Spot Frequency"
        spotName, ok = QInputDialog.getText(None, title, message, QLineEdit.Normal, "")
        bands.insertData(name=spotName, type=12, startF=f'{S1.vline.value()}',
                         stopF='', visible=1, colour=colourID('orange'))  # preset 12 is Marker (spot frequency).


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


def setPreferences():  # called when the preferences window is closed
    checkboxes.dwm.submit()
    bands.tm.submitAll()
    S4.hline.setValue(preferences.peakThreshold.value())
    # bandselect.filterType(False, ui.filterBox.currentText())
    if ui.presetMarker.isChecked():
        freqMarkers()
    isMixerMode()


def dialogPrefs():  # called by clicking on the setup > preferences menu
    bands.filterType(True, preferences.filterBox.currentText())
    # bands.tm.select()  # stopping marker add
    bands.currentRow = 0
    preferences.freqBands.selectRow(bands.currentRow)
    pwindow.show()


def about():
    message = ('TinySA Ultra GUI programme using Qt5 and PyQt\nAuthor: Ian Jefferson G4IXT\n\nVersion: {} \nConfig: {}'
               .format(app.applicationVersion(), config.dbpath))
    popUp(message, QMessageBox.Ok, QMessageBox.Information)


def clickEvent():
    logging.info('clickEvent')


def testComPort():
    index = preferences.deviceBox.currentIndex()
    tinySA.testPort(tinySA.ports[index - 1])  # allow for 'select device' entry

##############################################################################
# other methods


def exit_handler():
    numbers.dwm.submit()
    checkboxes.dwm.submit()
    if len(tinySA.ports) != 0:
        # save the marker frequencies
        record = numbers.tm.record(0)
        record.setValue('m1f', float(S1.vline.value()))
        record.setValue('m2f', float(S2.vline.value()))
        record.setValue('m3f', float(S3.vline.value()))
        record.setValue('m4f', float(S4.vline.value()))
        numbers.tm.setRecord(0, record)
        # save the gui field values and checkbox states
        if tinySA.sweeping:
            tinySA.sweeping = False  # tell the measurement thread to stop
            while tinySA.threadRunning:
                time.sleep(0.1)  # wait for measurements to stop
        tinySA.resume()
        tinySA.usbSend()
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
    S1.delFreqMarkers()
    S2.delFreqMarkers()
    for i in range(0, presetmarker.tm.rowCount()):
        try:
            startF = presetmarker.tm.record(i).value('StartF')
            stopF = presetmarker.tm.record(i).value('StopF')
            colour = presetmarker.tm.record(i).value('colour')
            name = presetmarker.tm.record(i).value('name')
            if ui.presetMarker.isChecked() and presetmarker.tm.record(i).value('visible') and stopF in (0, ''):
                S1.addFreqMarker(startF, colour, name, band=False)
                if ui.presetLabel.isChecked() and ui.presetLabel.checkState() == 2:
                    S1.marker.label.setAngle(90)
            if ui.presetMarker.isChecked() and stopF not in (0, ''):  # it's a band marker
                S1.addFreqMarker(startF, colour, name)
                S2.addFreqMarker(stopF, colour, name)
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


###############################################################################
# Instantiate classes

tinySA = analyser()

app = QtWidgets.QApplication([])  # create QApplication for the GUI
app.setApplicationName('QtTinySA')
app.setApplicationVersion(' v0.11.8')
window = QtWidgets.QMainWindow()
ui = QtTinySpectrum.Ui_MainWindow()
ui.setupUi(window)

pwindow = QtWidgets.QDialog()  # pwindow is the preferences dialogue box
preferences = QtTSApreferences.Ui_Preferences()
preferences.setupUi(pwindow)

fwindow = QtWidgets.QDialog()  # fwindow is the filebrowse dialogue box
filebrowse = QtTSAfilebrowse.Ui_Filebrowse()
filebrowse.setupUi(fwindow)

# Traces & markers
S1 = display('1', yellow)
S2 = display('2', magenta)
S3 = display('3', cyan)
S4 = display('4', white)

# Data models for configuration settings
config = database("QtTSAprefs.db")
config.connect()
checkboxes = modelView('checkboxes')
numbers = modelView('numbers')
markers = modelView('marker')
traces = modelView('trace')
tracetext = modelView('combo')
markertext = modelView('combo')
rbwtext = modelView('combo')
bandstype = modelView('freqtype')
colours = modelView('SVGColour')
maps = modelView('mapping')
bands = modelView('frequencies')
presetmarker = modelView('frequencies')
bandselect = modelView('frequencies')

###############################################################################
# GUI settings

# pyqtgraph settings for spectrum display
ui.graphWidget.setYRange(-110, -20)
ui.graphWidget.setDefaultPadding(padding=0)
ui.graphWidget.showGrid(x=True, y=True)
ui.graphWidget.setLabel('bottom', '', units='Hz')

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
# Connect signals from buttons and sliders.  Some connections are in 'initialise' Fn

ui.scan_button.clicked.connect(tinySA.scan)
ui.run3D.clicked.connect(tinySA.scan)
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
ui.presetMarker.clicked.connect(freqMarkers)
ui.presetLabel.clicked.connect(freqMarkerLabel)
ui.mToBand.clicked.connect(addBandPressed)
ui.filterBox.currentTextChanged.connect(freqMarkers)


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
preferences.neg25Line.stateChanged.connect(lambda: S1.hEnable(preferences.neg25Line))
preferences.zeroLine.stateChanged.connect(lambda: S2.hEnable(preferences.zeroLine))
preferences.plus6Line.stateChanged.connect(lambda: S3.hEnable(preferences.plus6Line))
preferences.addRow.clicked.connect(bands.addRow)
preferences.deleteRow.clicked.connect(lambda: bands.deleteRow(True))
preferences.deleteAll.clicked.connect(lambda: bands.deleteRow(False))
preferences.freqBands.clicked.connect(bands.tableClicked)
preferences.filterBox.currentTextChanged.connect(lambda: bands.filterType(True, preferences.filterBox.currentText()))
ui.filterBox.currentTextChanged.connect(lambda: bandselect.filterType(False, ui.filterBox.currentText()))
ui.actionPreferences.triggered.connect(dialogPrefs)  # open preferences dialogue when its menu is clicked
ui.actionAbout_QtTinySA.triggered.connect(about)
pwindow.finished.connect(setPreferences)  # update database checkboxes table on dialogue window close
preferences.exportButton.pressed.connect(exportData)
preferences.importButton.pressed.connect(importData)
preferences.deviceBox.activated.connect(testComPort)

# filebrowse
ui.actionBrowse_TinySA.triggered.connect(tinySA.dialogBrowse)
filebrowse.download.clicked.connect(tinySA.fileDownload)
filebrowse.listWidget.itemClicked.connect(tinySA.fileShow)

# Quit
ui.actionQuit.triggered.connect(app.closeAllWindows)

# Sweep time
# ui.sweepTime.valueChanged.connect(lambda: tinySA.sweepTime(ui.sweepTime.value()))


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

# set GUI fields using values from the configuration database
tinySA.restoreSettings()

window.show()
window.setWindowTitle(app.applicationName() + app.applicationVersion())
# window.setWindowIcon(QtGui.QIcon(os.path.join(basedir, 'tinySAsmall.png')))

# try to open a USB connection to the TinySA hardware
tinySA.usbCheck.start(500)  # check again every 500mS

###############################################################################
# run the application until the user closes it

try:
    app.exec()
finally:
    exit_handler()  # close cleanly
