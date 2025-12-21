#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright 2025 Ian Jefferson G4IXT
# SPDX-License-Identifier: GPL-3.0-or-later

# Compilation mode, support OS-specific options
# nuitka-project-if: {OS} in ("Windows", "Linux", "Darwin", "FreeBSD"):
#    nuitka-project: --mode=standalone
#    nuitka-project: --enable-plugin=pyqt6
#    nuitka-project: --include-qt-plugins=sqldrivers
#    nuitka-project: --include-data-file=QtTSAprefs.db=./
#    nuitka-project: --remove-output
# nuitka-project-else:
#    nuitka-project: --mode=standalone

"""TinySA GUI programme using Qt, PyQt and PyQtGraph.

This code attempts to replicate some of the TinySA Ultra on-screen commands and to provide PC control.
Development took place on Kubuntu 24.04LTS with Python 3.11 and PyQt6 using Spyder in Anaconda.
TinySA, TinySA Ultra and the tinysa icon are trademarks of Erik Kaashoek and are used with permission.
TinySA commands are based on Erik's Python examples: http://athome.kaashoek.com/tinySA/python/
Serial communication commands are based on Martin's Python NanoVNA/TinySA Toolset: https://github.com/Ho-Ro"""

import os
import sys
import time
import logging
import struct
import serial
from platform import system
import threading

import sdr_audio as sdr

try:
    from PyQt6 import QtWidgets, QtCore, uic
    from PyQt6.QtWidgets import QMessageBox, QDataWidgetMapper, QFileDialog, QInputDialog, QLineEdit, QTableWidgetItem
    from PyQt6.QtSql import QSqlDatabase, QSqlRelation, QSqlRelationalTableModel, QSqlRelationalDelegate, QSqlQuery
    from PyQt6.QtGui import QPixmap, QIcon
    from PyQt6.QtGui import QAction
except ModuleNotFoundError:
    from PyQt5 import QtWidgets, QtCore, uic
    from PyQt5.QtWidgets import QMessageBox, QDataWidgetMapper, QFileDialog, QInputDialog, QLineEdit, QTableWidgetItem
    from PyQt5.QtSql import QSqlDatabase, QSqlRelation, QSqlRelationalTableModel, QSqlRelationalDelegate, QSqlQuery
    from PyQt5.QtGui import QPixmap, QIcon

import queue
import shutil
import platformdirs
import csv
import numpy as np
import pyqtgraph
import pyqtgraph.opengl as pyqtgl  # For 3D
from datetime import datetime
from serial.tools import list_ports
from io import BytesIO
from QtTinyExporters import WWBExporter, WSMExporter

# Defaults to non local configuration/data dirs - needed for packaging
if system() == "Linux":
    os.environ['XDG_CONFIG_DIRS'] = '/etc:/usr/local/etc'
    os.environ['XDG_DATA_DIRS'] = '/usr/share:/usr/local/share'


# force Qt to use OpenGL rather than DirectX for Windows OS
QtCore.QCoreApplication.setAttribute(QtCore.Qt.ApplicationAttribute.AA_UseDesktopOpenGL)

logging.basicConfig(format="%(message)s", level=logging.INFO)
threadpool = QtCore.QThreadPool()
basedir = os.path.dirname(__file__)

# pyqtgraph custom exporters
WWBExporter.register()
WSMExporter.register()

# classes ##############################################################################


class CustomDialogue(QtWidgets.QDialog):
    def __init__(self, ui_name):
        super().__init__()
        self.ui = uic.loadUi(ui_name, self)
        self.ui.setWindowIcon(QIcon(os.path.join(basedir, 'tinySAsmall.png')))


class Analyser:
    def __init__(self):
        self.sdr_active = False # New attribute to track SDR state
        self.sdr_thread = None  # To hold the thread for SDR playback
        self.last_sdr_freq_sent = 0 # New attribute to store the last frequency sent to SDR
        self.sdr_freq_threshold = 1000 # Retune if frequency changes by more than 1 kHz (adjust as needed)
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
        self.signals.sweepEnds.connect(self.sweepComplete)
        self.runTimer = QtCore.QElapsedTimer()
        self.scale = 174
        self.scanMemory = 50
        self.fifo = queue.SimpleQueue()
        self.maxF = 6000
        self.memF = BytesIO()
        self.ports = []
        
        
    def sdr_audio_plus_freq(self):
        marker1_freq_hz = self.get_marker_frequency(1)
        #marker1_freq_hz += 105250  # add one khz
        frequencies, levels = M1.linked.fetchData() 
        marker1_freq_hz += (frequencies[1] - frequencies[0])
        self.set_marker_frequency(1, marker1_freq_hz)
        #tinySA.update_sdr_from_marker('1', marker1_freq_hz)    
   
    def sdr_audio_minus_freq(self):    
        marker1_freq_hz = self.get_marker_frequency(1)
        #marker1_freq_hz -= 105250  # sub one khz
        frequencies, levels = M1.linked.fetchData() 
        marker1_freq_hz -= (frequencies[1] - frequencies[0])        
        self.set_marker_frequency(1, marker1_freq_hz)
        #tinySA.update_sdr_from_marker('1', marker1_freq_hz) 
        
    def toggle_sdr_audio(self):
        if not self.sdr_active:
            marker1_freq_hz = self.get_marker_frequency(1)
            if marker1_freq_hz is not None:
                print(f"[QtTinySA: INFO] Starting SDR playback at {marker1_freq_hz / 1e6:.3f} MHz\n")
                self.sdr_thread = threading.Thread(target=sdr.start_sdr_playback, args=(marker1_freq_hz,))
                self.sdr_thread.daemon = True # Allow the program to exit even if thread is running
                self.sdr_thread.start()
                sdr_audio_popup.ui.toggle_audio_button.setText(f"Disable Audio Monitoring")
                
                self.sdr_active = True

            else:
                QMessageBox.warning(self, "SDR Error", "Could not retrieve Marker 1 frequency.")
        else:
            print("[QtTinySA: INFO] Stopping SDR playback\n")
            sdr.stop_sdr_playback() # Signal sdr to stop
            if self.sdr_thread and self.sdr_thread.is_alive():
                self.sdr_thread.join(timeout=5) # Wait for the thread to finish
            self.sdr_active = False
            sdr_audio_popup.ui.toggle_audio_button.setText(f"Enable Audio Monitoring")

    def get_marker_frequency(self, marker_number):
        if marker_number == 1:
            return M1.line.value() # This will return the frequency in Hz
        return None
        
    def set_marker_frequency(self, marker_number, mfreq):
        if marker_number == 1:
            M1.line.setValue(mfreq)
        return None


    def openPort(self):  # called by isConnected() triggered by the self.usbCheck QTimer at startup
        # Get tinySA comport using hardware ID
        VID = 0x0483  # 1155
        PID = 0x5740  # 22336
        usbPorts = list_ports.comports()
        for port in usbPorts:
            if port.vid == VID and port.pid == PID:
                if port not in self.ports:
                    settings.deviceBox.addItem(self.identify(port) + " on " + port.device)
                    self.ports.append(port)
        if len(self.ports) == 1:  # found only one device so just test it
            usbCheck.stop()
            self.testPort(self.ports[0])
            return
        if len(self.ports) > 1:  # several devices found
            settings.deviceBox.insertItem(0, "Select device")
            settings.deviceBox.setCurrentIndex(0)
            popUp(QtTSA, "Several devices detected.  Choose device in Settings > Preferences", 'Ok', 'Info')
            usbCheck.stop()

    def testPort(self, port):  # tests comms and initialises tinySA if found
        try:
            self.usb = serial.Serial(port.device, baudrate=576000)
            logging.info(f'Serial port {port.device} open: {self.usb.isOpen()}')
        except serial.SerialException:
            logging.info('Serial port exception. A possible cause is that your username is not in the "dialout" group.')
            popUp(QtTSA, 'Serial port exception', 'Ok', 'Critical')
        if self.usb:
            for i in range(4):  # try 4 times to communicate with tinySA over USB serial
                firmware = self.version()
                if firmware[:6] == 'tinySA':
                    logging.info(f'{port.device} test {i} : {firmware[:16]}')
                    break
                else:
                    time.sleep(1)
            # split firmware into a list of [device, major version number, minor version number, other stuff]
            self.firmware = firmware.replace('_', '-').split('-')
            if firmware[:6] == 'tinySA':
                if firmware[0] == 'tinySA4' and float(self.firmware[1][-3:] + self.firmware[2]) < 1.4177:
                    logging.info('for fastest possible scan speed, upgrade firmware to v1.4-177 or later')
                if self.firmware[1][0] == "v":
                    self.setForDevice(self.firmware)
                else:
                    logging.info(f'{port.device} test found unexpected firmware {firmware}')
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
                    # self.usbCheck.stop()
                    usbCheck.stop()
                else:
                    self.openPort()

    def setGUI(self):
        self.setStartFreq()

        # Traces
        T1.setup()
        T2.setup()
        T3.setup()
        T4.setup()

        # Markers
        M1.setup('yellow')
        M2.setup('yellow')
        M3.setup('yellow')
        M4.setup('yellow')

        # set various defaults
        setPreferences()
        bandselect.filterType(False, QtTSA.filterBox.currentText())  # setting the filter overwrites the band
        setSize()

        # now connect GUI controls that would interfere with restoration of data at startup
        connectActive()

        # call self.usbSend() every 200mS to send commands & update markers if not scanning
        self.fifoTimer = QtCore.QTimer()
        self.fifoTimer.timeout.connect(self.timerTasks)
        self.fifoTimer.start(200)

    def setForDevice(self, product):
        # product = 'tinySA'  # used for testing
        logging.debug('setForDevice: started')
        if product[0] == 'tinySA4':  # It's an Ultra
            self.tinySA4 = True
            self.maxF = settings.maxFreqBox.value()
            self.scale = 174
            QtTSA.spur_box.setCurrentText(checkboxes.tm.record(0).value("spur"))
        else:
            self.tinySA4 = False  # It's a Basic
            self.maxF = 960
            self.scale = 128
            rbwtext.tm.setFilter('type = "rbw" and value != "0.2" and value != "1" and value != "850"')  # fewer RBWs

        # Basic has no lna
        QtTSA.lna_box.setVisible(self.tinySA4)
        QtTSA.lna_box.setEnabled(self.tinySA4)

        self.setTime()
        self.setAbort(True)

        # show device information in GUI
        QtTSA.battery.setText(self.battery())
        QtTSA.version.setText(product[0] + " " + product[1] + " " + product[2])

        # self.fifoTimer.start(200)  # call self.usbSend() every 200mS to commands & update markers when scan is stopped

        logging.debug('setForDevice:: finished')

    def scan(self):  # called by 'run' button
        logging.debug(f'scan: self.usb = {self.usb}')
        if self.usb is not None:
            if self.sweeping:  # if it's running, stop it
                self.sweeping = False  # tells the measurement thread to stop once current scan complete
                QtTSA.scan_button.setEnabled(False)  # prevent repeat presses of 'stop'
                QtTSA.run3D.setEnabled(False)
            else:
                try:  # start measurements
                    self.fifoTimer.stop()
                    self.clearBuffer()
                    self.attenuator()
                    self.lna()
                    self.spur()
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
            popUp(QtTSA, 'TinySA not found', 'Ok', 'Critical')

    def startMeasurement(self):
        frequencies, readings, maxima, minima = self.set_arrays()
        self.sweep = Worker(self.measurement, frequencies, readings, maxima, minima)  # workers deleted when thread ends
        self.sweeping = True
        self.createTimeSpectrum(frequencies, readings)
        self.createWaterfall(frequencies, readings)
        self.reset3D()
        threadpool.start(self.sweep)

    def timerTasks(self):
        if self.usb:
            self.usbSend()
        M1.updateMarker()
        M2.updateMarker()
        M3.updateMarker()
        M4.updateMarker()

    def usbSend(self):
        while self.fifo.qsize() > 0:
            command = self.fifo.get(block=True, timeout=None)
            self.serialWrite(command)

    def serialQuery(self, command):
        self.usb.write(command.encode())
        self.usb.read_until(command.encode() + b'\n')  # skip command echo
        response = self.usb.read_until(b'ch> ')  # until prompt
        logging.debug(f'serialQuery: response = {response}')
        return response[:-6].decode()  # remove prompt

    def serialWrite(self, command):
        # self.usb.timeout = 1
        logging.debug(f'serialWrite: command = {command}')
        self.usb.write(command.encode())
        self.usb.read_until(b'ch> ')  # skip command echo and prompt

    def set_arrays(self):
        startF = QtTSA.start_freq.value() * 1e6  # freq in Hz
        stopF = QtTSA.stop_freq.value() * 1e6
        points = self.setPoints()
        maxima = np.full(points, -200, dtype=float)
        minima = np.full(points, 0, dtype=float)
        frequencies = np.linspace(startF, stopF, points, dtype=np.int64)
        logging.debug(f'set_arrays: frequencies = {frequencies}')
        readings = np.full((self.scanMemory, points), None, dtype=float)
        readings[0] = -200
        # signal fading QtTSA.  timeMarkVals is a 2D array with time in col 0 and dBm levels of each marker in 1 - 4
        self.timeMarkVals = np.full((int(settings.timePoints.value()), 5), None, dtype=float)
        self.timeIndex = 0
        return frequencies, readings, maxima, minima

    def setCentreFreq(self):
        startF = QtTSA.centre_freq.value()-QtTSA.span_freq.value()/2
        stopF = QtTSA.centre_freq.value()+QtTSA.span_freq.value()/2
        QtTSA.start_freq.setValue(startF)
        QtTSA.stop_freq.setValue(stopF)
        self.setGraphFreq(startF, stopF)

    def setStartFreq(self):
        startF = QtTSA.start_freq.value()  # freq in MHz
        stopF = QtTSA.stop_freq.value()
        if startF > stopF:
            stopF = startF
            QtTSA.stop_freq.setValue(stopF)
        QtTSA.centre_freq.setValue(startF + (stopF - startF) / 2)
        QtTSA.span_freq.setValue(stopF - startF)
        self.setGraphFreq(startF, stopF)

    def setGraphFreq(self, startF, stopF):
        QtTSA.graphWidget.setXRange(startF * 1e6, stopF * 1e6)
        if QtTSA.span_freq.value() != 0:
            lowF.line.setValue((startF + QtTSA.span_freq.value()/20) * 1e6)
            highF.line.setValue((stopF - QtTSA.span_freq.value()/20) * 1e6)

    def freq_changed(self, centre=False):
        if centre:
            self.setCentreFreq()
        else:
            self.setStartFreq()
        self.resume()  # puts a message in the fifo buffer so the measurement thread spots it and updates its settings

    def freqOffset(self, frequencies):  # for mixers or LNBs external to TinySA
        startF = frequencies[0]
        spanF = frequencies[-1] - startF
        loF = bandstype.freq
        if bandstype.freq > frequencies[0]:  # LNB LO is higher in freq than wanted signal
            scanF = (loF - startF - spanF, loF - startF)
        else:
            scanF = (startF - loF, startF - loF + spanF)
        if min(scanF) < 0:
            self.sweeping = False
            scanF = (88 * 1e6, 108 * 1e6)
            logging.info('LO frequency offset error, check settings')
            popUp(QtTSA, "LO frequency offset error, check settings", 'Ok', 'Critical')
        logging.debug(f'freqOffset(): scanF = {scanF}')
        return scanF

    def rbwChanged(self):
        if QtTSA.rbw_auto.isChecked():  # can't calculate Points because we don't know what the RBW will be
            QtTSA.rbw_box.setEnabled(False)
            QtTSA.points_auto.setChecked(False)
            QtTSA.points_auto.setEnabled(False)
        else:
            QtTSA.rbw_box.setEnabled(True)
            QtTSA.points_auto.setEnabled(True)
        self.setRBW()  # if measurement thread is running, calling setRBW() will force it to update scan parameters

    def setRBW(self):  # may be called by measurement thread as well as normally
        if QtTSA.rbw_auto.isChecked():
            rbw = 'auto'
        else:
            rbw = QtTSA.rbw_box.currentText()  # ui values are discrete ones in kHz
        logging.debug(f'rbw = {rbw}')
        command = f'rbw {rbw}\r'
        self.fifo.put(command)

    def setPoints(self):  # may be called by measurement thread as well as normally
        if QtTSA.points_auto.isChecked():
            rbw = float(QtTSA.rbw_box.currentText())
            points = settings.rbw_x.value() * int((QtTSA.span_freq.value()*1000)/(rbw))  # RBW multiplier * freq in kHz
            points = np.clip(points, settings.minPoints.value(), settings.maxPoints.value())  # limit points
        else:
            points = QtTSA.points_box.value()
            logging.debug(f'setPoints: points = {QtTSA.points_box.value()}')
        return points

    def clearBuffer(self):
        # self.usb.timeout = 1
        while self.usb.inWaiting():
            self.usb.read_all()  # keep the serial buffer clean
            time.sleep(0.01)

    def sweepTimeout(self, frequencies):  # freqs are in Hz
        startF = frequencies[0]
        stopF = frequencies[-1]
        points = np.size(frequencies)
        if QtTSA.rbw_auto.isChecked():
            # rbw auto setting from tinySA: ~7 kHz per 1 MHz scan frequency span
            rbw = (stopF - startF) * 7e-6
        else:
            rbw = float(QtTSA.rbw_box.currentText())
        rbw = np.clip(rbw, 0.2, 850)  # apply limits
        # timeout can be very long - use a heuristic approach
        # 1st summand is the scanning time, 2nd summand is the USB transfer overhead
        timeout = ((stopF - startF) / 20e3) / (rbw ** 2) + points / 500
        if (QtTSA.spur_box.currentText() == 'on' and stopF > 8 * 1e8) or QtTSA.spur_box.currentText() == 'auto':
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
            if bandstype.freq != 0:
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
                    self.signals.sweepEnds.emit(frequencies)

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
                if timeElapsed/1e6 > settings.intervalBox.value():
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
            QtTSA.openGLWidget.clear()
        self.surface = pyqtgl.GLSurfacePlotItem(x=-x, y=y, z=z, shader='heightColor',
                                                computeNormals=QtTSA.glNormals.isChecked(), smooth=QtTSA.glSmooth.isChecked())

        #  for each colour, map = pow(z * colorMap[0] + colorMap[1], colorMap[2])
        self.surface.shader()['colorMap'] = np.array([QtTSA.rMulti.value(),      # red   [0]
                                                      QtTSA.rConst.value(),      # red   [1]
                                                      QtTSA.rExponent.value(),   # red   [2]
                                                      QtTSA.gMulti.value(),      # green [3]
                                                      QtTSA.gConst.value(),      # green [4]
                                                      QtTSA.gExponent.value(),   # green [5]
                                                      QtTSA.bMulti.value(),      # blue  [6]
                                                      QtTSA.bConst.value(),      # blue  [7]
                                                      QtTSA.gExponent.value()])  # blue  [8]

        self.surface.translate(16, -points/40, -8)  # front/back, left/right, up/down
        self.surface.scale(points/1250, 0.05, 0.1, local=True)
        QtTSA.openGLWidget.addItem(self.surface)

        # Add a vertical grid to the 3D view
        self.vGrid = pyqtgl.GLGridItem(glOptions='translucent', color=(255, 255, 255, 70))
        self.vGrid.setSize(x=12, y=points/20, z=1)
        self.vGrid.rotate(90, 0, 1, 0)
        self.vGrid.setSpacing(1, 1, 2)
        QtTSA.openGLWidget.addItem(self.vGrid)
        if QtTSA.grid.isChecked():
            self.vGrid.show()
        else:
            self.vGrid.hide()

    def createWaterfall(self, frequencies, readings):
        self.waterfall = pyqtgraph.ImageItem(axisOrder='row-major')
        QtTSA.waterfall.addItem(self.waterfall)
        QtTSA.waterfall.setYRange(0, QtTSA.memBox.value())
        # QtTSA.waterfall.setXLink(QtTSA.graphWidget)

        # Histogram associated with waterfall
        self.histogram = pyqtgraph.HistogramLUTItem(gradientPosition='right', orientation='vertical')
        self.histogram.setImageItem(self.waterfall)  # connects the histogram to the waterfall
        self.histogram.gradient.loadPreset('plasma')  # set the colour map
        self.waterfall.setLevels((-100, -25))  # needs to be here, after histogram is created
        QtTSA.histogram.addItem(self.histogram)

    def updateWaterfall(self, frequencies, readings):
        if QtTSA.waterfall_size.value() > 0:
            QtTSA.waterfall.setXRange(0, np.size(readings, axis=1))
            self.waterfall.setImage(readings, autoLevels=QtTSA.waterfall_auto.isChecked())

    def resetGUI(self, frequencies, readings):
        self.waterfall.clear()
        self.createTimeSpectrum(frequencies, readings)

    def sweepComplete(self, frequencies):
        # update markers if not in zero span, where they are not relevant
        if frequencies[0] != frequencies[-1]:
            M1.updateMarker()
            M2.updateMarker()
            M3.updateMarker()
            M4.updateMarker()
            if fading.ui.isVisible():
                timeNow = time.time()
                if fading.ui.isVisible():
                    M1.updateMarkerTimePlot(frequencies, timeNow)
                    M2.updateMarkerTimePlot(frequencies, timeNow)
                    M3.updateMarkerTimePlot(frequencies, timeNow)
                    M4.updateMarkerTimePlot(frequencies, timeNow)
                if self.timeIndex < np.size(self.timeMarkVals, axis=0) - 1:
                    self.timeIndex += 1
                else:
                    # array of values is full so start rolling to the left and stop incrementing the index
                    self.timeMarkVals = np.roll(self.timeMarkVals, -1, axis=0)

        if phasenoise.ui.isVisible():  # fetches trace data from the graph and displays it in the phase noise window
            T1.phaseNoise(True)   # lsb
            T2.phaseNoise(False)  # usb

    def updateGUI(self, frequencies, readings, maxima, minima, runtime):  # called by signal from measurement() thread
        # for LNB/Mixer mode when LO is above measured freq the scan is reversed, i.e. low TinySA freq = high meas freq
        if bandstype.freq > frequencies[0]:
            frequencies = frequencies[::-1]  # reverse the array
            np.fliplr(readings)
            QtTSA.waterfall.invertX(True)
        else:
            QtTSA.waterfall.invertX(False)

        # update graph axes if in zero span
        if frequencies[0] == frequencies[-1]:
            QtTSA.graphWidget.setLabel('bottom', 'Time')
            frequencies = np.arange(1, len(frequencies) + 1, dtype=int)
            QtTSA.graphWidget.setXRange(frequencies[0], frequencies[-1])
        else:
            QtTSA.graphWidget.setLabel('bottom', units='Hz')

        # update the swept traces
        readingsAvg = np.nanmean(readings[0:QtTSA.avgBox.value()], axis=0)
        options = {'Normal': readings[0], 'Average': readingsAvg, 'Max': maxima, 'Min': minima, 'Freeze': readings[0]}
        T1.update(frequencies, options.get(T1.traceType))
        T2.update(frequencies, options.get(T2.traceType))
        T3.update(frequencies, options.get(T3.traceType))
        T4.update(frequencies, options.get(T4.traceType))

        self.updateWaterfall(frequencies, readings)

        # update 3D graph if enabled
        if QtTSA.stackedWidget.currentWidget() == QtTSA.View3D:
            z = readings + 120  # Surface plot height shader needs positive numbers so convert from dBm to dBf
            logging.debug(f'z = {z}')
            self.surface.setData(z=z)  # update 3D graph
            params = QtTSA.openGLWidget.cameraParams()
            logging.debug(f'camera {params}')
        if QtTSA.grid.isChecked():
            tinySA.vGrid.show()
        else:
            tinySA.vGrid.hide()

        # other updates
        if QtTSA.points_auto.isChecked():
            QtTSA.points_box.setValue(np.size(frequencies))

        QtTSA.updates.setText(str(int(1/(runtime/1e9))))  # the display update frequency indicator

        if not tinySA.sweeping:  # measurement thread is stopping
            QtTSA.scan_button.setText('Stopping ...')
            QtTSA.scan_button.setStyleSheet('background-color: orange')
            QtTSA.run3D.setText('Stopping ...')
            QtTSA.run3D.setStyleSheet('background-color: orange')

    def orbit3D(self, sign, azimuth=True):  # orbits the camera around the 3D plot
        degrees = QtTSA.rotateBy.value()
        if azimuth:
            QtTSA.openGLWidget.orbit(sign*degrees, 0)  # sign controls direction and is +1 or -1
        else:
            QtTSA.openGLWidget.orbit(0, sign*degrees)

    def axes3D(self, sign, axis):  # shifts the plot along one of its 3 axes - time, frequency, signal
        pixels = QtTSA.panBy.value()
        options = {'X': (pixels*sign, 0, 0), 'Y': (0, pixels*sign, 0), 'Z': (0, 0, pixels*sign)}
        s = options.get(axis)
        QtTSA.openGLWidget.pan(s[0], s[1], s[2], relative='global')

    def reset3D(self):  # sets the 3D view back to the starting point
        QtTSA.openGLWidget.reset()
        self.orbit3D(135, 'X')
        QtTSA.openGLWidget.pan(0, 0, -10, relative='global')
        self.zoom3D()

    def grid(self, sign):  # moves the grid backwards and forwards on the time axis
        step = QtTSA.rotateBy.value()
        if QtTSA.grid.isChecked():
            self.vGrid.translate(step*sign, 0, 0)

    def zoom3D(self):  # zooms the camera in and out
        zoom = QtTSA.zoom.value()
        QtTSA.openGLWidget.setCameraParams(distance=zoom)

    def runButton(self, action):
        # Update the Run/Stop buttons' text and colour
        QtTSA.scan_button.setText(action)
        QtTSA.run3D.setText(action)
        if action == 'Stopping':
            QtTSA.scan_button.setStyleSheet('background-color: yellow')
            QtTSA.run3D.setStyleSheet('background-color: yellow')
        else:
            QtTSA.scan_button.setStyleSheet('background-color: ')
            QtTSA.run3D.setStyleSheet('background-color: ')
            QtTSA.scan_button.setEnabled(True)
            QtTSA.run3D.setEnabled(True)

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
        # version = 'tinySA4_v1.4-199-gde12ba2'  # for testing ultra
        # version = 'tinySA_v1.4-175-g1419a93'   # for testing basic
        return version

    def spur(self):
        sType = QtTSA.spur_box.currentText()
        if sType == 'auto' and not self.tinySA4:  # tinySA3 (basic) has no auto spur mode
            sType = 'on'
        command = 'spur ' + sType + '\r'
        self.fifo.put(command)

    def lna(self):
        if QtTSA.lna_box.isChecked():
            command = 'lna on\r'
            QtTSA.atten_auto.setEnabled(False)  # attenuator and lna are switched so mutually exclusive
            QtTSA.atten_auto.setChecked(False)
            QtTSA.atten_box.setEnabled(False)
            QtTSA.atten_box.setValue(0)
            self.fifo.put('attenuate 0\r')
        else:
            command = 'lna off\r'
            QtTSA.atten_auto.setEnabled(True)
            QtTSA.atten_auto.setChecked(True)
            self.fifo.put('attenuate auto\r')
        self.fifo.put(command)

    def attenuator(self):
        atten = QtTSA.atten_box.value()
        if QtTSA.atten_auto.isChecked():
            atten = 'auto'
            QtTSA.atten_box.setEnabled(False)
        else:
            if not QtTSA.lna_box.isChecked():  # attenuator and lna are switched so mutually exclusive
                QtTSA.atten_box.setEnabled(True)
        command = f'attenuate {str(atten)}\r'
        self.fifo.put(command)

    def setTime(self):
        if self.tinySA4 and settings.syncTime.isChecked():
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
        command = f'repeat {QtTSA.sampleRepeat.value()}\r'
        self.fifo.put(command)

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
        if self.usb and not self.tinySA4:
            popUp(QtTSA, "TinySA basic does not have file storage", 'Ok', 'Info')
            return
        if self.threadRunning:
            popUp(QtTSA, "Cannot browse tinySA whilst a scan is running", 'Ok', 'Info')
            return
        elif self.usb:
            SD = self.listSD()
            filebrowse.listWidget.clear()
            ls = []
            for i in range(len(SD.splitlines())):
                ls.append(SD.splitlines()[i].split(" ")[0])
            filebrowse.listWidget.insertItems(0, ls)
            filebrowse.ui.show()
        else:
            popUp(QtTSA, 'TinySA not found', 'Ok', 'Critical')

    def saveFile(self, saveSingle=True):
        filebrowse.saveProgress.setValue(0)
        SD = self.listSD()
        for i in range(len(SD.splitlines())):
            if not self.directory:  # have not already saved a file, or ask for folder was checked
                self.directory = QFileDialog.getExistingDirectory(caption="Select folder to save SD card file")
            if not self.directory:
                break
            if saveSingle:
                fileName = filebrowse.listWidget.currentItem().text()  # the file selected in the list widget
            else:
                fileName = SD.splitlines()[i].split(" ")[0]
            with open(os.path.join(self.directory, fileName), "wb") as file:
                data = self.readSD(fileName)
                file.write(data)
            filebrowse.saveProgress.setValue(int(100 * (i+1)/len(SD.splitlines())))
            filebrowse.downloadInfo.setText(self.directory)  # show the path where the file was saved
            if filebrowse.askForPath.isChecked():
                self.directory = None
            if saveSingle:
                filebrowse.saveProgress.setValue(100)
                break

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
        xaxis = (QtTSA.graphWidget.getAxis('bottom').range)
        startF = float(xaxis[0]/1e6)
        stopF = float(xaxis[1]/1e6)
        logging.debug(f'mouseScaled: start = {startF} stop = {stopF}')
        QtTSA.start_freq.setValue(startF)
        QtTSA.stop_freq.setValue(stopF)
        self.freq_changed(False)

    # SDR: Handle SDR frequency updates from Marker class
    def update_sdr_from_marker(self, marker_name, frequency_hz):
        if marker_name == '1' and self.sdr_active:
            # Only update SDR frequency if it has changed significantly
            if abs(frequency_hz - self.last_sdr_freq_sent) > self.sdr_freq_threshold:
                sdr.set_sdr_frequency(frequency_hz)
                self.last_sdr_freq_sent = frequency_hz # Update last sent frequency
                sdr_audio_popup.ui.frequency_label.setText(f"Frequency: {frequency_hz / 1e6:.3f} MHz")
                print(f"[QtTinySA: INFO] SDR frequency updated from Marker {marker_name} to {frequency_hz / 1e6:.3f} MHz (threshold: {self.sdr_freq_threshold} Hz).\n")
            else:
                pass
                

class Trace:
    def __init__(self, name):
        self.name = name
        self.pen = None
        self.trace = QtTSA.graphWidget.plot([], [], name=name, width=1, padding=0)
        self.fifo = queue.SimpleQueue()
        self.noise = phasenoise.plotWidget.plot([], [], name=name, width=1, padding=0)
        self.box = pyqtgraph.TextItem(text='', color='k', border='y', fill='y', anchor=(-1, -0.4))  # anchor y=vertical
        self.box.setParentItem(phasenoise.plotWidget.plotItem)
        self.box.setVisible(False)

    def guiRef(self, opt):
        guiFields = ({'1': QtTSA.m1_type, '2': QtTSA.m2_type, '3': QtTSA.m3_type, '4': QtTSA.m4_type},
                     {'1': QtTSA.m1trace, '2': QtTSA.m2trace, '3': QtTSA.m3trace, '4': QtTSA.m4trace},
                     {'1': QtTSA.trace1, '2': QtTSA.trace2, '3': QtTSA.trace3, '4': QtTSA.trace4},
                     {'1': QtTSA.t1_type, '2': QtTSA.t2_type, '3': QtTSA.t3_type, '4': QtTSA.t4_type})
        Ref = guiFields[opt].get(self.name)
        return Ref

    def tType(self):
        self.traceType = self.guiRef(3).currentText()  # '3' selects trace type, 'guiRef' is replaced by the option

    def enable(self):  # show or hide a trace
        if self.guiRef(2).isChecked():  # 2 selects trace-enabled checkboxes
            tint = str("background-color: '" + self.pen + "';")
            self.guiRef(3).setStyleSheet(tint)  # 3 selects trace type combo-boxes
            self.trace.show()
        else:
            self.guiRef(3).setStyleSheet("background-color: ;")
            self.trace.hide()
        checkboxes.dwm.submit()

    def setup(self):
        self.tType()
        self.pen = tracecolours.tm.record(int(self.name)-1).value('colour')
        self.trace.setPen(self.pen)
        self.noise.setPen(self.pen)
        self.enable()

    def update(self, frequencies, levels):
        if self.traceType != 'Freeze':
            self.trace.setData(frequencies, levels)

    def fetchData(self):
        '''return the plotted data from the first trace listDataItems[0]'''
        frequencies = QtTSA.graphWidget.getPlotItem().listDataItems()[int(self.name) - 1].getData()[0]  # [0] = freq
        levels = QtTSA.graphWidget.getPlotItem().listDataItems()[int(self.name) - 1].getData()[1]  # [1] = level
        return frequencies, levels

    def phaseNoise(self, lsb):
        '''M1 is used in max tracking mode to find the frequency and level reference of the signal to be measured
           T1 data is used for the LSB measurement and T2 data is used for USB'''
        frequencies, levels = self.fetchData()
        if frequencies is None or levels is None:
            return
        rbw = float(QtTSA.rbw_box.currentText())  # kHz
        tone = np.argmin(np.abs(frequencies - (M1.line.value())))  # find freq array index of peak of signal
        mask = np.count_nonzero(frequencies[tone:] < frequencies[tone] + (rbw * 1e3 * settings.rbw_x.value()))

        # From https://github.com/Hagtronics/tinySA-Ultra-Phase-Noise
        shapeFactor = {0.2: 3.4, 1: -0.6, 3: -5.3, 10: 0, 30: 0, 100: 0, 300: 0, 600: 0, 850: 0}
        eqnbw = shapeFactor.get(rbw)

        # Calculate Noise Power 1Hz bandwidth normalising factor for the RBW
        factor = 10 * np.log10(rbw * 1e3)

        if lsb:
            delta = levels[:tone-mask+1] - levels[tone]  # relative level of sideband points to the lsb_tone
            dBcHz = delta + eqnbw - factor
            freqOffset = (frequencies[tone] - frequencies[:tone-mask+1])
        else:
            delta = levels[tone+mask:] - levels[tone]  # relative level of sideband points to the usb_tone
            dBcHz = delta + eqnbw - factor
            freqOffset = (frequencies[tone+mask:] - frequencies[tone])

        self.noise.setData(freqOffset, dBcHz)

        # show signal frequency from Marker 1 in label box
        T1.box.setVisible(True)
        decimal = M1.setPrecision(frequencies, frequencies[0])
        unit, multiple = M1.setUnit(frequencies[0])  # set units
        self.box.setText(f'{M1.line.value()/multiple:.{decimal}f}{unit}')


class Limit:
    def __init__(self, pen, x, y, movable):  # x = None, horizontal.  y = None, vertical
        self.pen = pen
        self.x = x
        self.y = y
        self.movable = movable

    def create(self, dash=False, mark='', posn=0.99):
        label = ''
        if self.y:
            label = '{value:.1f}'
        self.line = QtTSA.graphWidget.addLine(self.x, self.y, movable=self.movable, pen=self.pen, label=label,
                                              labelOpts={'position': 0.98, 'color': (self.pen), 'movable': True})
        self.line.addMarker(mark, posn, 10)
        if dash:
            self.line.setPen(self.pen, width=0.5, style=QtCore.Qt.PenStyle.DashLine)

    def visible(self, show=True):
        if show:
            self.line.show()
        else:
            self.line.hide()


class Marker:
    def __init__(self, name, box):
        self.name = name
        self.linked = None  # which trace data the marker uses as default
        self.level = 1  # marker tracking level (min or max), set per marker from GUI
        self.markerType = 'Normal'
        self.line = QtTSA.graphWidget.addLine(88, 90, movable=True, name=name,
                                              pen=pyqtgraph.mkPen('y', width=0.5), label=self.name)
        self.deltaline = QtTSA.graphWidget.addLine(0, 90, movable=True, name=name,
                                                   pen=pyqtgraph.mkPen('y', width=0.5, style=QtCore.Qt.PenStyle.DashLine), label=self.name)
        self.line.addMarker('^', 0, 10)
        self.deltaF = 0  # the delta marker frequency difference
        self.deltaRelative = True
        self.deltaline.sigClicked.connect(self.deltaClicked)
        self.line.sigClicked.connect(self.lineClicked)
        self.markerBox = pyqtgraph.TextItem(text='', border=None, anchor=(-0.7, -box), fill='k')  # box is vertical posn
        self.markerBox.setParentItem(QtTSA.graphWidget.plotItem)
        self.fifo = queue.SimpleQueue()
        self.dBm = -140
        self.createMarkerTimePlot()
        self.polar = pattern.plotwidget.plot([], [], name=name, width=1, padding=0)
        self.runTimer = QtCore.QElapsedTimer()  # for polar plot
        self.sweeptime = []  # for polar plot
        self.amplitude = []  # for polar plot
        self.samples = []  # for polar plot

    def guiRef(self, opt):
        guiFields = ({'1': QtTSA.m1_type, '2': QtTSA.m2_type, '3': QtTSA.m3_type, '4': QtTSA.m4_type},
                     {'1': QtTSA.m1trace, '2': QtTSA.m2trace, '3': QtTSA.m3trace, '4': QtTSA.m4trace},
                     {'1': 'm1f', '2': 'm2f', '3': 'm3f', '4': 'm4f'},
                     {'1': QtTSA.m1track, '2': QtTSA.m2track, '3': QtTSA.m3track, '4': QtTSA.m4track})
        Ref = guiFields[opt].get(self.name)
        return Ref

    def traceLink(self, setting):
        traces = {'1': T1, '2': T2, '3': T3, '4': T4}
        self.linked = traces.get(str(setting))
        tint = str("background-color: '" + self.linked.pen + "';")
        # self.guiRef(0).setStyleSheet(tint)
        self.guiRef(1).setStyleSheet(tint)
        checkboxes.dwm.submit()

    def setup(self, colour):
        '''restore the marker frequencies from the configuration database and set starting conditions'''
        self.line.setValue(numbers.tm.record(0).value(self.guiRef(2)))
        self.line.label.setColor(colour)
        self.line.label.setPosition(0.02)
        self.line.label.setMovable(True)
        self.line.setPen(color=colour, width=0.5)
        self.mType()
        self.traceLink(self.guiRef(1).value())
        self.setLevel(self.guiRef(3).value())
        self.deltaline.hide()
        self.deltaline.setValue(0)
        self.deltaF = 0
        self.deltaline.label.setPosition(0.05)
        self.deltaline.label.setMovable(True)
        M2.tplot.setXLink(M1.tplot)
        M3.tplot.setXLink(M1.tplot)
        M4.tplot.setXLink(M1.tplot)
 
    def start(self):  # set marker to the sweep start frequency
        if self.markerType != 'Off':
            self.line.setValue(QtTSA.start_freq.value() * 1e6)
            self.mType()

    def spread(self):  # spread markers equally across scan range
        if self.markerType != 'Off':
            if self.line.value() <= QtTSA.start_freq.value() * 1e6 or self.line.value() > QtTSA.stop_freq.value() * 1e6:
                self.line.setValue(QtTSA.start_freq.value() * 1e6 + (0.2 * int(self.name) * QtTSA.span_freq.value() * 1e6))
                self.mType()

    def lineClicked(self):  # toggle visibility of associated delta marker
        if self.deltaline.value() != 0:
            self.deltaline.hide()
            self.deltaline.setValue(0)
        else:
            self.deltaline.show()
            self.deltaline.setValue(self.line.value() + QtTSA.span_freq.value() * 2.5e4)
            # self.deltaF = 0
            self.deltaF = self.deltaline.value() - self.line.value()

    def deltaClicked(self):  # toggle relative or absolute labelling
        if self.deltaRelative:
            self.deltaRelative = False
        else:
            self.deltaRelative = True

    def deltaMoved(self):  # set the delta freq offset
        self.deltaF = self.deltaline.value() - self.line.value()
        self.updateMarker()

    def mType(self):
        self.markerType = self.guiRef(0).currentText()  # current combobox value from appropriate GUI field
        if self.markerType == 'Off' or self.markerType == '':
            self.line.hide()
            self.deltaline.hide()
            self.deltaline.setValue(0)
            self.deltaF = 0
        else:
            self.line.show()
            
    def setDelta(self):  # delta marker locking to reference marker
        self.deltaline.setValue(self.line.value() + self.deltaF)
        self.updateMarker()

    def updateMarker(self):  # called by sweepComplete() and fifo timer
        if self.markerType == 'Off':
            self.markerBox.setVisible(False)
            return
        else:
            self.markerBox.setVisible(True)
            
        frequencies, levels = self.linked.fetchData()  # fetch data from the graph
        if frequencies is None or levels is None:
            return

        # flatten the arrays
        frequencies.reshape(-1)
        levels.reshape(-1)

        if self.markerType in ('Max', 'Min'):
            self.calcMaskFreq(frequencies)
            maxmin = self.maxMin(frequencies, levels)
            logging.debug(f'updateMarker(): maxmin = {maxmin}')
            if self.markerType == 'Max':
                self.line.setValue(maxmin[0][self.level])
                if self.deltaline.value != 0:
                    self.deltaline.setValue(maxmin[0][self.level] + self.deltaF)
            if self.markerType == 'Min':
                self.line.setValue(maxmin[1][self.level])
                if self.deltaline.value != 0:
                    self.deltaline.setValue(maxmin[1][self.level] + self.deltaF)
        
        lineIndex = np.argmin(np.abs(frequencies - (self.line.value())))  # find closest value in freq array
        logging.debug(f'updateMarker(): index={lineIndex} frequency={frequencies[lineIndex]}\n')
        self.line.setValue(frequencies[lineIndex])
               
        self.dBm = levels[lineIndex]

        decimal = self.setPrecision(frequencies, frequencies[0])  # set decimal places
        unit, multiple = self.setUnit(frequencies[0])  # set units
        self.markerBox.setText(f'M{self.line.name()} {self.line.value()/multiple:.{decimal}f}{unit} {self.dBm:.1f}dBm')

        # SDR: Call tinySA to update SDR frequency if this is Marker 1
        if self.line.name() == '1': # Check if this is Marker 1
            # Access the global 'tinySA' instance to update SDR
            tinySA.update_sdr_from_marker(self.line.name(), self.line.value())

        if self.deltaF != 0:
            deltaLineIndex = np.argmin(np.abs(frequencies - (self.deltaline.value())))  # closest value in freq array
            if self.deltaRelative:
                deltaLinedBm = levels[deltaLineIndex]
                dBm = deltaLinedBm - self.dBm
                decimal = self.setPrecision(frequencies, self.deltaF)  # deltaF can be negative
                unit, multiple = self.setUnit(self.deltaF)
                self.deltaline.label.setText(
                    f'{chr(916)}{self.line.name()} {dBm:.1f}dB\n{(self.deltaF / multiple):.{decimal}f}{unit}')
            else:
                dBm = levels[deltaLineIndex]
                self.deltaline.label.setText(
                    f'{chr(916)}{self.line.name()} {dBm:.1f}dBm\n{(self.deltaline.value()/multiple):.{decimal}f}{unit}')

    def addFreqMarker(self, freq, colour, name, band=True):  # adds simple freq marker without full marker capability
        if QtTSA.presetLabel.isChecked():
            if band:
                self.marker = QtTSA.graphWidget.addLine(freq, 90, pen=pyqtgraph.mkPen(colour, width=0.5,
                                                        style=QtCore.Qt.PenStyle.DashLine), label=name,
                                                        labelOpts={'position': 0.97, 'color': (colour)})
            else:
                self.marker = QtTSA.graphWidget.addLine(freq, 90, pen=pyqtgraph.mkPen(colour, width=0.5,
                                                        style=QtCore.Qt.PenStyle.DashLine), label=name,
                                                        labelOpts={'position': 0.04, 'color': (colour),
                                                        'anchors': ((0, 0.2), (0, 0.2))})
            self.marker.label.setMovable(True)
        else:
            self.marker = QtTSA.graphWidget.addLine(freq, 90, pen=pyqtgraph.mkPen(colour, width=0.5,
                                                    style=QtCore.Qt.PenStyle.DashLine))
        self.fifo.put(self.marker)  # store the marker object in a queue
        logging.debug(f'addFreqMarker(): fifo size = {self.fifo.qsize()}')

    def delFreqMarkers(self):
        for i in range(0, self.fifo.qsize()):
            QtTSA.graphWidget.removeItem(self.fifo.get())  # remove the marker and its corresponding object in the queue

    def maxMin(self, frequencies, levels):  # finds the signal max/min (indexes) for setting markers
        logging.debug(f'maxmin: linked tracetype = {self.linked.traceType}')

        # mask outside high/low freq boundary lines
        levels = np.ma.masked_where(frequencies > highF.line.value(), levels)
        levels = np.ma.masked_where(frequencies < lowF.line.value(), levels)

        # mask readings below threshold line
        levels = np.ma.masked_where(levels <= threshold.line.value(), levels)

        maxi = [np.argmax(levels)]  # the index of the highest peak in the masked readings array
        mini = [np.argmin(levels)]  # the index of the deepest minimum in the masked readings array
        nextMax = nextMin = levels
        for i in range(8):
            # mask frequencies around detected peaks and find the next 8 highest/lowest peaks
            nextMax = np.ma.masked_where(np.abs(frequencies[maxi[-1]] - frequencies) < self.maskFreq, nextMax)
            maxi.append(np.argmax(nextMax))
            nextMin = np.ma.masked_where(np.abs(frequencies[mini[-1]] - frequencies) < self.maskFreq, nextMin)
            mini.append(np.argmin(nextMin))
        return (list(frequencies[maxi]), list(frequencies[mini]))

    def setLevel(self, setting):
        self.level = setting - 1  # array indexes start at 0 not 1

    def calcMaskFreq(self, frequencies):
        '''calculate a frequency width factor to use to mask readings near each max/min frequency'''
        if QtTSA.rbw_auto.isChecked():
            # auto rbw is ~7 kHz per 1 MHz scan frequency span
            approx_rbw = 7 * (frequencies[-1] - frequencies[0]) / 1e6  # kHz
            # find the nearest lower discrete rbw value
            for i in range(0, rbwtext.tm.rowCount() - 1):
                rbw = float(rbwtext.tm.record(i).value('value'))  # kHz
                if approx_rbw <= float(rbwtext.tm.record(i).value('value')):
                    break
            self.maskFreq = settings.rbw_x.value() * rbw * 1e3  # Hz
        else:
            # manual rbw setting
            self.maskFreq = settings.rbw_x.value() * float(QtTSA.rbw_box.currentText()) * 1e3  # Hz
            logging.debug(f'manual rbw masking factor = {self.maskFreq/1e3}kHz')

    def setPrecision(self, frequencies, spotF):  # sets the marker indicated frequency precision
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

    def setUnit(self, spotF):
        index = int(np.log10(abs(spotF)))
        suffix = ['Hz', 'Hz', 'Hz', 'kHz', 'kHz', 'kHz', 'MHz', 'MHz', 'MHz', 'GHz', 'GHz']
        multiple = [1, 1, 1, 1e3, 1e3, 1e3, 1e6, 1e6, 1e6, 1e9, 1e9]
        return suffix[index], multiple[index]

    def createMarkerTimePlot(self):
        '''multiplot is the name of the graphics layout grid widget in the fading dialogue window'''
        self.tplot = multiplot.addPlot(title='Marker ' + self.name)
        self.tplot.setAxisItems({'bottom': pyqtgraph.DateAxisItem()})
        self.tplot.showGrid(x=True, y=True)
        multiplot.nextRow()
        self.tplot.plot([], [])

    def updateMarkerTimePlot(self, frequencies, timeNow):  # called by sweepComplete()
        self.tplot.clear()  # if it's not cleared the GUI runs slower and slower
        if self.markerType == 'Off':
            self.tplot.hide()
            self.runTimer.invalidate()
            return
        else:
            self.tplot.show()
        decimal = self.setPrecision(frequencies, frequencies[0])  # set decimal places
        unit, multiple = self.setUnit(self.line.value())  # set units
        self.tplot.setTitle(f'Marker {self.name} = {(self.line.value()/multiple):.{decimal}f}' + unit)

        tinySA.timeMarkVals[tinySA.timeIndex, 0] = timeNow
        tinySA.timeMarkVals[tinySA.timeIndex, int(self.name)] = self.dBm
        self.tplot.plot(tinySA.timeMarkVals[:, 0], tinySA.timeMarkVals[:, int(self.name)], pen=self.linked.pen)

        if self.runTimer.isValid():  # polar pattern plot is active
            self.updatePolarPlot()

    def setPolarPlot(self):
        if self.markerType != 'Off':
            pattern.progress.setValue(0)
            if pattern.manual.isChecked():
                self.samples = []
            else:
                self.sweeptime = []
                self.amplitude = []
            self.runTimer.start()

    def updatePolarPlot(self):
        if pattern.manual.isChecked():
            if len(self.samples) < pattern.scanCount.value():
                self.samples.append(self.dBm)
                pattern.progress.setValue(int(100 * len(self.samples) / pattern.scanCount.value()))
                return
            else:
                if pattern.max.isChecked():
                    self.amplitude.append(np.max(self.samples))
                if pattern.avg.isChecked():
                    self.amplitude.append(np.average(self.samples))
                if pattern.min.isChecked():
                    self.amplitude.append(np.min(self.samples))
                self.sweeptime.append(pattern.heading.value())  # append the current heading (instead of rotation time)
                self.runTimer.invalidate()
                theta = np.divide((np.multiply(self.sweeptime, np.pi)), 180)  # convert heading in degrees to radians
        else:
            if self.sweeptime == []:  # auto plot has just been started
                self.sweeptime.append(0)
            else:
                self.sweeptime.append(self.runTimer.elapsed() / 1000)
            self.amplitude.append(self.dBm)
            if pattern.clockwise.isChecked():
                theta = np.divide((np.multiply(self.sweeptime, 2 * np.pi)), pattern.rotateTime.value())
                pattern.heading.setValue(int(360*(theta[-1] / (2 * np.pi))))
            else:
                theta = np.divide((np.multiply(self.sweeptime, -2 * np.pi)), pattern.rotateTime.value())
                pattern.heading.setValue(360 + int(360*(theta[-1] / (2 * np.pi))))
            pattern.progress.setValue(int(100 * (abs(theta[-1]) / (2 * np.pi))))

        peak = max(np.max(self.amplitude), pattern.refdBm.value())  # peak is maximum when antenna points at the source
        factor = 40 - peak  # correction factor to make the max signal amplitude read 40 units on the polar grid
        dBm = np.round(np.add(self.amplitude, factor), decimals=1)

        r = np.clip(dBm, 0, 40)  # clip the signal vector to a max amplitude of 40 and minimum of 0
        x = np.multiply(r, np.sin(theta))
        y = np.multiply(r, np.cos(theta))
        self.polar.setData(x, y, pen=self.linked.pen)

        if self.sweeptime[-1] >= pattern.rotateTime.value():  # rotation is complete
            self.runTimer.invalidate()
            if pattern.beamUp.isChecked() and not pattern.manual.isChecked():
                self.rotatePolarPlot(r, theta)

    def rotatePolarPlot(self, r, theta):
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
        self.polar.setData(x, y, pen=self.linked.pen)


class WorkerSignals(QtCore.QObject):
    error = QtCore.pyqtSignal(str)
    result = QtCore.pyqtSignal(np.ndarray, np.ndarray, np.ndarray, np.ndarray, float)
    fullSweep = QtCore.pyqtSignal(np.ndarray, np.ndarray)
    saveResults = QtCore.pyqtSignal(np.ndarray, np.ndarray)
    resetGUI = QtCore.pyqtSignal(np.ndarray, np.ndarray)
    finished = QtCore.pyqtSignal()
    sweepEnds = QtCore.pyqtSignal(np.ndarray)


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


class CustomTableModel(QSqlRelationalTableModel):
    def __init__(self, parent=None, db=None, ro_columns=tuple()):
        super().__init__(parent, db)
        self.read_only = ro_columns

    def flags(self, index):
        if index.column() in self.read_only:
            return QtCore.Qt.ItemFlag.ItemIsEnabled | QtCore.Qt.ItemFlag.ItemIsSelectable
        else:
            return super().flags(index)


class ModelView():
    '''set up and process data models bound to the GUI widgets'''

    def __init__(self, table_name, db_name, ro_columns):
        self.currentRow = 0
        self.ID = 0
        self.freq = 0
        self.createTableModel(table_name, db_name, ro_columns)

    def createTableModel(self, table_name, db_name, limit_edit):
        self.tm = CustomTableModel(db=db_name, ro_columns=limit_edit)
        self.tm.setTable(table_name)

    def createMapper(self):
        self.dwm = QDataWidgetMapper()
        self.dwm.setModel(self.tm)
        self.dwm.setSubmitPolicy(QDataWidgetMapper.SubmitPolicy.AutoSubmit)

    def addRow(self):  # adds a blank row to the table widget above current row
        logging.debug(f'addRow(): currentRow = {self.currentRow}')
        if self.currentRow == 0:
            self.tm.insertRow(0)
        else:
            self.tm.insertRow(self.currentRow)
        self.tm.layoutChanged.emit()  # don't invoke select() because the row has not yet been populated or saved

    def saveChanges(self):
        self.dwm.submit()

    def deleteRow(self, single=True):  # deletes rows in the table widget
        if single:
            logging.debug(f'deleteRow: current row = {self.currentRow}')
            self.tm.removeRow(self.currentRow)
        else:
            for i in range(0, self.tm.rowCount()):
                self.tm.removeRow(i)
        self.tm.select()
        self.tm.layoutChanged.emit()

    def deletePsType(self):
        record = self.tm.record(self.currentRow)
        if record.value('ID') == bandstype.ID:
            popUp(presetFreqs, "Cannot delete a preset type that is selected on main screen", 'Ok', 'Critical')
            return
        bands.filterType(True, record.value('preset'))
        bands.deleteRow(False)
        if bands.tm.rowCount() == 0:
            # now no freq records with the preset type, so can delete it and keep database referential integrity
            self.deleteRow(True)

    def tableClicked(self, table):
        self.currentRow = table.currentIndex().row()  # the row index from the QModelIndexObject
        logging.debug(f'row {self.currentRow} clicked')
        if table == presetFreqs.typeTable:
            record = self.tm.record(self.currentRow)
            bands.filterType(True, record.value('preset'))
            bands.unlimited()
            presetFreqs.psCount.setValue(bands.tm.rowCount())

    def insertData(self, **data):
        record = self.tm.record()
        logging.debug(f'insertData: record = {record}')
        for key, value in data.items():
            logging.debug(f'insertData: key = {key} value={value}')
            record.setValue(str(key), value)
        self.tm.insertRecord(-1, record)
        self.tm.select()
        self.tm.layoutChanged.emit()
        self.dwm.submit()

    def filterType(self, prefsDialog, boxText):
        sql = 'preset = "' + boxText + '"'
        if prefsDialog:
            self.tm.setFilter(sql)
        else:
            sql = 'visible = "1" AND preset = "' + boxText + '"'
            if tinySA.tinySA4 is False:  # It's a tinySA basic with limited frequency range
                sql = sql + ' AND startF <= "960000000"'
            self.tm.setFilter(sql)
            QtTSA.band_box.activated.emit(0)

            # find and store the ID of the preset type selected in the combobox
            bandstype.unlimited()
            for index in range(0, bandstype.tm.rowCount()):
                record = bandstype.tm.record(index)
                if record.value('preset') == boxText:
                    bandstype.ID = record.value('ID')
                    bandstype.freq = record.value('LO')
                    isMixerMode()
                    break

    def readCSV(self, fileName):
        with open(fileName, "r") as fileInput:
            reader = csv.DictReader(fileInput)
            for row in reader:
                logging.debug(f'readCSV(): row = {row}')
                record = self.tm.record()
                for key, value in row.items():
                    if key == 'preset':
                        value = bandstype.fetch_ID('preset', value)
                    if key == 'colour':
                        value = colours.fetch_ID('colour', value)
                    if key == 'value':
                        value = int(eval(value))

                    if key == 'Frequency':  # to match RF mic CSV files
                        key = 'startF'
                        value = str(float(value) / 1e3)

                    if key != 'ID':  # ID is the table primary key and is auto-populated
                        record.setValue(str(key), value)

                if record.value('value') not in (0, 1):  # because it's not present in RF mic CSV files
                    record.setValue('value', 1)
                if record.value('preset') == '':  # preset missing so use current preferences filterbox text
                    record.setValue('preset', bandstype.fetch_ID('preset', presetFreqs.filterBox.currentText()))
                self.tm.insertRecord(-1, record)
        self.tm.select()
        self.tm.layoutChanged.emit()
        # self.dwm.submit()

    def fetch_ID(self, field, lookup_value):  # find the relation table ID from an aliased field name
        for i in range(0, self.tm.rowCount()):
            record = self.tm.record(i).value(field)
            if record == lookup_value:
                ID = self.tm.record(i).value('ID')
                return ID
        return 1

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

    def exportData(self, filename=''):
        if filename == '':
            filename = QFileDialog.getSaveFileName(caption="Save As", filter="Comma Separated Values (*.csv)")[0]
        logging.info(f'exporting data to {filename}')
        if filename != '':
            self.writeCSV(filename)

    def importData(self, filename=''):
        if filename == '':
            filename = QFileDialog.getOpenFileName(caption="Open File", filter="Comma Separated Values (*.csv)")[0]
        logging.info(f'importing data from {filename}')
        if filename != '':
            self.readCSV(filename)

    def mapWidget(self, modelName):  # maps the widget combo-box fields to the database tables, using the mapping table
        maps.tm.setFilter('model = "' + modelName + '"')
        for index in range(0, maps.tm.rowCount()):
            gui = maps.tm.record(index).value('gui')
            column = maps.tm.record(index).value('column')
            self.dwm.addMapping(eval(gui), int(column))

    def unlimited(self):  # remove 256 row limit for QSql Query
        while self.tm.canFetchMore():
            self.tm.fetchMore()

    def showAll(self):
        presetFreqs.typeTable.clearSelection()
        self.tm.setFilter('')
        self.unlimited()
        presetFreqs.psCount.setValue(bands.tm.rowCount())

    def update_row(self, row, **data):
        record = self.tm.record(row)
        for key, value in data.items():
            logging.debug(f'update_row: key = {key} value={value}')
            record.setValue(str(key), value)
        self.tm.setRecord(row, record)
        self.updateModel()

    def read_tables(self):  # read the correction tables from the tinySA and display in a table widget
        if tinySA.usb is None:
            popUp(offset, 'TinySA not found', 'Ok', 'Critical')
            return
        if tinySA.threadRunning:
            popUp(offset, "Cannot read from tinySA whilst a scan is running", 'Ok', 'Info')
            return
        self.unlimited()
        write_config = QMessageBox.StandardButton.Ok
        if self.tm.rowCount() > 0 and offset.save_box.isChecked():
            message = ('OK to over-write config database table with\rcorrection data from tinySA?')
            write_config = popUp(offset, message, 'OkC', 'Question')
            if write_config == QMessageBox.StandardButton.Ok:
                correction.deleteRow(single=False)
        offset.tsa_table.clear()
        offset.tsa_table.setRowCount(200)
        offset.tsa_table.setColumnCount(4)
        offset.tsa_table.setHorizontalHeaderLabels(['mode', 'entry', 'frequency', 'dB'])
        offset.tsa_table.show()
        offset.tsa_table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeMode.ResizeToContents)

        k = 0
        for i in range(correctiontext.tm.rowCount()):
            # step through each correction mode, fetch its table from the tinySA
            command = 'correction ' + correctiontext.tm.record(i).value('value') + '\r'
            data = tinySA.serialQuery(command)
            mode_table = data.splitlines()[1:]  # make a list of the rows, discard the mode header
            mode_rows = [row.split(' ')[1:] for row in mode_table]  # split each row into a list, discard first col
            for j in range(20):
                # step through each row of the current mode and write the fields to the tablewidget 'tsa_table'
                mode = str(mode_rows[j][0])
                entry = str(mode_rows[j][1])
                frequency = str(mode_rows[j][2])
                dB = str(mode_rows[j][3])
                offset.tsa_table.setItem(k+j, 0, QTableWidgetItem(mode))
                offset.tsa_table.setItem(k+j, 1, QTableWidgetItem(entry))
                offset.tsa_table.setItem(k+j, 2, QTableWidgetItem(frequency))
                offset.tsa_table.setItem(k+j, 3, QTableWidgetItem(dB))

                if offset.save_box.isChecked() and write_config == QMessageBox.StandardButton.Ok:
                    self.insertData(mode=mode, entry=entry, frequency=frequency, dB=dB)
            k += 20

    def upload_correction(self):  # upload the correction table(s) from the config database to the tinySA
        if tinySA.threadRunning:
            popUp(offset, "Cannot update tinySA whilst a scan is running", 'Ok', 'Info')
            return
        self.unlimited()
        offset.progress.setValue(0)
        update_failed = False
        for i in range(self.tm.rowCount()):
            record = self.tm.record(i)
            mode = str(record.value('mode')) + ' '
            entry = str(record.value('entry')) + ' '
            frequency = str(record.value('frequency')) + ' '
            dB = str(record.value('dB'))
            command = 'correction ' + mode + entry + frequency + dB + '\r'
            response = tinySA.serialQuery(command)
            logging.debug(f'upload_correction(): {response}')
            if response != 'updated ' + entry + 'to ' + frequency + dB:
                # the error trapping on the tinySA is not comprehensive so this may not work for all scenarios
                update_failed = True
                logging.info(f'Update failure: {command}')
            else:
                offset.progress.setValue(100 * int(i / (self.tm.rowCount() - 1)))
        if update_failed:
            popUp(offset, "One or more of the updates failed.", 'Ok', 'Critical')


###############################################################################
# respond to GUI signals


def band_changed():
    index = QtTSA.band_box.currentIndex()
    startF = bandselect.tm.record(index).value('StartF')
    stopF = bandselect.tm.record(index).value('StopF')
    if stopF not in (0, '', startF):
        QtTSA.start_freq.setValue(startF / 1e6)
        QtTSA.stop_freq.setValue(stopF / 1e6)
        tinySA.freq_changed(False)  # start/stop mode
    else:
        centreF = startF / 1e6
        QtTSA.centre_freq.setValue(centreF)
        QtTSA.span_freq.setValue(int(centreF / 10))  # default span to a tenth of the centre freq
        tinySA.freq_changed(True)  # centre mode
    numbers.dwm.submit()
    freqMarkers()


def addBand():
    if QtTSA.m1_type.currentText() == 'Off':
        message = 'Please enable Marker 1'
        popUp(QtTSA, message, 'Ok', 'Info')
        return
    if QtTSA.m1_type.currentText() != 'Off' and QtTSA.m2_type.currentText() != 'Off':  # Two markers to set a band limit
        if M1.line.value() >= M2.line.value():
            message = 'M1 frequency >= M2 frequency'
            popUp(QtTSA, message, 'Ok', 'Info')
            return
        ID = bandstype.fetch_ID('preset', str(QtTSA.filterBox.currentText()))
        title = "New Frequency Band"
        message = "Enter a name for the new band."
        bandName, ok = QInputDialog.getText(None, title, message, QLineEdit.Normal, "")
        bands.insertData(name=bandName, type=ID, startF=f'{int(M1.line.value())}',
                         stopF=f'{int(M2.line.value())}', visible=1, colour=colours.fetch_ID('colour', 'green'))


def addFixed():
    title = "New fixed frequency Marker"
    message = "Enter a name for the fixed Marker"
    fixedMkr, ok = QInputDialog.getText(None, title, message, QLineEdit.Normal, "")
    bands.insertData(name=fixedMkr, type=12, startF=f'{int(M1.line.value())}',
                     stopF=0, visible=1, colour=colours.fetch_ID('colour', 'orange'))  # preset type 12 = fixed Marker


def pointsChanged():
    if QtTSA.points_auto.isChecked():
        QtTSA.points_box.setEnabled(False)
        QtTSA.rbw_box.setEnabled(True)
    else:
        QtTSA.points_box.setEnabled(True)
    tinySA.resume()  # puts a message in the fifo buffer so the measurement thread spots it and updates its settings


def memChanged():
    depth = QtTSA.memBox.value()
    if depth < QtTSA.avgBox.value():
        QtTSA.avgBox.setValue(depth)
    tinySA.scanMemory = depth
    tinySA.resume()  # puts a message in the fifo buffer so the measurement thread spots it and updates its settings


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


def centreToMarker():
    centreF = M1.line.value() * 1e-6
    QtTSA.centre_freq.setValue(centreF)


def markerLevel():
    M1.setLevel(QtTSA.m1track.value())
    M2.setLevel(QtTSA.m2track.value())
    M3.setLevel(QtTSA.m3track.value())
    M4.setLevel(QtTSA.m4track.value())


def setPreferences():  # called when the preferences window is closed
    checkboxes.dwm.submit()
    bands.tm.submitAll()
    threshold.line.setValue(settings.peakThreshold.value())
    best.visible(settings.neg25Line.isChecked())
    maximum.visible(settings.zeroLine.isChecked())
    damage.visible(settings.plus6Line.isChecked())

    if QtTSA.presetMarker.isChecked():
        freqMarkers()


def dialogPrefs():  # called by clicking on the setup > preferences menu
    presetFreqs.ui.show()
    presetFreqs.psCount.setValue(bands.tm.rowCount())


def about():
    message = ('TinySA Ultra GUI programme using Qt5 and PyQt\nAuthor: Ian Jefferson G4IXT\n\nVersion: {} \nConfig: {}'
               .format(app.applicationVersion(), config.databaseName()))
    popUp(QtTSA, message, 'Ok', 'Info')


def clickEvent():
    logging.info('clickEvent')


def testComPort():
    index = settings.deviceBox.currentIndex()
    tinySA.testPort(tinySA.ports[index - 1])  # allow for 'select device' entry


def correction_filter():
    if offset.filter_box.isChecked():
        sql = 'mode = "' + offset.correction_mode.currentText() + '"'
        correction.tm.setFilter(sql)
    else:
        correction.tm.setFilter('')


##############################################################################
# other methods

def saveFile(frequencies, readings):
    if settings.saveSweep.isChecked():
        timeStamp = time.strftime('%Y-%m-%d-%H%M%S')
        saver = Worker(writeSweep, timeStamp, frequencies, readings)  # workers deleted when thread ends
        threadpool.start(saver)


def writeSweep(timeStamp, frequencies, readings):
    array = np.insert(readings, 0, frequencies, axis=0)  # insert the measurement freqs at the top of the readings array
    dBm = np.transpose(np.round(array, decimals=2))  # transpose columns and rows
    fileName = str(timeStamp + '_RBW' + QtTSA.rbw_box.currentText() + '.csv')
    with open(fileName, "w", newline='') as fileOutput:
        output = csv.writer(fileOutput)
        for rowNumber in range(0, np.shape(dBm)[0]):
            fields = [dBm[rowNumber, columnNumber] for columnNumber in range(0, np.shape(dBm)[1])]
            output.writerow(fields)


def getPath(dbName):
    # 1. check if a personal database file exists already
    personalDir = platformdirs.user_config_dir(appname=app.applicationName(), appauthor=False)
    if not os.path.exists(personalDir):
        os.mkdir(personalDir)

    if os.path.isfile(os.path.join(personalDir, dbName)):
        logging.info(f'Database {dbName} found at {personalDir}')
        return personalDir

    # 2. if not, then check if a global database file exists
    globalDir = platformdirs.site_config_dir(appname=app.applicationName(), appauthor=False)
    if os.path.isfile(os.path.join(globalDir, dbName)):
        shutil.copy(os.path.join(globalDir, dbName), personalDir)
        logging.info(f'Database {dbName} copied from {globalDir} to {personalDir}')
        return personalDir

    # 3. if not, check if database file exists in the app directory
        file_path = app_dir(dbName)
        if os.path.isfile(file_path):
            shutil.copy(file_path, personalDir)
            logging.info(f'{dbName} copied from {app_dir} to {personalDir}')
            return personalDir

    # 4. If not, then look in current working folder & where the python file is stored/linked from
    workingDirs = [os.path.dirname(__file__), os.path.dirname(os.path.realpath(__file__)), os.getcwd()]
    for directory in workingDirs:
        if os.path.isfile(os.path.join(directory, dbName)):
            shutil.copy(os.path.join(directory, dbName), personalDir)
            logging.info(f'{dbName} copied from {directory} to {personalDir}')
            return personalDir

    raise FileNotFoundError("Unable to find the database {self.dbName}")


def app_dir(filename):
    # 'meipass' is used by pyinstaller to flag executable/file bundles
    if getattr(sys, 'frozen', True):
        base_path = sys._MEIPASS if hasattr(sys, '_MEIPASS') else os.path.dirname(__file__)
        logging.debug(f'base path = {base_path} or {os.path.dirname(__file__)}')
        bundled_file = os.path.join(base_path, filename)
        if os.path.isfile(bundled_file):
            return bundled_file


def connect(dbFile, con, target):
    db = QSqlDatabase.addDatabase('QSQLITE', connectionName=con)
    dbPath = getPath(dbFile)
    if QtCore.QFile.exists(os.path.join(dbPath, dbFile)):
        db.setDatabaseName(os.path.join(dbPath, dbFile))
        db.open()
        logging.info(f'{dbFile} open: {db.isOpen()}  Connection = "{db.connectionName()}"')
        logging.debug(f'tables available = {db.tables()}')
        checkVersion(db, target, dbFile)  # check that the actual database version matches the target version
    else:
        logging.info('Database file {dbPath}{dbFile} is missing')
        popUp(QtTSA, 'Database file is missing', 'Ok', 'Critical')
        return
    return db


def disconnect(db):
    db.close()
    logging.info(f'Database {db.databaseName()} open: {db.isOpen()}')
    QSqlDatabase.removeDatabase(db.databaseName())


def checkVersion(db, target, dbFile):
    existing = fetchVersion(db)
    logging.info(f'Database version is {existing}, expected {target}')
    if existing != target:
        message = "This version of QtTinySA needs database version " + str(target) + ".\n\n" + \
                "Database " + db.databaseName() + "\nversion " + str(existing) + \
                " may not be compatible.\n" + \
                "\nClicking OK will replace it with version " + str(target) + \
                " and will reset some settings."
        replace = popUp(QtTSA, message, 'OkC', 'Question')
        if replace == QMessageBox.StandardButton.Ok:
            impex = ModelView('frequencies', db, ())
            impex.tm.select()
            impex.unlimited()
            personalDir = platformdirs.user_config_dir(appname=app.applicationName(), appauthor=False)
            fileName = personalDir + "/frequencies_" + str(target) + ".csv"
            impex.exportData(fileName)
            logging.info(f'Renaming file {db.databaseName()} to {db.databaseName()}.{str(existing)}')
            disconnect(db)
            os.rename(db.databaseName(), db.databaseName() + '.' + str(existing))

            getPath(dbFile)  # this ought to return the same path as when it was run earlier in connect()
            db.open()  # the database connection has not changed, only the file, so can re-open it with the new file
            found = fetchVersion(db)
            logging.info(f'Found new database version {found}')
            if found != target:
                message = "Found new database version " + str(found) + "\nbut expected version " + str(target)
                restore = popUp(QtTSA, message, 'Ok', 'Info')
            message = "Restore your previous preset frequencies to the new database?"
            restore = popUp(QtTSA, message, 'OkC', 'Question')
            if restore == QMessageBox.StandardButton.Ok:
                impex.tm.select()
                impex.unlimited()
                impex.deleteRow(False)
                logging.info(f'Deleting records from frequencies table of database version {found}')
                impex.tm.submit()
                impex.importData(fileName)


def fetchVersion(db):
    query = QSqlQuery(db)
    query.exec("PRAGMA user_version;")  # execute PRAGMA command to fetch the user-defined version number
    query.next()  # advances to the result row, and query.value(0) retrieves the user version.
    version = query.value(0)
    query.clear()
    return version


def exit_handler():
    if len(tinySA.ports) != 0:
        # save the marker frequencies
        record = numbers.tm.record(0)
        record.setValue('m1f', float(M1.line.value()))
        record.setValue('m2f', float(M2.line.value()))
        record.setValue('m3f', float(M3.line.value()))
        record.setValue('m4f', float(M4.line.value()))
        numbers.tm.setRecord(0, record)

        if tinySA.sweeping:
            tinySA.sweeping = False  # tell the measurement thread to stop
            while tinySA.threadRunning:
                time.sleep(0.05)  # wait for measurements to stop
        tinySA.resume()
        tinySA.usbSend()
        tinySA.closePort()  # close USB connection

    # save the gui field values and checkbox states
    checkboxes.dwm.submit()
    numbers.dwm.submit()
    disconnect(config)

    logging.info('QtTinySA Closed')


def popUp(window, message, button, icon):
    icons = {'Warn': QMessageBox.Icon.Warning, 'Info': QMessageBox.Icon.Information,
             'Critical': QMessageBox.Icon.Critical, 'Question': QMessageBox.Icon.Question}
    buttons = {'Ok': QMessageBox.StandardButton.Ok, 'Cancel': QMessageBox.StandardButton.Cancel,
               'OkC': QMessageBox.StandardButton.Ok | QMessageBox.StandardButton.Cancel}
    msg = QMessageBox(parent=(window))
    msg.setIcon(icons.get(icon))
    msg.setText(message)
    msg.setStandardButtons(buttons.get(button))
    return msg.exec()


def freqMarkers():
    M1.delFreqMarkers()
    M2.delFreqMarkers()
    presetmarker.unlimited()
    for i in range(0, presetmarker.tm.rowCount()):
        try:
            startF = presetmarker.tm.record(i).value('StartF')
            stopF = presetmarker.tm.record(i).value('StopF')
            colour = presetmarker.tm.record(i).value('colour')
            name = presetmarker.tm.record(i).value('name')
            if QtTSA.presetMarker.isChecked() and presetmarker.tm.record(i).value('visible') and stopF in (0, ''):
                M1.addFreqMarker(startF, colour, name, band=False)
                if QtTSA.presetLabel.isChecked() and QtTSA.presetLabel.checkState() == 2:
                    M1.marker.label.setAngle(90)
                    M1.marker.label.setPosition(0)
            if QtTSA.presetMarker.isChecked() and stopF not in (0, '', startF):  # it's a band marker
                M1.addFreqMarker(startF, colour, name)
                M2.addFreqMarker(stopF, colour, name)
        except ValueError:
            logging.info('freqMarkers(): value error')
            continue


def freqMarkerLabel():
    freqMarkers()


def isMixerMode():
    if bandstype.freq == 0:
        QtTSA.mixerMode.setVisible(False)
        QtTSA.start_freq.setStyleSheet('background-color:None')
        QtTSA.stop_freq.setStyleSheet('background-color:None')
        QtTSA.centre_freq.setStyleSheet('background-color:None')
        QtTSA.start_freq.setMaximum(tinySA.maxF)
        QtTSA.centre_freq.setMaximum(tinySA.maxF)
        QtTSA.stop_freq.setMaximum(tinySA.maxF)
    else:
        QtTSA.mixerMode.setVisible(True)
        QtTSA.start_freq.setStyleSheet('background-color:lightGreen')
        QtTSA.stop_freq.setStyleSheet('background-color:lightGreen')
        QtTSA.centre_freq.setStyleSheet('background-color:lightGreen')
        QtTSA.start_freq.setMaximum(100000)
        QtTSA.centre_freq.setMaximum(100000)
        QtTSA.stop_freq.setMaximum(100000)


def setSize():
    QtTSA.waterfall.setMaximumSize(QtCore.QSize(16777215, QtTSA.waterfall_size.value()))


def createPolarGrid(rings, radius):
    '''Draw concentric circles and radial lines to simulate polar axes.'''
    pattern.plotwidget.setAspectLocked(True)
    pattern.plotwidget.hideAxis('bottom')
    pattern.plotwidget.hideAxis('left')
    for i in range(1, rings + 1):
        r = i * radius / rings
        circle = QtWidgets.QGraphicsEllipseItem(-r, -r, 2 * r, 2 * r)
        circle.setPen(pyqtgraph.mkPen('grey', width=0.3))
        pattern.plotwidget.addItem(circle)
    r = radius - (radius/(rings*3))
    circle = QtWidgets.QGraphicsEllipseItem(-r, -r, 2 * r, 2 * r)
    circle.setPen(pyqtgraph.mkPen('red', width=0.3))
    pattern.plotwidget.addItem(circle)

    # Add radial lines
    for angle_deg in range(0, 360, 15):
        angle_rad = np.deg2rad(angle_deg)
        x = radius * np.cos(angle_rad)
        y = radius * np.sin(angle_rad)
        line = QtWidgets.QGraphicsLineItem(0, 0, x, y)
        line.setPen(pyqtgraph.mkPen('grey', width=0.5))
        pattern.plotwidget.addItem(line)


def startPolarPlot():
    if not fading.ui.isVisible():
        popUp(QtTSA, 'Measurement requires Signal Level Monitor window to be open', 'Ok', 'Critical')
    else:
        M1.setPolarPlot()
        M2.setPolarPlot()
        M3.setPolarPlot()
        M4.setPolarPlot()


def correction_window():
    offset.progress.setValue(0)
    offset.ui.show()


def connectActive():
    '''Connect signals from controls that send messages to tinySA or use trace data.  Called by setGUI().'''

    QtTSA.atten_box.valueChanged.connect(tinySA.attenuator)
    QtTSA.atten_auto.clicked.connect(tinySA.attenuator)
    QtTSA.spur_box.currentIndexChanged.connect(tinySA.spur)
    QtTSA.lna_box.clicked.connect(tinySA.lna)
    QtTSA.points_auto.stateChanged.connect(pointsChanged)
    QtTSA.points_box.editingFinished.connect(pointsChanged)
    QtTSA.setRange.clicked.connect(tinySA.mouseScaled)
    QtTSA.band_box.activated.connect(band_changed)
    QtTSA.rbw_box.currentIndexChanged.connect(tinySA.rbwChanged)
    QtTSA.rbw_auto.clicked.connect(tinySA.rbwChanged)

    # frequencies
    QtTSA.start_freq.editingFinished.connect(tinySA.freq_changed)
    QtTSA.stop_freq.editingFinished.connect(tinySA.freq_changed)
    QtTSA.centre_freq.valueChanged.connect(lambda: tinySA.freq_changed(True))  # centre/span mode
    QtTSA.span_freq.valueChanged.connect(lambda: tinySA.freq_changed(True))  # centre/span mode

    QtTSA.sampleRepeat.valueChanged.connect(tinySA.sampleRep)

    # 3D graph controls
    QtTSA.orbitL.clicked.connect(lambda: tinySA.orbit3D(1, True))
    QtTSA.orbitR.clicked.connect(lambda: tinySA.orbit3D(-1, True))
    QtTSA.orbitU.clicked.connect(lambda: tinySA.orbit3D(-1, False))
    QtTSA.orbitD.clicked.connect(lambda: tinySA.orbit3D(1, False))
    QtTSA.timeF.clicked.connect(lambda: tinySA.axes3D(-1, 'X'))
    QtTSA.timeR.clicked.connect(lambda: tinySA.axes3D(1, 'X'))
    QtTSA.freqR.clicked.connect(lambda: tinySA.axes3D(-1, 'Y'))
    QtTSA.freqL.clicked.connect(lambda: tinySA.axes3D(1, 'Y'))
    QtTSA.signalUp.clicked.connect(lambda: tinySA.axes3D(-1, 'Z'))
    QtTSA.signalDown.clicked.connect(lambda: tinySA.axes3D(1, 'Z'))
    QtTSA.gridF.clicked.connect(lambda: tinySA.grid(1))
    QtTSA.gridR.clicked.connect(lambda: tinySA.grid(-1))
    QtTSA.zoom.sliderMoved.connect(tinySA.zoom3D)
    QtTSA.reset3D.clicked.connect(tinySA.reset3D)
    QtTSA.timeSpectrum.clicked.connect(lambda: QtTSA.stackedWidget.setCurrentWidget(QtTSA.View3D))
    QtTSA.analyser.clicked.connect(lambda: QtTSA.stackedWidget.setCurrentWidget(QtTSA.ViewNormal))

    # filebrowse
    filebrowse.download.clicked.connect(lambda: tinySA.saveFile(True))
    filebrowse.saveAll.clicked.connect(lambda: tinySA.saveFile(False))
    filebrowse.listWidget.itemClicked.connect(tinySA.fileShow)

    # marker dragging
    M1.line.sigPositionChanged.connect(M1.setDelta)
    M2.line.sigPositionChanged.connect(M2.setDelta)
    M3.line.sigPositionChanged.connect(M3.setDelta)
    M4.line.sigPositionChanged.connect(M4.setDelta)
    M1.deltaline.sigPositionChanged.connect(M1.deltaMoved)
    M2.deltaline.sigPositionChanged.connect(M2.deltaMoved)
    M3.deltaline.sigPositionChanged.connect(M3.deltaMoved)
    M4.deltaline.sigPositionChanged.connect(M4.deltaMoved)

    # Sweep time
    # QtTSA.sweepTime.valueChanged.connect(lambda: tinySA.sweepTime(QtTSA.sweepTime.value()))

    # level calibration
    offset.correction_mode.currentTextChanged.connect(correction_filter)
    offset.filter_box.stateChanged.connect(correction_filter)
    offset.read_button.clicked.connect(correction.read_tables)
    offset.upload_button.clicked.connect(correction.upload_correction)

# SDR
def showSDR_Audio_Popup():
    # show current frequency in SDR Audio popup
    sdr_audio_popup.ui.frequency_label.setText(f"Frequency: {QtTSA.start_freq.value()} MHz")
    sdr_audio_popup.ui.show()

def connectPassive():
    # Connect signals from GUI controls that don't cause messages to go to the tinySA

    QtTSA.memBox.valueChanged.connect(memChanged)

    # Quit
    QtTSA.actionQuit.triggered.connect(app.closeAllWindows)
    
    QtTSA.scan_button.clicked.connect(tinySA.scan)
    QtTSA.run3D.clicked.connect(tinySA.scan)

    # marker setting within span range
    QtTSA.mkr_start.clicked.connect(markerToStart)
    QtTSA.mkr_centre.clicked.connect(markerToCentre)

    # marker tracking level
    QtTSA.m1track.valueChanged.connect(lambda: M1.setLevel(QtTSA.m1track.value()))
    QtTSA.m2track.valueChanged.connect(lambda: M2.setLevel(QtTSA.m2track.value()))
    QtTSA.m3track.valueChanged.connect(lambda: M3.setLevel(QtTSA.m3track.value()))
    QtTSA.m4track.valueChanged.connect(lambda: M4.setLevel(QtTSA.m4track.value()))

    # connect GUI controls that set marker associated trace
    QtTSA.m1trace.valueChanged.connect(lambda: M1.traceLink(QtTSA.m1trace.value()))
    QtTSA.m2trace.valueChanged.connect(lambda: M2.traceLink(QtTSA.m2trace.value()))
    QtTSA.m3trace.valueChanged.connect(lambda: M3.traceLink(QtTSA.m3trace.value()))
    QtTSA.m4trace.valueChanged.connect(lambda: M4.traceLink(QtTSA.m4trace.value()))

    # marker type changes
    QtTSA.m1_type.activated.connect(M1.mType)
    QtTSA.m2_type.activated.connect(M2.mType)
    QtTSA.m3_type.activated.connect(M3.mType)
    QtTSA.m4_type.activated.connect(M4.mType)

    # frequency band and fixed markers
    QtTSA.presetMarker.clicked.connect(freqMarkers)
    QtTSA.presetLabel.clicked.connect(freqMarkerLabel)
    QtTSA.addBandPreset.clicked.connect(addBand)
    QtTSA.addFix.clicked.connect(addFixed)
    QtTSA.filterBox.currentTextChanged.connect(freqMarkers)

    # trace checkboxes
    QtTSA.trace1.stateChanged.connect(T1.enable)
    QtTSA.trace2.stateChanged.connect(T2.enable)
    QtTSA.trace3.stateChanged.connect(T3.enable)
    QtTSA.trace4.stateChanged.connect(T4.enable)

    # trace type changes
    QtTSA.t1_type.activated.connect(T1.tType)
    QtTSA.t2_type.activated.connect(T2.tType)
    QtTSA.t3_type.activated.connect(T3.tType)
    QtTSA.t4_type.activated.connect(T4.tType)

    # preset freqs and settings
    presetFreqs.addPs.clicked.connect(bands.addRow)
    presetFreqs.deletePs.clicked.connect(lambda: bands.deleteRow(True))
    presetFreqs.deleteAll.clicked.connect(lambda: bands.deleteRow(False))
    presetFreqs.freqTable.clicked.connect(lambda: bands.tableClicked(presetFreqs.freqTable))
    presetFreqs.typeTable.clicked.connect(lambda: bandstype.tableClicked(presetFreqs.typeTable))
    presetFreqs.addPsType.clicked.connect(bandstype.addRow)
    presetFreqs.deletePsType.clicked.connect(bandstype.deletePsType)
    presetFreqs.clearFilter.clicked.connect(bands.showAll)

    presetFreqs.ui.finished.connect(setPreferences)  # update database checkboxes table on dialogue window close

    presetFreqs.exportPs.pressed.connect(lambda: bands.exportData(''))
    presetFreqs.importPs.pressed.connect(lambda: bands.importData(''))
    settings.deviceBox.activated.connect(testComPort)

    QtTSA.filterBox.currentTextChanged.connect(lambda: bandselect.filterType(False, QtTSA.filterBox.currentText()))
    QtTSA.actionPresets.triggered.connect(dialogPrefs)  # open preferences dialogue when its menu is clicked
    QtTSA.actionSettings.triggered.connect(settings.ui.show)
    QtTSA.actionCorrection.triggered.connect(correction_window)

    # preferences
    QtTSA.actionAbout_QtTinySA.triggered.connect(about)

    # Waterfall
    QtTSA.waterfall_size.valueChanged.connect(setSize)

    # Measurement menu
    QtTSA.actionPhNoise.triggered.connect(phasenoise.ui.show)
    QtTSA.actionFading.triggered.connect(fading.ui.show)
    QtTSA.actionPattern.triggered.connect(pattern.ui.show)



    # phase noise
    phasenoise.centre.clicked.connect(centreToMarker)

    # File menu
    
    # SDR Add
    QtTSA.actionSDR_Audio_Popup.triggered.connect(showSDR_Audio_Popup)

    # SDR Audio popup connections
    sdr_audio_popup.ui.toggle_audio_button.clicked.connect(tinySA.toggle_sdr_audio)
    sdr_audio_popup.ui.plus_freq_button.clicked.connect(tinySA.sdr_audio_plus_freq)
    sdr_audio_popup.ui.minus_freq_button.clicked.connect(tinySA.sdr_audio_minus_freq)
    
    QtTSA.actionBrowse_TinySA.triggered.connect(tinySA.dialogBrowse)

    # polar pattern
    pattern.measure.clicked.connect(startPolarPlot)

    # correction
    offset.export_button.clicked.connect(lambda: correction.exportData(''))
    offset.import_button.clicked.connect(lambda: correction.importData(''))

    #


###############################################################################
# Instantiate classes

tinySA = Analyser()

# create QApplication for the GUI
app = QtWidgets.QApplication([])
app.setApplicationName('QtTinySA')
app.setApplicationVersion(' v1.2.2')

QtTSA = uic.loadUi(app_dir('spectrum.ui'))
presetFreqs = CustomDialogue(app_dir('bands.ui'))
settings = CustomDialogue(app_dir('settings.ui'))
filebrowse = CustomDialogue(app_dir('filebrowse.ui'))
phasenoise = CustomDialogue(app_dir('phasenoise.ui'))
fading = CustomDialogue(app_dir('fading.ui'))
pattern = CustomDialogue(app_dir('pattern.ui'))
offset = CustomDialogue(app_dir('offset.ui'))

# SDR
sdr_audio_popup = CustomDialogue(app_dir('sdr_audio_popup.ui'))

# Markers
multiplot = pyqtgraph.GraphicsLayout()  # for plotting marker signal level over time
fading.grView.setCentralItem(multiplot)
M1 = Marker('1', 0.1)
M2 = Marker('2', 0.9)
M3 = Marker('3', 1.7)
M4 = Marker('4', 2.5)

# Traces
T1 = Trace('1')
T2 = Trace('2')
T3 = Trace('3')
T4 = Trace('4')

# limit lines
best = Limit('gold', None, -25, movable=False)
maximum = Limit('red', None, 0, movable=False)
damage = Limit('red', None, 6, movable=False)
threshold = Limit('cyan', None, settings.peakThreshold.value(), movable=True)
lowF = Limit('cyan', (QtTSA.start_freq.value() + QtTSA.span_freq.value()/20)*1e6, None, movable=True)
highF = Limit('cyan', (QtTSA.stop_freq.value() - QtTSA.span_freq.value()/20)*1e6, None, movable=True)
reference = Limit('yellow', None, -110, movable=True)

best.create(True, '|>', 0.99)
maximum.create(True, '|>', 0.99)
damage.create(False, '|>', 0.99)
threshold.create(True, '<|', 0.99)
lowF.create(True, '|>', 0.01)
highF.create(True, '<|', 0.01)
reference.create(True, '<|>', 0.99)


# Database and models for recording and playback (can't get multiple databases to work)
# saveData = connect("QtTSArecording.db", "measurements")

# data = modelView('data', saveData)
# ? = modelView('settings', saveData)
# data.tm.select()
# ?.tm.select()


###############################################################################
# GUI settings

# pyqtgraph settings for spectrum display
QtTSA.graphWidget.setYRange(-112, -20)
QtTSA.graphWidget.setDefaultPadding(padding=0.005)
QtTSA.graphWidget.showGrid(x=True, y=True)
QtTSA.graphWidget.setLabel('bottom', '', units='Hz')

# pyqtgraph settings for waterfall and histogram display
QtTSA.waterfall.setDefaultPadding(padding=0.005)
QtTSA.waterfall.getPlotItem().hideAxis('bottom')
QtTSA.waterfall.setLabel('left', '.', **{'color': '#FFF', 'font-size': '2pt'})
QtTSA.waterfall.invertY(True)

QtTSA.histogram.setDefaultPadding(padding=0)
QtTSA.histogram.plotItem.invertY(True)
QtTSA.histogram.getPlotItem().hideAxis('bottom')
QtTSA.histogram.getPlotItem().hideAxis('left')

# pyqtgraph settings for Phase Noise
phasenoise.plotWidget.setYRange(-120, -40)
phasenoise.plotWidget.plotItem.showGrid(x=True, y=True, alpha=0.5)
phasenoise.plotWidget.plotItem.setLogMode(x=True)
phasenoise.plotWidget.setLabel('bottom', 'Offset Frequency', units='Hz')
phasenoise.plotWidget.setLabel('left', 'Phase Noise', units='dBc/Hz')

# pyqtgraph settings for antenna pattern display
createPolarGrid(4, 40)


###############################################################################
# set up the application
logging.info(f'{app.applicationName()}{app.applicationVersion()}')

# Database and models for configuration settings
config = connect("QtTSAprefs.db", "settings", 122)  # third parameter is the database version

# field mapping of the checkboxes and numbers database tables, for storing startup configuration
maps = ModelView('mapping', config, ())
maps.tm.select()

# populate the preset frequencies relational table in the presetFreqs window
bands = ModelView('frequencies', config, ())
bands.tm.setSort(3, QtCore.Qt.SortOrder.AscendingOrder)
bands.tm.setHeaderData(5, QtCore.Qt.Orientation.Horizontal, "visible")
bands.tm.setEditStrategy(QSqlRelationalTableModel.EditStrategy.OnRowChange)
bands.tm.setRelation(2, QSqlRelation("freqtype", "ID", "preset"))  # set "type" column to a freq type choice combo box
bands.tm.setRelation(5, QSqlRelation("boolean", "ID", "value"))  # set "view" column to a True/False choice combo box
bands.tm.setRelation(6, QSqlRelation("SVGColour", "ID", "colour"))  # set "marker" column to a colours choice combo box
presets = QSqlRelationalDelegate(presetFreqs.freqTable)
presetFreqs.freqTable.setItemDelegate(presets)
colHeader = presetFreqs.freqTable.horizontalHeader()
colHeader.setSectionResizeMode(QtWidgets.QHeaderView.ResizeMode.ResizeToContents)
bands.tm.select()

# populate the preset Types table in the preset frequencies window
bandstype = ModelView('freqtype', config, ())
bandstype.tm.select()
presetFreqs.typeTable.setModel(bandstype.tm)
presetFreqs.typeTable.hideColumn(0)  # hide primary key so user can't change it

# populate the correction values table in the correction window
correction = ModelView('correction', config, (0, 1, 2, 3))
c_header = offset.c_table.horizontalHeader()
c_header.setSectionResizeMode(QtWidgets.QHeaderView.ResizeMode.ResizeToContents)
correction.tm.select()
offset.c_table.setModel(correction.tm)
offset.c_table.hideColumn(0)

# to lookup the preset bands and markers colours because can't get the relationships to work
colours = ModelView('SVGColour', config, ())
colours.tm.select()

# for the main screen preset markers, which need different filtering to the preset frequencies window
presetmarker = ModelView('frequencies', config, ())
presetmarker.tm.setRelation(6, QSqlRelation('SVGColour', 'ID', 'colour'))
presetmarker.tm.setRelation(2, QSqlRelation('freqtype', 'ID', 'preset'))
presetmarker.tm.setSort(3, QtCore.Qt.SortOrder.AscendingOrder)
presetmarker.tm.select()

# populate the ui band selection combo box; needs different filter to the main and preset frequencies window
bandselect = ModelView('frequencies', config, ())
bandselect.tm.setRelation(2, QSqlRelation('freqtype', 'ID', 'preset'))
bandselect.tm.setRelation(5, QSqlRelation('boolean', 'ID', 'value'))
bandselect.tm.setRelation(6, QSqlRelation('SVGColour', 'ID', 'colour'))
bandselect.tm.setSort(3, QtCore.Qt.SortOrder.AscendingOrder)
QtTSA.band_box.setModel(bandselect.tm)
QtTSA.band_box.setModelColumn(1)
bandselect.tm.select()

# populate the preset bands and markers dialogue and ui filter combo boxes
QtTSA.filterBox.setModel(bandstype.tm)
QtTSA.filterBox.setModelColumn(1)

# connect the preset frequencies window table widget to the data model
presetFreqs.freqTable.setModel(bands.tm)
presetFreqs.freqTable.hideColumn(0)  # ID

# connect the settings window trace colours widget to the data model
tracecolours = ModelView('trace', config, (0, 1))
tracecolours.tm.setRelation(2, QSqlRelation('SVGColour', 'ID', 'colour'))
tracecolours.tm.setEditStrategy(QSqlRelationalTableModel.EditStrategy.OnFieldChange)
tracesettings = QSqlRelationalDelegate(settings.colourTable)
settings.colourTable.setItemDelegate(tracesettings)
traceHeader = settings.colourTable.horizontalHeader()
traceHeader.setSectionResizeMode(QtWidgets.QHeaderView.ResizeMode.ResizeToContents)
settings.colourTable.setModel(tracecolours.tm)
settings.colourTable.hideColumn(0)  # ID
settings.colourTable.verticalHeader().setVisible(True)
tracecolours.tm.select()

settingstext = ModelView('settings', config, ())

# Map data tables to presets/settings/GUI fields *lines need to be in this order and here or the mapping doesn't work*
checkboxes = ModelView('checkboxes', config, ())
checkboxes.createMapper()
checkboxes.mapWidget('checkboxes')  # uses mapping table from database
checkboxes.tm.select()
checkboxes.dwm.setCurrentIndex(0)  # 0 = (last used) default settings

# populate the spur combo box
QtTSA.spur_box.addItems(['off', 'on', 'auto'])
QtTSA.spur_box.setCurrentIndex(2)

# populate the rbw combobox
rbwtext = ModelView('combo', config, ())
rbwtext.tm.setFilter('type = "rbw"')
QtTSA.rbw_box.setModel(rbwtext.tm)
rbwtext.tm.select()

# populate the trace comboboxes
tracetext = ModelView('combo', config, ())
tracetext.tm.setFilter('type = "trace"')
QtTSA.t1_type.setModel(tracetext.tm)
QtTSA.t2_type.setModel(tracetext.tm)
QtTSA.t3_type.setModel(tracetext.tm)
QtTSA.t4_type.setModel(tracetext.tm)
tracetext.tm.select()

# populate the marker comboboxes
markertext = ModelView('combo', config, ())
markertext.tm.setFilter('type = "marker"')
QtTSA.m1_type.setModel(markertext.tm)
QtTSA.m2_type.setModel(markertext.tm)
QtTSA.m3_type.setModel(markertext.tm)
QtTSA.m4_type.setModel(markertext.tm)
markertext.tm.select()

# populate the correction comboboxes
correctiontext = ModelView('combo', config, ())
correctiontext.tm.setFilter('type = "correction"')
offset.correction_mode.setModel(correctiontext.tm)
correctiontext.tm.select()

# The models for saving number, marker and trace settings
numbers = ModelView('numbers', config, ())
numbers.createMapper()
numbers.mapWidget('numbers')  # uses mapping table from database
numbers.tm.select()
numbers.dwm.setCurrentIndex(0)

QtTSA.show()
QtTSA.setWindowTitle(app.applicationName() + app.applicationVersion())
QtTSA.setWindowIcon(QIcon(os.path.join(basedir, 'tinySAsmall.png')))

# connect GUI controls that don't interfere with restoration of data at startup
connectPassive()

tinySA.setGUI()

# try to open a USB connection to the TinySA hardware
usbCheck = QtCore.QTimer()
usbCheck.timeout.connect(tinySA.isConnected)
usbCheck.start(500)  # check again every 500mS

###############################################################################
# run the application until the user closes it

try:
    app.exec()
finally:
    exit_handler()  # close cleanly
