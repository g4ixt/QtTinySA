#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Originally created on Tue 1 May 2023 @author: Ian Jefferson G4IXT
TinySA Ultra GUI programme using Qt5 and PyQt.

This code attempts to replicate some of the TinySA Ultra on-screen commands and to provide PC control.
Development took place on Kubuntu 22.04LTS with Python 3.9 and PyQt5 using Spyder in Anaconda.
Not tested in any Windows version.

TinySA and TinySA Ultra are trademarks of Erik Kaashoek and are used with permission.

TinySA commands are based on Erik's Python examples:
http://athome.kaashoek.com/tinySA/python/

The serial communication commands are based on the Python NanoVNA/TinySA Toolset of Martin Ho-Ro:
https://github.com/Ho-Ro

"""

import time
import logging
import numpy as np
from PyQt5 import QtWidgets, QtCore
from PyQt5.QtCore import pyqtSlot, pyqtSignal, QRunnable, QObject, QThreadPool
from PyQt5.QtWidgets import QMessageBox
import pyqtgraph
import QtTinySpectrum  # the GUI
import struct
import serial
from serial.tools import list_ports

#  For 3D
import pyqtgraph.opengl as pyqtgl
import matplotlib.pyplot as plt

logging.basicConfig(format="%(message)s", level=logging.INFO)
threadpool = QThreadPool()

# pyqtgraph pens
red = pyqtgraph.mkPen(color='r', width=1.0)
yellow = pyqtgraph.mkPen(color='y', width=1.0)
white = pyqtgraph.mkPen(color='w', width=1.0)
cyan = pyqtgraph.mkPen(color='c', width=1.0)
red_dash = pyqtgraph.mkPen(color='r', width=0.5, style=QtCore.Qt.DashLine)
blue_dash = pyqtgraph.mkPen(color='b', width=0.5,  style=QtCore.Qt.DashLine)

###############################################################################
# classes


class analyser:
    def __init__(self):
        # self.dev = getport()
        self.dev = None
        self._frequencies = None
        self.sweeping = False
        self.signals = WorkerSignals()
        self.signals.result.connect(self.sigProcess)
        self.timeout = 1
        self.scanCount = 0
        self.runTimer = QtCore.QElapsedTimer()
        self.scale = 174
        self.scanMemory = 50
        self.scan3D = False

    @property
    def frequencies(self):
        # what does this do?
        return self._frequencies

    def initialise(self, doPopUp=True):
        # Get tinysa device automatically using hardware ID
        VID = 0x0483  # 1155
        PID = 0x5740  # 22336
        try:
            device_list = list_ports.comports()
        except serial.SerialException:
            logging.info('serial port exception')
        for x in device_list:
            if x.vid == VID and x.pid == PID:
                self.dev = x.device
        if self.dev is None:
            activeButtons(False) # do not trigger serial commands
            ui.version.setText('Not Connected')
            if doPopUp:
                popUp('TinySA not found', 'ok')
            return

        # amateur frequency band values (plus VHF radio)
        self.fBandStart = [1.8, 3.5, 7.0, 10.1, 14.0, 18.068, 21.0, 24.89, 28.0,
                           50.0, 70.0, 87.5, 144.0, 430.0, 1240, 2300, 2390, 3300, 5650]
        self.fBandStop = [2.0, 3.8, 7.1, 10.15, 14.35, 18.168, 21.45, 24.99, 29.7,
                          52.0, 70.5, 108.0, 146.0, 440.0, 1325, 2310, 2450, 3500, 5925]

        # TinySA Ultra resolution bandwidth filters in kHz
        self.resBW = ['0.2', '1', '3', '10', '30', '100', '300', '600', '850']

        hardware = self.version()
        # hardware = 'basic'
        logging.info(f'version = {hardware}')
        if hardware[:7] == 'tinySA4':  # It's an Ultra
            self.tinySA4 = True
            ui.spur_box.setTristate(True)  # TinySA Ultra has 'auto', 'on' and 'off' setting for Spur
            ui.spur_box.setCheckState(QtCore.Qt.PartiallyChecked)
            self.spur(1)  # 1 = auto
        else:
            self.tinySA4 = False
            self.scale = 128
            self.fBandStart = self.fBandStart[:13]  # TinySA Basic has a smaller frequency band range
            self.fBandStop = self.fBandStop[:13]
            self.resBW = self.resBW[2:8]  # TinySA Basic has fewer resolution bandwidth filters
            ui.spur_box.setTristate(False)  # TinySA Basic has only 'on' and 'off' setting for Spur'
            ui.spur_box.setChecked(True)
            self.spur(2)  # 2 = on

        # Basic has no lna
        ui.lna_label.setVisible(self.tinySA4)
        ui.lna_box.setVisible(self.tinySA4)
        ui.lna_box.setEnabled(self.tinySA4)

        # set the frequency band & rbw comboboxes to suit detected hardware
        bands = list(map(str, self.fBandStart))  # convert start freq float list to string list for GUI combobox
        bands = [freq for freq in bands]
        bands.insert(0, 'Band')
        self.resBW.insert(0, 'auto')
        ui.rbw_box.addItems(self.resBW)
        ui.band_box.addItems(bands)

        activeButtons(True) # enable ui components that trigger serial commands
        if self.tinySA4:
            self.lna()  # LNA off at first run

        # show hardware information in GUI
        ui.battery.setText(self.battery())
        ui.version.setText(hardware)

        S1.trace.show()

    def startMeasurement(self, startF, stopF):
        self.sweep = Worker(self.measurement, startF, stopF)  # workers are auto-deleted when thread stops
        self.sweeping = True
        self.sweepresults = np.full((self.scanMemory, self.points), -100, dtype=float)  # to do - add row count to GUI
        if ui.Enabled3D.isChecked():
            tinySA.createTimeSpectrum()
        threadpool.start(self.sweep)

    def serialSend(self, command):
        self.clearBuffer()
        with serial.Serial(port=self.dev, baudrate=3000000) as SA:  # baudrate does nothing for USB cnx
            SA.timeout = 1
            logging.debug(command)
            SA.write(command)
            SA.read_until(b'ch> ')  # skip command echo and prompt

    def serialQuery(self, command):
        self.clearBuffer()
        with serial.Serial(port=self.dev, baudrate=3000000) as SA:  # baudrate does nothing for USB cnx
            SA.timeout = 1
            logging.debug(command)
            SA.write(command)
            SA.read_until(command + b'\n')  # skip command echo
            response = SA.read_until(b'ch> ')
            logging.debug(response)
            return response[:-6].decode()  # remove prompt

    def set_frequencies(self, startF, stopF, points):
        # creates a np array of equi-spaced freqs in Hz (but doesn't set it on the tinySA)
        self.points = points
        self._frequencies = np.linspace(startF, stopF, self.points, dtype=int)
        logging.debug(f'frequencies = {self._frequencies}')

    def setRBW(self):
        if ui.rbw_box.currentIndex == 0:
            self.rbw = 'auto'
        else:
            self.rbw = ui.rbw_box.currentText()  # ui values are discrete ones in kHz
        rbw_command = f'rbw {self.rbw}\r'.encode()
        self.serialSend(rbw_command)

    def clearBuffer(self):
        with serial.Serial(self.dev, baudrate=3000000) as serialPort:  # baudrate does nothing for USB cnx
            serialPort.timeout = 1
            while serialPort.inWaiting():
                serialPort.read_all()  # keep the serial buffer clean
                time.sleep(0.1)

    def sweepTimeout(self, f_low, f_high):  # freqs are in Hz
        if self.rbw == 'auto':
            # rbw auto setting from tinySA: ~7 kHz per 1 MHz scan frequency span
            rbw = (f_high - f_low) * 7e-6
        else:
            rbw = float(self.rbw)
        # lower / upper limit
        if rbw < float(self.resBW[1]):
            rbw = float(self.resBW[1])
        elif rbw > float(self.resBW[-1]):
            rbw = float(self.resBW[-1])
        # timeout can be very long - use a heuristic approach
        # 1st summand is the scanning time, 2nd summand is the USB transfer overhead
        timeout = ((f_high - f_low) / 20e3) / (rbw ** 2) + self.points / 500
        if (ui.spur_box.checkState() == 1 and f_high > 8 * 1e8) or ui.spur_box.checkState() == 2:
            timeout *= 2  # scan time doubles with spur on or spur auto above 800 MHz
        # transfer is done in blocks of 20 points, this is the timeout for one block
        self.timeout = timeout * 20 / self.points + 1  # minimum is 1 second
        logging.debug(f'sweepTimeout = {self.timeout} s')

    def measurement(self, f_low, f_high):  # runs in a separate thread
        self.threadrunning = True
        while self.sweeping:
            with serial.Serial(self.dev, baudrate=3000000) as serialPort:  # baudrate does nothing for USB cnx
                serialPort.timeout = self.timeout
                scan_command = f'scanraw {int(f_low)} {int(f_high)} {int(self.points)}\r'.encode()
                serialPort.write(scan_command)
                index = 0
                serialPort.read_until(scan_command + b'\n{')  # skip command echo
                dataBlock = ''
                self.sweepresults[0] = self.sweepresults[1]  # populate each sweep with previous sweep as starting point
                while dataBlock != b'}ch':  # if dataBlock is '}ch' it's reached the end of the scan points
                    dataBlock = (serialPort.read(3))  # read a block of 3 bytes of data
                    logging.debug(f'dataBlock: {dataBlock}\n')
                    if dataBlock != b'}ch':
                        logging.debug(f'index {index} elapsed time = {self.runTimer.nsecsElapsed()/1e6}')
                        c, data = struct.unpack('<' + 'cH', dataBlock)
                        logging.debug(f'dataBlock: {dataBlock} data: {data}\n')
                        dBm_power = (data / 32) - self.scale  # scale 0..4095 -> -128..-0.03 dBm
                        # write each measurement into sweepresults and emit
                        self.sweepresults[0, index] = dBm_power
                        if index // 20 == index / 20 or index == (self.points - 1):
                            self.signals.result.emit(self.sweepresults)
                        index += 1
                    logging.debug(f'level = {dBm_power}dBm')
                serialPort.read(2)  # discard the command prompt
            # store each sweep in an array with most recent at index 0
            self.sweepresults = np.roll(self.sweepresults, 1, axis=0)
        self.threadrunning = False

    def sigProcess(self, signaldBm):  # signaldBm is emitted from the worker thread
        signalAvg = np.average(signaldBm[:ui.avgSlider.value(), ::], axis=0)
        signalMax = np.amax(signaldBm[:100, ::], axis=0)
        signalMin = np.amin(signaldBm[:100, ::], axis=0)
        options = {'Normal': signaldBm[0], 'Average': signalAvg, 'Max': signalMax, 'Min': signalMin}
        S1.updateGUI(options.get(S1.traceType))
        S2.updateGUI(options.get(S2.traceType))
        S3.updateGUI(options.get(S3.traceType))
        S4.updateGUI(options.get(S4.traceType))
        if ui.Enabled3D.isChecked():  # change this so that it only updates 3D once per sweep, for performance reasons
            self.updateTimeSpectrum()

    def createTimeSpectrum(self):  # To Do: move this into 'display' class
        x = np.arange(start=0, stop=self.scanMemory, step=1)  # the time axis depth
        y = np.arange(start=0, stop=self.points)  # the frequency axis width
        z = self.sweepresults + 120  # the measurement azis heights in dBm, convert to dBf
        logging.debug(f'z = {z}')
        #self.p2 = pyqtgl.GLSurfacePlotItem(x=-x, y=y, z=z, shader='normalColor', computeNormals=True, smooth=False)
        # self.p2.shader()['colorMap'] = np.array([20, -100, 0.5, -100, 1, 1, 0.2, -100, 2])
        # test
        zmax = 100
        zmin = 1
        self.p2 = pyqtgl.GLSurfacePlotItem(x=-x, y=y, z=z, shader = 'heightColor', computeNormals=False, smooth=False)

        # cmap = plt.get_cmap('jet')
        # minZ=1
        # maxZ=2
        # rgba_img = cmap((-minZ)/(maxZ -minZ))
        # self.p2 = pyqtgl.GLSurfacePlotItem(x=-x, y=y, z=z, colors=rgba_img, computeNormals=False, smooth=False)

        # This shader uses a uniform called "colorMap" to determine how to map the colors:
        #    red   = pow(z * colorMap[0] + colorMap[1], colorMap[2])
        #    green = pow(z * colorMap[3] + colorMap[4], colorMap[5])
        #    blue  = pow(z * colorMap[6] + colorMap[7], colorMap[8])

        self.p2.shader()['colorMap'] = np.array([0.8/zmin,   # red [0]
                                                  0.05,        # red   [1]
                                                  0.8,      # red   [2]
                                                  0.3/zmax,  # green   [3]
                                                  0.15,        # green  [4]
                                                  0.4,     # green      [5]
                                                 20/zmin,  # blue    [6]
                                                 0,    # blue        [7]
                                                  0.3])       # blue    [8]

        # self.p2.shader()['colorMap'] = np.array([.01/zmax,  # red 1
        #                                           0,         # red 2
        #                                           0.01,      # red 3
        #                                           0.01/zmax, # green 1
        #                                           0,         # green 2
        #                                           .05,       # green 3
        #                                           1/zmax,    # blue 1
        #                                           -zmin,     # blue 2
        #                                           1])        # blue 3
        # test
        self.p2.translate(-0.7*self.scanMemory, -self.points/2, -self.points/3)
        self.p2.scale(self.points/2000, 0.05, 0.05, local=False)
        self.p2.rotate(45, 0, 0, 1)
        ui.openGLWidget.addItem(self.p2)

        # Add a grid to the 3D view
        g = pyqtgl.GLGridItem()
        g.scale(0.5, 0.5, 0.5)
        g.rotate(-45, 0, 0, 1)
        g.translate(0, 0, 0)
        g.setSpacing(0.5, 0.5, 0.5)
        ui.openGLWidget.addItem(g)

    def updateTimeSpectrum(self):  # To Do: move this into 'display' class
        z = self.sweepresults + 120  # convert from dBm to dBf
        logging.debug(f'z = {z}')
        self.p2.setData(z=z)

    def pause(self):
        # pauses the sweeping in either input or output mode
        pause_command = 'pause\r'.encode()
        self.serialSend(pause_command)

    def resume(self):
        # resumes the sweeping in either input or output mode
        resume_command = 'resume\r'.encode()
        self.serialSend(resume_command)

    def battery(self):
        command = 'vbat\r'.encode()
        vbat = self.serialQuery(command)
        return vbat

    def version(self):
        command = 'version\r'.encode()
        version = self.serialQuery(command)
        return version

    def spur(self, sType=0):
        options = {0: 'spur off\r'.encode(), 1: 'spur auto\r'.encode(), 2: 'spur on\r'.encode()}
        command = options.get(sType)
        tinySA.serialSend(command)
        if sType == 1:
            ui.spur_box.setText('Auto')
        else:
            ui.spur_box.setText('')

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
        tinySA.serialSend(command)


class display:
    def __init__(self, name, pen):
        self.trace = ui.graphWidget.plot([], [], name=name, pen=pen, width=1)
        self.trace.hide()
        self.traceType = 'Normal'  # Normal, Average, Max, Min
        self.markerType = 'Normal'  # Normal, Delta; Peak
        self.vline = ui.graphWidget.addLine(88, 90, movable=True, name=name, pen=pyqtgraph.mkPen('g', width=0.5, style=QtCore.Qt.DashLine), label="{value:.2f}")
        self.vline.hide()
        self.fIndex = 0  # index of current marker freq in frequencies array
        self.dIndex = 0  # the difference between this marker and reference marker 1

    def setDiscrete(self):
        # update marker to discrete freq point nearest, if it's within the sweep range
        if self.vline.value() >= ui.start_freq.value() and self.vline.value() <= ui.stop_freq.value():
            try:
                for i in range(tinySA.points):
                    if tinySA.frequencies[i] / 1e6 >= self.vline.value():
                        self.vline.setValue(tinySA.frequencies[i] / 1e6)
                        self.fIndex = i
                        if self.markerType == 'Delta':
                            self.dIndex = self.fIndex - S1.fIndex
                        return
            except AttributeError:
                return

    def mStart(self):
        # set marker to the sweep start frequency
        self.fIndex = 0
        self.vline.setValue(ui.start_freq.value())

    def mType(self, uiBox):
        self.markerType = uiBox.currentText()
        self.dIndex = self.fIndex - S1.fIndex
        logging.debug(f'marker = type {self.markerType}')

    def tType(self, uiBox):
        self.traceType = uiBox.currentText()

    def mEnable(self, mkr):
        if mkr.isChecked():
            self.vline.show()
        else:
            self.vline.hide()

    def tEnable(self, trace):
        if trace.isChecked():
            self.trace.show()
        else:
            self.trace.hide()

    def mPeak(self, signal):
        peaks = np.argsort(-signal)  # finds the indices of the peaks in a copy of signal array; indices sorted desc
        if signal[peaks[0]] >= ui.mPeak.value():  # largest peak value is above the threshold set in GUI
            options = {'Peak1': peaks[0], 'Peak2': peaks[1], 'Peak3': peaks[2], 'Peak4': peaks[3]}
            self.fIndex = options.get(self.markerType)
            self.vline.setValue(tinySA.frequencies[self.fIndex] / 1e6)
            logging.debug(f'peaks = {peaks[:4]}')

    def mDelta(self):
        self.fIndex = S1.fIndex + self.dIndex
        if self.fIndex < 0:  # delta marker is now below sweep range
            self.fIndex = 0
        if self.fIndex > tinySA.points - 1:  # delta marker is now above sweep range
            self.fIndex = tinySA.points - 1
        self.vline.setValue(tinySA.frequencies[self.fIndex] / 1e6)
        # S1.vline.setPen(color='g')

    def updateGUI(self, signal):
        self.trace.setData((tinySA.frequencies/1e6), signal)
        if self.markerType != 'Normal' and self.markerType != 'Delta':  # then it must be a peak marker
            self.mPeak(signal)
        self.vline.label.setText(f'M{self.vline.name()} {tinySA.frequencies[self.fIndex]/1e6:.3f}MHz  {signal[self.fIndex]:.1f}dBm')


class WorkerSignals(QObject):
    error = pyqtSignal(str)
    result = pyqtSignal(np.ndarray)
    finished = pyqtSignal()


class Worker(QRunnable):
    '''Worker threads so that functions can run outside GUI event loop'''

    def __init__(self, fn, *args):
        super(Worker, self).__init__()
        self.fn = fn
        self.args = args
        self.signals = WorkerSignals()

    @pyqtSlot()
    def run(self):
        '''Initialise the runner'''
        logging.info(f'{self.fn.__name__} thread running')
        self.fn(*self.args)
        logging.info(f'{self.fn.__name__} thread stopped')


###############################################################################
# other methods

def scan():
    tinySA.scan3D = ui.Enabled3D.isChecked()
    if tinySA.dev is None:
        tinySA.initialise()
    if tinySA.dev is not None:
        if tinySA.sweeping:  # if it's running, stop it
            tinySA.sweeping = False  # tells the measurement thread to stop once current scan complete
            ui.scan_button.setEnabled(False)  # prevent repeat presses of 'stop'
            ui.scan_button.setText('Stopping ...')  # highlight for slow scans
            while tinySA.threadrunning:
                app.processEvents()  # keep updating the trace until the scan ends
                time.sleep(0.1)  # wait until the measurement thread stops using the serial comms
            ui.scan_button.setEnabled(True)
            activeButtons(True)
            ui.scan_button.setText('Run')  # toggle the 'Stop' button text
            ui.battery.setText(tinySA.battery())
            tinySA.resume()
        else:
            try:
                tinySA.pause()
                startF = ui.start_freq.value()*1e6
                stopF = ui.stop_freq.value()*1e6
                points = ui.points_box.value()
                tinySA.set_frequencies(startF, stopF, points)
                tinySA.clearBuffer()
                tinySA.setRBW()  # fetches rbw value from the GUI combobox and sends it to TinySA
                tinySA.sweepTimeout(startF, stopF)
                activeButtons(False)
                ui.scan_button.setText('Stop')  # toggle the 'Run' button text
                ui.battery.setText(tinySA.battery())
                app.processEvents()
                tinySA.startMeasurement(startF, stopF)  # runs measurement in separate thread
            except serial.SerialException:
                tinySA.dev = None
                tinySA.initialise()


def start_freq_changed():
    ui.band_box.setCurrentIndex(0)
    start = ui.start_freq.value()
    stop = ui.stop_freq.value()
    if start > stop:
        ui.stop_freq.setValue(start)
        stop = start
        stop_freq_changed()
    ui.graphWidget.setXRange(start, stop)

    command = f'sweep start {start * 1e6}\r'.encode()
    tinySA.serialSend(command)


def stop_freq_changed():
    ui.band_box.setCurrentIndex(0)
    start = ui.start_freq.value()
    stop = ui.stop_freq.value()
    if start > stop:
        ui.start_freq.setValue(stop)
        start = stop
        start_freq_changed()
    ui.graphWidget.setXRange(start, stop)

    command = f'sweep stop {stop * 1e6}\r'.encode()
    tinySA.serialSend(command)


def band_changed():
    index = ui.band_box.currentIndex()
    if index == 0:
        return
    else:
        index -= 1
        start = tinySA.fBandStart[index]
        ui.start_freq.setValue(start)
        start_freq_changed()
        stop = tinySA.fBandStop[index]
        ui.stop_freq.setValue(stop)
        stop_freq_changed()


def attenuate_changed():
    atten = ui.atten_box.value()
    if ui.atten_auto.isChecked():
        atten = 'auto'
        ui.atten_box.setEnabled(False)
    else:
        if not ui.lna_box.isChecked():  # attenuator and lna are switched so mutually exclusive
            ui.atten_box.setEnabled(True)
    command = f'attenuate {str(atten)}\r'.encode()
    tinySA.serialSend(command)


def spur_box():
    boxState = ui.spur_box.checkState()
    logging.debug(f'spur_box state = {boxState}')
    tinySA.spur(boxState)


def markerToStart():
    if ui.marker1.isChecked():
        S1.mStart()
    if ui.marker2.isChecked():
        S2.mStart()
    if ui.marker3.isChecked():
        S3.mStart()
    if ui.marker4.isChecked():
        S4.mStart()


def mkr1_moved():
    S1.setDiscrete()
    if S2.markerType == 'Delta' or S3.markerType == 'Delta' or S4.markerType == 'Delta':
        S1.vline.setPen(color='g', width=0.5)
    else:
        S1.vline.setPen(color='g', width=0.5, style=QtCore.Qt.DashLine)
    if S2.markerType == 'Delta':
        S2.mDelta()
    if S3.markerType == 'Delta':
        S3.mDelta()
    if S4.markerType == 'Delta':
        S4.mDelta()


def memChanged():
    depth = ui.memSlider.value()
    if depth < ui.avgSlider.value():
        ui.avgSlider.setValue(depth)
    tinySA.scanMemory = depth


def exit_handler():
    if tinySA.dev is not None:
        tinySA.sweeping = False
        time.sleep(1)  # allow time for measurements to stop
        tinySA.resume()
    app.processEvents()
    logging.info('Closed')


def popUp(message, button):
    msg = QMessageBox(parent=(window))
    msg.setIcon(QMessageBox.Warning)
    msg.setText(message)
    msg.addButton(button, QMessageBox.ActionRole)
    msg.exec_()


##############################################################################
# respond to GUI signals


def activeButtons(tF):
    # disable/enable buttons that send commands to TinySA (Because Comms are in use if scanning)
    ui.atten_box.setEnabled(tF)
    ui.atten_auto.setEnabled(tF)
    ui.spur_box.setEnabled(tF)
    ui.lna_box.setEnabled(tF and tinySA.tinySA4)
    ui.rbw_box.setEnabled(tF)
    ui.points_box.setEnabled(tF)
    ui.band_box.setEnabled(tF)
    ui.start_freq.setEnabled(tF)
    ui.stop_freq.setEnabled(tF)
    ui.memSlider.setEnabled(tF)
    ui.Enabled3D.setEnabled(tF)


###############################################################################
# Instantiate classes

tinySA = analyser()

app = QtWidgets.QApplication([])  # create QApplication for the GUI
window = QtWidgets.QMainWindow()
ui = QtTinySpectrum.Ui_MainWindow()
ui.setupUi(window)

# Traces & markers
S1 = display('1', yellow)
S2 = display('2', red)
S3 = display('3', cyan)
S4 = display('4', white)

###############################################################################
# GUI settings

# pyqtgraph settings for spectrum display
ui.graphWidget.setYRange(-110, 5)
ui.graphWidget.setXRange(87.5, 108)
ui.graphWidget.setBackground('k')  # black
ui.graphWidget.showGrid(x=True, y=True)
ui.graphWidget.addLine(y=6, movable=False, pen=red, label='', labelOpts={'position':0.05, 'color':('r')})
ui.graphWidget.addLine(y=0, movable=False, pen=red_dash, label='max', labelOpts={'position':0.025, 'color':('r')})
ui.graphWidget.addLine(y=-25, movable=False, pen=blue_dash, label='best', labelOpts={'position':0.025, 'color':('b')})
ui.graphWidget.setLabel('left', 'Signal', 'dBm')
ui.graphWidget.setLabel('bottom', 'Frequency MHz')

# marker label positions
S1.vline.label.setPosition(0.99)
S2.vline.label.setPosition(0.95)
S3.vline.label.setPosition(0.90)
S4.vline.label.setPosition(0.85)


###############################################################################
# Connect signals from buttons and sliders

ui.scan_button.clicked.connect(scan)
ui.rbw_box.currentTextChanged.connect(tinySA.setRBW)
ui.atten_box.valueChanged.connect(attenuate_changed)
ui.atten_auto.clicked.connect(attenuate_changed)
ui.start_freq.editingFinished.connect(start_freq_changed)
ui.stop_freq.editingFinished.connect(stop_freq_changed)
ui.spur_box.stateChanged.connect(spur_box)
ui.lna_box.stateChanged.connect(tinySA.lna)
ui.band_box.currentTextChanged.connect(band_changed)

S1.vline.sigPositionChanged.connect(mkr1_moved)
S2.vline.sigPositionChanged.connect(S2.setDiscrete)
S3.vline.sigPositionChanged.connect(S3.setDiscrete)
S4.vline.sigPositionChanged.connect(S4.setDiscrete)

ui.marker1.stateChanged.connect(lambda: S1.mEnable(ui.marker1))
ui.marker2.stateChanged.connect(lambda: S2.mEnable(ui.marker2))
ui.marker3.stateChanged.connect(lambda: S3.mEnable(ui.marker3))
ui.marker4.stateChanged.connect(lambda: S4.mEnable(ui.marker4))

ui.mkr_start.clicked.connect(markerToStart)
ui.m1_type.currentTextChanged.connect(lambda: S1.mType(ui.m1_type))
ui.m2_type.currentTextChanged.connect(lambda: S2.mType(ui.m2_type))
ui.m3_type.currentTextChanged.connect(lambda: S3.mType(ui.m3_type))
ui.m4_type.currentTextChanged.connect(lambda: S4.mType(ui.m4_type))

ui.trace1.stateChanged.connect(lambda: S1.tEnable(ui.trace1))
ui.trace2.stateChanged.connect(lambda: S2.tEnable(ui.trace2))
ui.trace3.stateChanged.connect(lambda: S3.tEnable(ui.trace3))
ui.trace4.stateChanged.connect(lambda: S4.tEnable(ui.trace4))

ui.t1_type.currentTextChanged.connect(lambda: S1.tType(ui.t1_type))
ui.t2_type.currentTextChanged.connect(lambda: S2.tType(ui.t2_type))
ui.t3_type.currentTextChanged.connect(lambda: S3.tType(ui.t3_type))
ui.t4_type.currentTextChanged.connect(lambda: S4.tType(ui.t4_type))

ui.memSlider.sliderMoved.connect(memChanged)

###############################################################################
# set up the application

ui.t1_type.addItems(['Normal', 'Average', 'Max', 'Min'])
ui.t2_type.addItems(['Normal', 'Average', 'Max', 'Min'])
ui.t3_type.addItems(['Normal', 'Average', 'Max', 'Min'])
ui.t3_type.setCurrentIndex(1)
ui.t4_type.addItems(['Normal', 'Average', 'Max', 'Min'])
ui.t4_type.setCurrentIndex(2)
ui.m1_type.addItems(['Normal', 'Peak1', 'Peak2', 'Peak3', 'Peak4'])  # Marker 1 is the reference for others
ui.m2_type.addItems(['Normal', 'Delta', 'Peak1', 'Peak2', 'Peak3', 'Peak4'])
ui.m3_type.addItems(['Normal', 'Delta', 'Peak1', 'Peak2', 'Peak3', 'Peak4'])
ui.m4_type.addItems(['Normal', 'Delta', 'Peak1', 'Peak2', 'Peak3', 'Peak4'])

tinySA.initialise(False)  # try to init, ignore failure

window.show()

###############################################################################
# run the application until the user closes it

try:
    app.exec()
finally:
    exit_handler()  # close cleanly
