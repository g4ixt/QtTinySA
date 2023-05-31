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
from PyQt5.QtWidgets import QMessageBox, QFileDialog
import pyqtgraph
import QtTinySpectrum  # the GUI
import struct
import serial
from serial.tools import list_ports

#  For 3D
from pyqtgraph.Qt import QtCore, QtGui
import pyqtgraph.opengl as pyqtgl
#  3D

logging.basicConfig(format="%(message)s", level=logging.INFO)

threadpool = QThreadPool()

VID = 0x0483  # 1155
PID = 0x5740  # 22336

# amateur frequency band values
fBandStart = [1.8, 3.5, 7.0, 10.1, 14.0, 18.068, 21.0, 24.89, 28.0,
              50.0, 70.0, 144.0, 430.0, 1240, 2300, 2390, 3300, 5650]

fBandStop = [2.0, 3.8, 7.1, 10.15, 14.35, 18.168, 21.45, 24.99, 29.7,
             52.0, 70.5, 146.0, 440.0, 1325, 2310, 2450, 3500, 5925]

bands = list(map(str, fBandStart))  # convert start freq float list to string list for GUI combobox
bands = [freq + ' MHz' for freq in bands]
bands.insert(0, 'Band')

###############################################################################
# classes


class analyser:
    def __init__(self, dev=None):
        self.dev = getport()
        self._frequencies = None
        self.sweeping = False
        self.signals = WorkerSignals()
        self.signals.result.connect(self.sigProcess)
        self.timeout = 1
        # self.marker = ui.graphWidget.addLine(0, 90, movable=True, pen='g', label="{value:.2f}")
        # self.marker.label.setPosition(0.1)

    @property
    def frequencies(self):
        # what does this do?
        return self._frequencies

    def startMeasurement(self, startF, stopF):
        self.sweep = Worker(self.spectrum, startF, stopF)  # workers are auto-deleted when thread stops
        self.sweeping = True
        self.sweepresults = np.zeros((100, self.points), dtype=float)  # to do - add row count to GUI
        threadpool.start(self.sweep)

    def serialSend(self, command):
        self.clearBuffer()
        with serial.Serial(port=self.dev, baudrate=3000000) as SA:  # baudrate does nothing for USB cnx
            SA.timeout = 1
            logging.debug(command)
            SA.write(command)
            SA.read_until(b'ch> ')  # skip command echo and prompt

    def set_frequencies(self, startF, stopF, points):  # needs update
        # creates a np array of equi-spaced freqs but doesn't actually set it on the tinySA
        self.points = points
        self._frequencies = np.linspace(startF, stopF, self.points)
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
            rbw = (f_high / 1e3 - f_low / 1e3) / self.points  # rbw equal to freq step size in kHz
        else:
            rbw = float(self.rbw)

        if rbw < 0.2:  # change this to something more fancy
            rbw = 0.2
        elif rbw > 850:
            rbw = 850

        # timeout can be very long - use a heuristic approach
        timeout = int(((f_high - f_low) / 1e3) / (rbw ** 2) + self.points / 1e3) + 5
        self.timeout = timeout
        logging.debug(f'sweepTimeout = {self.timeout}')

    # return 1D numpy array with power as dBm.  Freqs are in Hz
    def spectrum(self, f_low, f_high):  # runs in a separae thread
        self.threadrunning = True
        while self.sweeping:
            with serial.Serial(self.dev, baudrate=3000000) as serialPort:  # baudrate does nothing for USB cnx
                serialPort.timeout = self.timeout
                logging.debug(f'serial timeout: {self.timeout} s\n')
                logging.debug(f'points: {self.points} s\n')
                scan_command = f'scanraw {int(f_low)} {int(f_high)} {int(self.points)}\r'.encode()
                serialPort.write(scan_command)
                serialPort.read_until(b'{')  # skip command echoes
                raw_data = serialPort.read_until(b'}ch> ')
            raw_data = struct.unpack('<' + 'xH'*self.points, raw_data[:-5])  # ignore trailing '}ch> '
            raw_data = np.array(raw_data, dtype=np.uint16)
            logging.debug(f'raw data: {raw_data} s\n')
            SCALE = 174  # tinySA: 128  tinySA4: 174
            dBm_power = (raw_data / 32) - SCALE  # scale 0..4095 -> -128..-0.03 dBm
            self.signals.result.emit(dBm_power)
        self.threadrunning = False

#    def battery():
        # command = 'vbat\r'.encode()
        # need to get data

    def sigProcess(self, signaldBm):  # signaldBm is emitted from the worker thread
        # store each sweep in an array with most recent at index 0
        self.sweepresults = np.roll(self.sweepresults, 1, axis=0)
        self.sweepresults[0] = signaldBm
        self.procresult = signaldBm

        method = ui.calc_box.currentText()  # future - make this adjustable from the GUI
        if method == 'aver4':
            self.procresult = np.average(self.sweepresults[:4, ::], axis=0)
        if method == 'aver16':
            self.procresult = np.average(self.sweepresults[:16, ::], axis=0)
        if method == 'maxh':
            self.procresult = np.amax(self.sweepresults[:50, ::], axis=0)
        if method == 'minh':
            self.procresult = np.amin(self.sweepresults[:50, ::], axis=0)

        self.updateGUI()
        # self.updateTimeSpectrum()

    def updateGUI(self):
        # self.sweepresults[1:2:, 0::] = self.sweepresults[0:1:, 0::]
        spectrumDisplay.setData((self.frequencies/1e6), self.procresult)

    # def createTimeSpectrum(self):
    #     xarray = np.ndarray((1, self.points), dtype=float)
    #     yarray = np.ndarray((1, self.points), dtype=float)
    #     zarray = np.ndarray((1, self.points), dtype=float)
    #     pts = np.vstack([xarray, zarray, yarray]).transpose()
    #     self.plt = pyqtgl.GLLinePlotItem(pos=pts, color=pyqtgraph.glColor((1, 5)), width=1, antialias=True)
    #     # ui.openGLWidget.setCameraPosition(distance=1)
    #     # self.plt.translate(0, 0, 0)
    #     ui.openGLWidget.addItem(self.plt)

    # def updateTimeSpectrum(self):
    #     xarray = (self.frequencies/1e6)
    #     yarray = np.ndarray((1, self.points), dtype=float)
    #     for i in range(10):
    #         zarray = -1*self.sweepresults[i]
    #         logging.info(f'xarray = {xarray}')
    #         logging.info(f'yarray = {yarray}')
    #         logging.info(f'zarray = {zarray}')
    #         yarray.fill(-i)
    #         pts = np.vstack([xarray, yarray, zarray]).transpose()
    #         self.plt.setData(pos=pts, color=pyqtgraph.glColor((i, 5)), width=1, antialias=True)

    def pause(self):
        # pauses the sweeping in either input or output mode
        pause_command = 'pause\r'.encode()
        self.serialSend(pause_command)

    def resume(self):
        # resumes the sweeping in either input or output mode
        resume_command = 'resume\r'.encode()
        self.serialSend(resume_command)

    def initialise(self):
        command = 'spur auto\r'.encode()
        tinySA.serialSend(command)
        self.spur_auto = True
        command = 'lna off\r'.encode()
        tinySA.serialSend(command)
        self.lna_on = False

# tinySA end ######################################################################


class WorkerSignals(QObject):
    error = pyqtSignal(str)
    result = pyqtSignal(np.ndarray)
    finished = pyqtSignal()


class Worker(QRunnable):
    '''Worker threads so that measurements can run outside GUI event loop'''

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

# Get tinysa device automatically
def getport() -> str:
    try:
        device_list = list_ports.comports()
    except serial.SerialException:
        logging.info('serial exception')
    for x in device_list:
        if x.vid == VID and x.pid == PID:
            return x.device
    raise OSError("TinySA not found")


def scan():
    if tinySA.sweeping:  # if it's running, stop it
        tinySA.sweeping = False  # tells the thread to stop
        ui.scan_button.setEnabled(False)  # prevent repeat presses of 'stop'
        while tinySA.threadrunning:
            time.sleep(0.1)  # wait until the thread stops using serial comms
        ui.scan_button.setEnabled(True)
        activeButtons(True)
        ui.scan_button.setText('Run')  # toggle the 'Stop' button text
        # tinySA.updateTimeSpectrum()
    else:
        tinySA.pause()
        startF = ui.start_freq.value()*1e6
        stopF = ui.stop_freq.value()*1e6
        points = int(ui.points_box.currentText())
        tinySA.set_frequencies(startF, stopF, points)
        tinySA.setRBW()  # fetches rbw value from the GUI combobox and sends it to TinySA
        tinySA.clearBuffer()
        tinySA.sweepTimeout(startF, stopF)
        activeButtons(False)
        ui.scan_button.setText('Stop')  # toggle the 'Run' button text
        app.processEvents()
        tinySA.startMeasurement(startF, stopF)  # runs measurement in separate thread


def rbw_changed():
    tinySA.setRBW()


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
        start = fBandStart[index]
        ui.start_freq.setValue(start)
        start_freq_changed()
        stop = fBandStop[index]
        ui.stop_freq.setValue(stop)
        stop_freq_changed()


# def start_freq_button():
#     if ui.start_freq_button.isChecked:
#         ui.start_freq_button.setText('Centre Freq')
#         ui.stop_freq_button.setText('Freq span')
#     else:
#         ui.start_freq_button.setText('Start Freq')
#         ui.stop_freq_button.setText('Stop Freq')


def attenuate_changed():  # lna and attenuator are switched so mutually exclusive. To do: add code for this
    atten = ui.atten_box.value()
    if atten == 0:
        atten = 'auto'
    command = f'attenuate {str(atten)}\r'.encode()
    tinySA.serialSend(command)


def spur():
    if tinySA.spur_auto:
        command = 'spur off\r'.encode()
        tinySA.spur_auto = False
        ui.spur_button.setText('SPUR off')
    else:
        command = 'spur auto\r'.encode()
        tinySA.spur_auto = True
        ui.spur_button.setText('SPUR auto')
    tinySA.serialSend(command)


def lna():  # lna and attenuator are switched so mutually exclusive. To do: add code for this
    if tinySA.lna_on:
        command = 'lna off\r'.encode()
        tinySA.lna_on = False
        ui.lna_button.setText('LNA off')
    else:
        command = 'lna on\r'.encode()
        tinySA.lna_on = True
        ui.lna_button.setText('LNA on')
    tinySA.serialSend(command)


def exit_handler():
    tinySA.sweeping = False
    time.sleep(1)  # allow time for measurements to stop
    tinySA.resume()
    app.processEvents()
    logging.info('Closed')

# axes = pyqtgl.GLAxisItem()
# axes.setSize(120, -120, 5)
# ui.openGLWidget.addItem(axes)
# tinySA.createTimeSpectrum()


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
    ui.spur_button.setEnabled(tF)
    ui.lna_button.setEnabled(tF)
    ui.rbw_box.setEnabled(tF)
    ui.vbw_box.setEnabled(tF)
    ui.points_box.setEnabled(tF)
    ui.band_box.setEnabled(tF)
    ui.start_freq.setEnabled(tF)
    ui.stop_freq.setEnabled(tF)


###############################################################################
# Instantiate classes

tinySA = analyser(getport())
app = QtWidgets.QApplication([])  # create QApplication for the GUI
window = QtWidgets.QMainWindow()
ui = QtTinySpectrum.Ui_MainWindow()
ui.setupUi(window)

###############################################################################
# GUI settings

# pyqtgraph settings for spectrum display
red = pyqtgraph.mkPen(color='r', width=1.0)
red_dash = pyqtgraph.mkPen(color='r', width=0.5, style=QtCore.Qt.DashLine)
yellow = pyqtgraph.mkPen(color='y', width=1.0)
blue = pyqtgraph.mkPen(color='b', width=0.5,  style=QtCore.Qt.DashLine)
ui.graphWidget.setYRange(-110, 5)
ui.graphWidget.setXRange(88, 100)
ui.graphWidget.setBackground('k')  # black
ui.graphWidget.showGrid(x=True, y=True)
ui.graphWidget.addLine(y=6, movable=False, pen=red, label='', labelOpts={'position':0.05, 'color':('r')})
ui.graphWidget.addLine(y=0, movable=False, pen=red_dash, label='max', labelOpts={'position':0.025, 'color':('r')})
ui.graphWidget.addLine(y=-25, movable=False, pen=blue, label='best', labelOpts={'position':0.025, 'color':('b')})
# ui.graphWidget.addLine(y=-50, movable=False, pen=blue, label='', labelOpts={'position':0.025, 'color':('c')})
ui.graphWidget.setLabel('left', 'Signal', 'dBm')
ui.graphWidget.setLabel('bottom', 'Frequency MHz')

spectrumDisplay = ui.graphWidget.plot([], [], name='Spectrum', pen=yellow, width=1)


# pyqtgraph settings for time spectrum
# axes = pyqtgl.GLAxisItem()
# # axes.setSize(120, -120, 5)
# ui.openGLWidget.addItem(axes)
# tinySA.createTimeSpectrum()


# voltage = tinySA.battery()
# ui.battery.setValue(voltage)


###############################################################################
# Connect signals from buttons and sliders

ui.scan_button.clicked.connect(scan)
ui.rbw_box.currentTextChanged.connect(rbw_changed)
ui.atten_box.valueChanged.connect(attenuate_changed)
ui.start_freq.valueChanged.connect(start_freq_changed)
ui.stop_freq.valueChanged.connect(stop_freq_changed)
ui.spur_button.clicked.connect(spur)
ui.lna_button.clicked.connect(lna)
ui.band_box.currentTextChanged.connect(band_changed)


###############################################################################
# set up the application

ui.rbw_box.addItems(['auto', '0.2', '1', '3', '10', '30', '100', '300', '600', '850'])
ui.vbw_box.addItems(['auto'])
ui.calc_box.addItems(['Off', 'minh', 'maxh', 'aver4', 'aver16'])
ui.points_box.addItems(['25', '50', '100', '200', '290', '450', '900', '1800', '3600', '7200', '15400'])
ui.points_box.setCurrentIndex(5)
ui.band_box.addItems(bands)

tinySA.initialise()

window.show()

###############################################################################
# run the application until the user closes it

try:
    app.exec()
finally:
    exit_handler()  # close cleanly
