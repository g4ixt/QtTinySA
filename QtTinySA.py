#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Created on Tue 1 May 2023
@author: Ian Jefferson G4IXT
TinySA GUI programme using Qt5.

TinySA class based on Erik's Python.  TinySA is Erik's trademark.  Write more here.

This code makes use of a subset of the code from touchstone.py from scikit-rf, an open-source
Python package for RF and Microwave applications.
"""

import time
import logging
import numpy as np
from PyQt5 import QtWidgets, QtCore
from PyQt5.QtCore import pyqtSlot, pyqtSignal, QRunnable, QObject, QThreadPool
from PyQt5.QtWidgets import QMessageBox, QFileDialog
import pyqtgraph
import QtTinySpectrum  # the GUI

#  For 3D
from pyqtgraph.Qt import QtCore, QtGui
import pyqtgraph.opengl as pyqtgl
#  3D

# tinySA4 #####################################################################
# import pylab as pl
import struct
import serial
from serial.tools import list_ports
# tinySA ######################################################################

threadpool = QThreadPool()

VID = 0x0483  # 1155
PID = 0x5740  # 22336
REF_LEVEL = (1 << 9)

logging.basicConfig(format="%(message)s", level=logging.INFO)


F_LOW = 88e6
F_HIGH = 90e6


###############################################################################
# classes

class analyser:
    def __init__(self, dev=None):
        self.dev = getport()
        self._frequencies = None
        self.points = 101
        self.running = False
        self.signals = WorkerSignals()
        self.signals.result.connect(self.sigProcess)
        self.timeout = 1
        # self.signals.error.connect(spiError)
        # self.fifo = queue.SimpleQueue()

    @property
    def frequencies(self):
        # what does this do?
        return self._frequencies

    def startMeasurement(self, startF, stopF):
        self.sweep = Worker(self.spectrum, startF, stopF)  # workers are auto-deleted when thread stops
        self.running = True
        self.sweepresults = np.zeros((100, self.points), dtype=float)  # to do - add row count to GUI
        threadpool.start(self.sweep)

    def setTinySA(self, command):
        self.clearBuffer()
        with serial.Serial(port=self.dev, baudrate=3000000) as SA:
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
        self.rbw = ui.rbw_box.currentText()  # ui values are discrete ones in kHz
        rbw_command = f'rbw {self.rbw}\r'.encode()
        self.setTinySA(rbw_command)

    def clearBuffer(self):
        with serial.Serial(self.dev, baudrate=3000000) as serialPort:
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
        while self.running:
            with serial.Serial(self.dev, baudrate=3000000) as serialPort:
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

#    def battery():
        # command = 'vbat\r'.encode()
        # tinySA.write(command)  # need to get data

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
        self.updateTimeSpectrum()

    def updateGUI(self):
        # self.sweepresults[1:2:, 0::] = self.sweepresults[0:1:, 0::]
        spectrumDisplay.setData((self.frequencies/1e6), self.procresult)

    def createTimeSpectrum(self):
        xarray = np.ndarray((1, self.points), dtype=float)
        yarray = np.ndarray((1, self.points), dtype=float)
        zarray = np.ndarray((1, self.points), dtype=float)
        pts = np.vstack([xarray, zarray, yarray]).transpose()
        self.plt = pyqtgl.GLLinePlotItem(pos=pts, color=pyqtgraph.glColor((1, 5)), width=1, antialias=True)
        # ui.openGLWidget.setCameraPosition(distance=1)
        # self.plt.translate(0, 0, 0)
        ui.openGLWidget.addItem(self.plt)

    def updateTimeSpectrum(self):
        xarray = (self.frequencies/1e6)
        yarray = np.ndarray((1, self.points), dtype=float)
        for i in range(10):
            zarray = -1*self.sweepresults[i]
            logging.info(f'xarray = {xarray}')
            logging.info(f'yarray = {yarray}')
            logging.info(f'zarray = {zarray}')
            yarray.fill(-i)
            pts = np.vstack([xarray, yarray, zarray]).transpose()
            self.plt.setData(pos=pts, color=pyqtgraph.glColor((i, 5)), width=1, antialias=True)

    def resume(self):
        # resumes the sweeping in either input or output mode
        resume_command = 'resume\r'.encode()
        self.setTinySA(resume_command)

    def pause(self):
        # pauses the sweeping in either input or output mode
        pause_command = 'pause\r'.encode()
        self.setTinySA(pause_command)


# tinySA ######################################################################


class WorkerSignals(QObject):
    error = pyqtSignal(str)
    result = pyqtSignal(np.ndarray)


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


def scan_start():
    tinySA.pause()
    startF = ui.start_freq.value()*1e6
    stopF = ui.stop_freq.value()*1e6
    points = int(ui.points_box.currentText())
    tinySA.set_frequencies(startF, stopF, points)
    tinySA.setRBW()  # fetches rbw value from the GUI combobox and sends it to TinySA
    tinySA.clearBuffer()
    tinySA.sweepTimeout(startF, stopF)
    activeButtons(False)
    tinySA.startMeasurement(startF, stopF)  # runs measurement in separate thread


def scan_stop():
    tinySA.running = False
    activeButtons(True)
    # tinySA.updateTimeSpectrum()


def rbw_changed():
    tinySA.setRBW()


def start_freq_changed():
    start = ui.start_freq.value() * 1e6
    stop = ui.stop_freq.value()
    ui.graphWidget.setXRange(start / 1e6, stop)
    command = f'sweep start {start}\r'.encode()
    tinySA.setTinySA(command)


def stop_freq_changed():
    stop = ui.stop_freq.value() * 1e6
    start = ui.start_freq.value()
    ui.graphWidget.setXRange(start, stop / 1e6)
    command = f'sweep stop {stop}\r'.encode()
    tinySA.setTinySA(command)


def measure_button():
    command = 'vbw 1.0\r'.encode()
    tinySA.setTinySA(command)


def attenuate_changed():
    atten = ui.atten_box.value()
    command = f'attenuate {atten}\r'.encode()
    tinySA.setTinySA(command)


def spur_checked():  # doesn't work
    command = 'spur off'.encode()
    if ui.spur.isChecked:
        command = 'spur on'.encode()
    tinySA.setTinySA(command)


# def start_or_centre():  # doesn't work
#     if ui.start_freq_button.isChecked:
#         ui.start_freq_button.setText('Centre Freq')


def exit_handler():
    # meter.running = False
    # while meter.fifo.qsize() > 0:
    #     time.sleep(0.2)  # allow time for the fifo queue to empty
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
    # prevent User button presses when TinySA is scanning
    ui.atten_box.setEnabled(tF)
    ui.spur.setEnabled(tF)
    ui.rbw_box.setEnabled(tF)
    ui.vbw_box.setEnabled(tF)
    ui.points_box.setEnabled(tF)
    ui.band_box.setEnabled(tF)
    ui.start_freq.setEnabled(tF)
    ui.stop_freq.setEnabled(tF)
    ui.scan_start.setEnabled(tF)


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
ui.graphWidget.setYRange(-110, 5)
ui.graphWidget.setXRange(90, 100)
ui.graphWidget.setBackground('k')  # black
ui.graphWidget.showGrid(x=True, y=True)
ui.graphWidget.addLine(y=6, movable=False, pen=red, label='', labelOpts={'position':0.05, 'color':('r')})
ui.graphWidget.addLine(y=0, movable=False, pen=red_dash, label='max', labelOpts={'position':0.025, 'color':('r')})
# ui.graphWidget.addLine(y=-50, movable=False, pen=blue, label='', labelOpts={'position':0.025, 'color':('c')})
ui.graphWidget.setLabel('left', 'Signal', 'dBm')
ui.graphWidget.setLabel('bottom', 'Frequency', 'MHz')
spectrumDisplay = ui.graphWidget.plot([], [], name='Spectrum', pen=yellow, width=1)

# pyqtgraph settings for time spectrum

# create three grids, add each to the view
# xgrid = gl.GLGridItem()
# xgrid.rotate(90, 0, 1, 0)  # rotate to face the correct direction
# ygrid = gl.GLGridItem()
# ygrid.rotate(90, 1, 0, 0)  # rotate to face the correct direction
# zgrid = gl.GLGridItem()
# ui.openGLWidget.addItem(xgrid)
# ui.openGLWidget.addItem(ygrid)
# ui.openGLWidget.addItem(zgrid)
axes = pyqtgl.GLAxisItem()
# axes.setSize(120, -120, 5)
ui.openGLWidget.addItem(axes)
tinySA.createTimeSpectrum()

# # ?
# xgrid.translate(-10, 0, 0)
# ygrid.translate(0, -10, 0)
# zgrid.translate(0, 0, -10)


# voltage = tinySA.battery()
# ui.battery.setValue(voltage)


###############################################################################
# Connect signals from buttons and sliders

ui.scan_start.clicked.connect(scan_start)
ui.scan_stop.clicked.connect(scan_stop)
ui.rbw_box.currentTextChanged.connect(rbw_changed)
ui.atten_box.valueChanged.connect(attenuate_changed)
ui.start_freq.valueChanged.connect(start_freq_changed)
ui.stop_freq.valueChanged.connect(stop_freq_changed)
ui.spur.stateChanged.connect(spur_checked)


###############################################################################
# set up the application

ui.rbw_box.addItems(['auto', '0.2', '1', '3', '10', '30', '100', '300', '600', '850'])
ui.vbw_box.addItems(['not available'])
ui.calc_box.addItems(['off', 'minh', 'maxh', 'aver4', 'aver16'])
ui.points_box.addItems(['25', '50', '100', '200', '290', '450', '900', '1800', '3600', '7200', '15400'])
ui.points_box.setCurrentIndex(5)
ui.band_box.addItems(['set freq', '14', '50', '70', '144', '432', '1296', '2320', '3400', '5700'])

window.show()

###############################################################################
# run the application until the user closes it

try:
    app.exec()
finally:
    exit_handler()  # close database
