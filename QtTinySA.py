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
import queue
from PyQt5 import QtWidgets, QtCore
from PyQt5.QtCore import pyqtSlot, pyqtSignal, QRunnable, QObject, QThreadPool
from PyQt5.QtWidgets import QMessageBox, QFileDialog
import pyqtgraph
import QtTinySpectrum  # the GUI

# tinySA4 #####################################################################
import pylab as pl
import struct
import serial
from serial.tools import list_ports
# tinySA ######################################################################

threadpool = QThreadPool()

# Frequency radio button values
fBand = [14, 50, 70, 144, 432, 1296, 2320, 3400, 5700]

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
        self.signals.result.connect(self.updateGUI)
        self.timeout = 1
        # self.signals.error.connect(spiError)
        # self.fifo = queue.SimpleQueue()

    @property
    def frequencies(self):
        # what does this do?
        return self._frequencies

    def startMeasurement(self, startF, stopF, points):
        self.sweep = Worker(self.spectrum, startF, stopF, points)  # workers are auto-deleted when thread stops
        self.running = True
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
        if points:
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

    def sweepTimeout(self, f_low, f_high, points):  # freqs are in Hz
            if self.rbw == 'auto':
                rbw = (f_high / 1e3 - f_low / 1e3) / points  # rbw equal to freq step size in kHz
            else:
                rbw = float(self.rbw)

            if rbw < 0.2:  # change this to something more fancy
                rbw = 0.2
            elif rbw > 850:
                rbw = 850

            # timeout can be very long - use a heuristic approach
            timeout = int(((f_high - f_low) / 1e3) / (rbw ** 2) + points / 1e3) + 5
            self.timeout = timeout
            logging.debug(f'sweepTimeout = {self.timeout}')

    # return 1D numpy array with power as dBm.  Freqs are in Hz
    def spectrum(self, f_low, f_high, points):  # to be threaded
        while self.running:
            with serial.Serial(self.dev, baudrate=3000000) as serialPort:
                serialPort.timeout = self.timeout
                logging.debug(f'serial timeout: {self.timeout} s\n')
                logging.debug(f'points: {points} s\n')
                scan_command = f'scanraw {int(f_low)} {int(f_high)} {int(points)}\r'.encode()
                serialPort.write(scan_command)
                serialPort.read_until(b'{')  # skip command echoes
                raw_data = serialPort.read_until(b'}ch> ')
            raw_data = struct.unpack('<' + 'xH'*points, raw_data[:-5])  # ignore trailing '}ch> '
            raw_data = np.array(raw_data, dtype=np.uint16)
            logging.debug(f'raw data: {raw_data} s\n')
            SCALE = 174  # tinySA: 128  tinySA4: 174
            dBm_power = (raw_data / 32) - SCALE  # scale 0..4095 -> -128..-0.03 dBm
            self.signals.result.emit(dBm_power)

    def battery():
        command = 'vbat\r'.encode()
        tinySA.write(command)  # needs to use a diffeent one to get data
        #     voltage = tinySA.cmd('vbat')
        #     voltage = float(voltage[:4])/1000
        #     return voltage
        # ui.battery.setValue(voltage)

    def updateGUI(self, signaldBm):
        spectrumDisplay.setData((tinySA.frequencies/1e6), signaldBm)

    def resume(self):
        # resumes the sweeping in either input or output mode
        resume_command = 'resume\r'.encode()
        self.setTinySA(resume_command)

    def pause(self):
        # pauses the sweeping in either input or output mode
        pause_command = 'pause\r'.encode()
        self.setTinySA(pause_command)

    # def temperature(self):
    #     self.send_command("k\r")
    #     data = self.fetch_data()
    #     for line in data.split('\n'):
    #         if line:
    #             return float(line)

# def logmag(self, x):  # plots graph using matplotlib
#     pl.grid(True)
#     pl.xlim(self.frequencies[0], self.frequencies[-1])
#     pl.plot(self.frequencies, x)


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
    tinySA.sweepTimeout(startF, stopF, points)
    activeButtons(False)
    tinySA.startMeasurement(startF, stopF, points)  # runs measurement in separate thread


def scan_stop():
    tinySA.running = False
    activeButtons(True)


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


def calc_changed():
    logging.info('calc does nothing yet')


def spur_checked():  # doesn't work
    command = 'spur off'.encode()
    if ui.spur.isChecked:
        command = 'spur on'.encode()
    tinySA.setTinySA(command)


def centre_checked():  # doesn't work
    if ui.centre.isChecked:
        ui.freq_label_1.setText('Centre')
        ui.freq_label_2.setText('Span')
    else:
        ui.freq_label_1.setText('Start')
        ui.freq_label_2.setText('Stop')


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

# pyqtgraph settings for calibration display
# ui.slopeFreq.addLegend(offset=(20, 10))
# ui.slopeFreq.setXRange(0, 3200, padding=0)
# ui.slopeFreq.showGrid(x=True, y=True)
# ui.slopeFreq.setBackground('k')  # white
# ui.slopeFreq.setLabel('bottom', 'Frequency', '')
# ui.slopeFreq.setLabel('left', 'Transfer Fn Slope (Codes/dB)', '')
# maxLim = ui.slopeFreq.plot(fSpec, maxSlope, name='AD8318 spec limits', pen='y')
# minLim = ui.slopeFreq.plot(fSpec, minSlope, pen='y')

# voltage = tinySA.battery()
# ui.battery.setValue(voltage)



###############################################################################
# Connect signals from buttons and sliders

# ui.runButton.clicked.connect(startMeasurement)
# ui.scan.clicked.connect(lambda: analyser.scan())
ui.scan_start.clicked.connect(scan_start)
ui.scan_stop.clicked.connect(scan_stop)
ui.rbw_box.currentTextChanged.connect(rbw_changed)
ui.atten_box.valueChanged.connect(attenuate_changed)
ui.start_freq.valueChanged.connect(start_freq_changed)
ui.stop_freq.valueChanged.connect(stop_freq_changed)
ui.calc_box.currentTextChanged.connect(calc_changed)
ui.spur.stateChanged.connect(spur_checked)
ui.centre.stateChanged.connect(centre_checked)


###############################################################################
# set up the application

ui.rbw_box.addItems(['auto', '0.2', '1', '3', '10', '30', '100', '300', '600', '850'])
ui.vbw_box.addItems(['not available'])
ui.calc_box.addItems(['off', 'minh', 'maxh', 'maxd', 'aver4', 'aver16', 'quasip'])
ui.points_box.addItems(['25', '50', '100', '200', '290', '450'])
ui.points_box.setCurrentIndex(5)
ui.band_box.addItems(['set freq', '14', '50', '70', '144', '432', '1296', '2320', '3400', '5700'])
# tinySA.set_frequencies(90, 100, 450)
# tinySA.set_sweep(90e6, 100e6)

window.show()

###############################################################################
# run the application until the user closes it

try:
    app.exec()
finally:
    exit_handler()  # close database
