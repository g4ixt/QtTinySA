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

logging.basicConfig(format="%(message)s", level=logging.DEBUG)

###############################################################################
# classes

class analyser:
    def __init__(self, dev=None):
        self.dev = getport()
        self.serial = None
        self._frequencies = None
        self.points = 101

    @property
    def frequencies(self):
        # what does this do?
        return self._frequencies

    def set_frequencies(self, start=1e6, stop=350e6, points=None):
        # creates an array of equi-spaced freqs but doesn't actually set it on the tinySA
        if points:
            self.points = points
        self._frequencies = np.linspace(start, stop, self.points)
        logging.debug(f'frequencies = {self._frequencies}')

    def fetch_frequencies(self):  # fetch list of frequencies from TinySA and store on self in np array
        # dumps the frequencies used by the last sweep
        self.send_command("frequencies\r")
        data = self.fetch_data()
        logging.debug(f'fetch frequencies {data}')
        x = []
        for line in data.split('\n'):
            if line:
                x.append(float(line))
        self._frequencies = np.array(x)

    def openComms(self):  # changed from 'open' because 'open' is a Python Function Name
        if self.serial is None:
            self.serial = serial.Serial(self.dev)  # self.dev comes from getport() in init

    def closeComms(self):  # changed from 'close' because 'close' is a Python Function Name
        if self.serial:
            self.serial.close()
        self.serial = None

    def send_command(self, cmd):  # what is the repationship between this Fn and cmd()?
        self.openComms()
        self.serial.write(cmd.encode())
        self.serial.readline()  # discard empty line
#        print(self.serial.readline()) # discard empty line

    def cmd(self, text):
        self.openComms()
        self.serial.write((text + "\r").encode())
        self.serial.readline()  # discard empty line
        data = self.fetch_data()
        return data
#        self.serial.readline() # discard empty line

    def set_sweep(self, start, stop):
        # Set sweep boundaries or execute a sweep
        # usage: sweep [ ( start|stop|center|span|cw {frequency} ) | ( {start(Hz)} {stop(Hz)} [0..290] ) ]
        # Sweep without arguments lists the current sweep settings, the frequencies specified
        # should be within the permissible range. The sweep commands apply both to input and output modes
        if start is not None:
            self.send_command("sweep start %d\r" % start)
        if stop is not None:
            self.send_command("sweep stop %d\r" % stop)

    def set_span(self, span):
        if span is not None:
            self.send_command("sweep span %d\r" % span)

    def set_center(self, center):
        if center is not None:
            self.send_command("sweep center %d\r" % center)

    def set_level(self, level):
        # sets the output level in dBm.	  Usage: level -76..13. 	Not all values in the range are available
        if level is not None:
            self.send_command("level %d\r" % level)

    def set_output(self, on):
        if on is not None:
            if on:
                self.send_command("output on\r")
            else:
                self.send_command("output off\r")

    def set_frequency(self, freq):
        # pauses the sweep and sets the measurement frequency.  Postfix k, M, G accepted.
        if freq is not None:
            self.send_command("freq %d\r" % freq)

    def measure(self, freq):
        # measures the input level at each of the requested frequencies
        # Usage: hop {start(Hz)} {stop(Hz)} {step(Hz) | points} [outmask]
        # If the third parameter is < 450 it assumes points, else as frequency step
        # Outmask selects the output. 1 = frequency 2 = level
        if freq is not None:
            self.send_command("hop %d 2\r" % freq)
            # hop can also accept start/stop/step parameters.  Frequency (1) or level (2) is output
            data = self.fetch_data()
            for line in data.split('\n'):
                if line:
                    return float(line)

    def temperature(self):
        self.send_command("k\r")
        data = self.fetch_data()
        for line in data.split('\n'):
            if line:
                return float(line)

    def rbw(self, data=0):
        # sets the rbw to either automatic or a specific value
        # usage: rbw auto|3..600.  The number specifies the target rbw in kHz
        if data == 0:
            self.send_command("rbw auto\r")
            return
        if data < 1:
            self.send_command("rbw %f\r" % data)
            return
        if data >= 1:
            self.send_command("rbw %d\r" % data)

    def fetch_data(self):  # a general Fn used to return various types of data depending on calling Fn
        result = ''
        line = ''
        while True:
            c = self.serial.read().decode('utf-8')
            if c == chr(13):
                next  # ignore CR
            line += c
            if c == chr(10):
                result += line
                line = ''
                next
            if line.endswith('ch>'):
                # stop on prompt
                break
        return result

    def resume(self):
        # resumes the sweeping in either input or output mode
        self.send_command("resume\r")

    def pause(self):  # not used anywhere
        # pauses the sweeping in either input or output mode
        self.send_command("pause\r")

    def marker_value(self, nr=1):  # default to marker 1 only?
        # sets or dumps marker info.  Usage: marker {id} on|off|peak|{freq}|{index}
        # where id=1..4 index=0..num_points-1.  Marker levels will use the selected unit.
        # Marker peak will activate the marker (if not done already), position the marker
        # on the strongest 	signal and display the marker info.  The frequency must be within the selected sweep range
        self.send_command("marker %d\r" % nr)
        data = self.fetch_data()
        line = data.split('\n')[0]
#        print(line)
        if line:
            dl = line.strip().split(' ')
            if len(dl) >= 4:
                d = line.strip().split(' ')[3]
                return float(d)
        return 0

    def listSD(self, file=""):  # changed from 'list' because 'list' is a Python Function Name
        self.send_command("sd_list %s\r" % file)
        data = self.fetch_data()
        return (data)

    def readSD(self, file):
        self.send_command("sd_read %s\r" % file)
        f = "<1i"
        b = self.serial.read(4)
        size = struct.unpack(f, b)
        size = size[0]
        print(size)
        data = self.serial.read(size)
        # print (data.size)
        return (data)

    def data(self, array=2):
        # dumps the trace data. 0=temp value, 1=stored trace, 2=measurement
        self.send_command("data %d\r" % array)
        data = self.fetch_data()
        logging.debug(f'data = {data}')
        x = []
        for line in data.split('\n'):  # a list with newline as separator
            if line:
                d = line.strip().split(' ')  # for each item in the list, trim spaces, split at . into a list
                logging.debug(f'data {d}')
                # then d isn't used?
                x.append(float(d))
        logging.debug(f'x = {x}')
        return np.array(x)

    def send_scan(self, start=1e6, stop=900e6, points=None):
        # performs a scan and optionally outputs the measured data.
        # Usage: scan {start(Hz)} {stop(Hz)} [points] [outmask]
        # where the outmask is a binary OR of 1=frequencies, 2=measured data, 4=stored data and points
        if points:
            self.send_command("scan %d %d %d\r" % (start, stop, points))
        else:
            self.send_command("scan %d %d\r" % (start, stop))

    def scan(self):  # scans in segments if needed, using send_scan Fn
        segment_length = 101
        array0 = []  # not used
        array1 = []
        if self._frequencies is None:  # from when class is instantiated or from set_frequencies()
            self.fetch_frequencies()  # fetch and stores them
        freqs = self._frequencies
        logging.debug(f'frequencies in scan Fn = {self._frequencies}')
        while len(freqs) > 0:
            seg_start = freqs[0]
            # chop the scan into segments
            seg_stop = freqs[segment_length-1] if len(freqs) >= segment_length else freqs[-1]
            length = segment_length if len(freqs) >= segment_length else len(freqs)
            # print((seg_start, seg_stop, length))
            # and set the scan parameters
            self.send_scan(seg_start, seg_stop, length)
            # and use data Fn to get the results from TinySA
            array1.extend(self.data(1))
            freqs = freqs[segment_length:]
        self.resume()  # re-start TinySA normal scanning
        return (array0, array1)  # array0 is still empty

    def scanRaw(self):
        # performas a scan of unlimited amount of points and send data in binary form
        # Usage: scanraw {start(Hz)} {stop(Hz)} [points]
        # Measured data is level in dBm and is sent as '{'('x' MSB LSB)*points '}
        # To get the dBm level from the 16-bit data, divide by 32 and subtract 128
        x = 0

    def logmag(self, x):  # plots graph using matplotlib
        pl.grid(True)
        pl.xlim(self.frequencies[0], self.frequencies[-1])
        pl.plot(self.frequencies, x)

    def battery(self):
        voltage = tinySA.cmd('vbat')
        voltage = float(voltage[:4])/1000
        return voltage


# tinySA ######################################################################


class WorkerSignals(QObject):
    error = pyqtSignal(str)
    result = pyqtSignal(np.ndarray, float, float)


class Worker(QRunnable):
    '''Worker threads so that measurements can run outside GUI event loop'''

    def __init__(self, fn):
        super(Worker, self).__init__()
        self.fn = fn
        self.signals = WorkerSignals()

    @pyqtSlot()
    def run(self):
        '''Initialise the runner'''

        logging.info(f'{self.fn.__name__} thread running')
        self.fn()


###############################################################################
# other methods
#             nv.fetch_frequencies()
#             s = nv.data(p)
# #            nv.fetch_frequencies()
#     if opt.save:
#         nv.writeCSV(s, opt.save)
#     if opt.plot:
#         nv.logmag(s)
        pl.show()


# Get tinysa device automatically
def getport() -> str:
    device_list = list_ports.comports()
    for x in device_list:
        if x.vid == VID and x.pid == PID:
            return x.device
    raise OSError("device not found")

def scan_button():
    startF = ui.start_freq.value()*1e6
    stopF = ui.stop_freq.value()*1e6
    tinySA.set_sweep(startF, stopF)
    tinySA.set_frequencies(startF, stopF, 450)
    tinySA.send_command('pause\r')
    tinySA.send_command("scan %d %d %d %d\r" % (startF, stopF, 450, 2))
    s = tinySA.data(1)
    logging.debug(f'scan = {s}')

def rbw_changed():
    i = 0

def measure_button():
    i = 0


def plot_button():
    tinySA.fetch_frequencies()
    s = tinySA.data(0)
#    logging.debug(f's = {s}')
#    tinySA.logmag(s)
#    pl.show()
    logging.debug(f's = {s}')
    logging.debug(f'freqs = {tinySA.frequencies/1e6}')
    spectrum.setData((tinySA.frequencies/1e6), s)
    app.processEvents()

# ##   nv.set_port(opt.port)
#     if opt.start or opt.stop or opt.points:
#         nv.set_frequencies(opt.start, opt.stop, opt.points)
# ##    plot = opt.plot
#     if opt.plot or opt.save or opt.scan:
#         p = int(opt.port) if opt.port else 0
#         if opt.scan or opt.points > 101:
#             s = nv.scan()
#             s = s[p]
#         else:
#             if opt.start or opt.stop:
#                 nv.set_sweep(opt.start, opt.stop)
#             nv.fetch_frequencies()
#             s = nv.data(p)
# ##            nv.fetch_frequencies()
    # if opt.save:
    #     nv.writeCSV(s, opt.save)
    # if opt.plot:
    #     nv.logmag(s)
    #     pl.show()

def exit_handler():
    # meter.running = False
    # while meter.fifo.qsize() > 0:
    #     time.sleep(0.2)  # allow time for the fifo queue to empty
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
    # prevent User button presses that may affect readings when meter is running
    ui.saveDevice.setEnabled(tF)
    ui.loadS2P.setEnabled(tF)
    ui.saveValues.setEnabled(tF)
#    ui.measHigh.setEnabled(tF)
#    ui.measLow.setEnabled(tF)
    ui.addDevice.setEnabled(tF)
    ui.deleteDevice.setEnabled(tF)
    ui.addFreq.setEnabled(tF)
    ui.delFreq.setEnabled(tF)
    ui.delAllFreq.setEnabled(tF)
#    ui.calibrate.setEnabled(tF)
#    ui.saveCal.setEnabled(tF)
#    ui.addCal.setEnabled(tF)
#    ui.deleteCal.setEnabled(tF)
    ui.inUse.setEnabled(tF)


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
ui.graphWidget.setYRange(-100, 5)
ui.graphWidget.setXRange(90, 100)
ui.graphWidget.setBackground('k')  # black
ui.graphWidget.showGrid(x=True, y=True)
ui.graphWidget.addLine(y=6, movable=False, pen=red, label='max', labelOpts={'position':0.05, 'color':('r')})
ui.graphWidget.addLine(y=0, movable=False, pen=red_dash, label='', labelOpts={'position':0.025, 'color':('c')})
# ui.graphWidget.addLine(y=-50, movable=False, pen=blue, label='', labelOpts={'position':0.025, 'color':('c')})
ui.graphWidget.setLabel('left', 'Signal', 'dBm')
ui.graphWidget.setLabel('bottom', 'Frequency', 'MHz')
spectrum = ui.graphWidget.plot([], [], name='Spectrum', pen=yellow, width=1)

# pyqtgraph settings for calibration display
# ui.slopeFreq.addLegend(offset=(20, 10))
# ui.slopeFreq.setXRange(0, 3200, padding=0)
# ui.slopeFreq.showGrid(x=True, y=True)
# ui.slopeFreq.setBackground('k')  # white
# ui.slopeFreq.setLabel('bottom', 'Frequency', '')
# ui.slopeFreq.setLabel('left', 'Transfer Fn Slope (Codes/dB)', '')
# maxLim = ui.slopeFreq.plot(fSpec, maxSlope, name='AD8318 spec limits', pen='y')
# minLim = ui.slopeFreq.plot(fSpec, minSlope, pen='y')

voltage = tinySA.battery()
ui.battery.setValue(voltage)



###############################################################################
# Connect signals from buttons and sliders

# ui.runButton.clicked.connect(startMeasurement)
# ui.scan.clicked.connect(lambda: analyser.scan())
ui.scan_button.clicked.connect(scan_button)
# ui.rbw_box.valueChanged.connect(rbw_changed)

# test
ui.measure.clicked.connect(measure_button)
ui.plot.clicked.connect(plot_button)
#


###############################################################################
# set up the application

ui.rbw_box.addItems(['auto', '0.2', '1', '3', '10', '30', '100', '300', '600', '850'])
ui.vbw_box.addItems(['auto', '0.01', '0.03', '0.1', '0.33', '1.0'])
ui.accuracy_box.addItems(['normal', 'precise', 'fast', 'noise source', 'speedup'])
ui.points_box.addItems(['25', '50', '100', '200', '290', '450'])
ui.points_box.setCurrentIndex(5)
ui.band_box.addItems(['set freq', '14', '50', '70', '144', '432', '1296', '2320', '3400', '5700'])
# tinySA.set_frequencies(90, 100, 450)
tinySA.set_sweep(90e6, 100e6)

window.show()

###############################################################################
# run the application until the user closes it

try:
    app.exec()
finally:
    exit_handler()  # close database
