#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright 2026 Ian Jefferson G4IXT
# SPDX-License-Identifier: GPL-3.0-or-later

# nuitka-project: --enable-plugin=pyside6
# nuitka-project: --include-qt-plugins=sqldrivers,designer,qml
# nuitka-project: --include-data-file=QtTSAprefs.db=./
# nuitka-project: --include-data-file=*baseline.txt=./
# nuitka-project: --include-data-file=*.ui=./
# nuitka-project: --remove-output

"""TinySA GUI programme using Qt, PySide6 and PyQtGraph.

This code provides some of the TinySA Ultra on-screen commands and PC control.
Development is now on Kubuntu 25.10 with Python 3.13 and PySide6 using Spyder.
TinySA, TinySA Ultra and the tinysa icon are trademarks of Erik Kaashoek and are used with permission.
TinySA commands are based on Erik's Python examples: http://athome.kaashoek.com/tinySA/python/
Serial communication commands are based on Martin's Python NanoVNA/TinySA Toolset: https://github.com/Ho-Ro"""

import os
import sys
import time
import logging

from platform import system
from PySide6 import QtCore
from PySide6 import QtWidgets
from PySide6.QtUiTools import QUiLoader
from PySide6.QtCore import QFile, Slot, QSignalBlocker
from PySide6.QtWidgets import QMessageBox, QDataWidgetMapper, QFileDialog, QTableWidgetItem
from PySide6.QtSql import QSqlDatabase, QSqlRelation, QSqlRelationalTableModel, QSqlRelationalDelegate, QSqlQuery
from PySide6.QtGui import QPixmap, QIcon

import queue
import shutil
import platformdirs
import csv
import numpy as np
import pyqtgraph

# from datetime import datetime

from io import BytesIO

from modules.exporters import WWBExporter, WSMExporter
from modules.graphs import SurfaceGraph, PhaseNoiseGraph, Spectrum

from modules.devices import USBdevice, Worker

# Defaults to non local configuration/data dirs - needed for packaging
if system() == "Linux":
    os.environ['XDG_CONFIG_DIRS'] = '/etc:/usr/local/etc'
    os.environ['XDG_DATA_DIRS'] = '/usr/share:/usr/local/share'

# force Qt to use OpenGL rather than DirectX for Windows OS
# QtCore.QCoreApplication.setAttribute(QtCore.Qt.ApplicationAttribute.AA_UseDesktopOpenGL)

logging.basicConfig(format="%(message)s", level=logging.INFO)
threadpool = QtCore.QThreadPool()
basedir = os.path.dirname(__file__)

# pyqtgraph custom exporters
WWBExporter.register()
WSMExporter.register()

# classes ##############################################################################


class CustomTableModel(QSqlRelationalTableModel):
    def __init__(self, parent=None, db=None, ro_columns=tuple()):
        super().__init__(parent, db)
        self.read_only = ro_columns

    def flags(self, index):
        if index.column() in self.read_only:
            return QtCore.Qt.ItemFlag.ItemIsEnabled | QtCore.Qt.ItemFlag.ItemIsSelectable
        else:
            return super().flags(index)


class CustomLoader(QUiLoader):
    def createWidget(self, className, parent=None, name=""):
        logging.debug(f'className = {className}')
        if className == "PlotWidget":
            return pyqtgraph.PlotWidget(parent=parent)
        if className == "GraphicsView":
            return pyqtgraph.GraphicsView(parent=parent)
        return super().createWidget(className, parent, name)


class CustomDialogue(QtWidgets.QDialog):
    def __init__(self, ui_name):
        super().__init__()
        ui_file = QFile(ui_name)
        ui_file.open(QFile.ReadOnly)
        loader = CustomLoader()
        self.ui = loader.load(ui_file)
        self.ui.setWindowIcon(QIcon(os.path.join(basedir, 'tinySAsmall.png')))


class Analyser:
    '''sets up and controls the GUI, and starts and stops measurements'''

    def __init__(self):
        self.directory = None
        self.fifo = queue.SimpleQueue()
        self.maxF = 6000
        self.memF = BytesIO()
        self.readings = np.ndarray(2)
        self.mkr_update_timer = QtCore.QTimer()
        self.dev_ref = []

    def setGraphs(self):
        self.phaseNoise = PhaseNoiseGraph(phasenoise.ui.plotWidget, np.ndarray, np.ndarray, 1)
        self.timespectrum = SurfaceGraph(QtTSA.plot_3D, np.ndarray, np.ndarray)
        self.timespectrum.zoom(QtTSA.zoom.value())
        self.timespectrum.rotateX(QtTSA.x_rotation.value())
        self.timespectrum.rotateY(QtTSA.y_rotation.value())
        # instantiate each spectrum, which has four elements: 1 trace; 4 markers; 1 waterfall; 1 monitor
        self.s0 = Spectrum(QtTSA.graphWidget, QtTSA.waterfall, QtTSA.histogram, multiplot)
        self.s1 = Spectrum(QtTSA.graphWidget, QtTSA.waterfall, QtTSA.histogram, multiplot)
        self.s2 = Spectrum(QtTSA.graphWidget, QtTSA.waterfall, QtTSA.histogram, multiplot)
        self.s3 = Spectrum(QtTSA.graphWidget, QtTSA.waterfall, QtTSA.histogram, multiplot)
        self.spectra = (self.s0, self.s1, self.s2, self.s3)

    def setSignals(self):
        usbInstr.signals.result.connect(self.router)
        usbInstr.signals.saveResults.connect(saveFile)
        usbInstr.signals.error.connect(popUp)
        usbInstr.stopped.connect(self.allStopped)
        usbInstr.update_info.connect(self.set_device_info)
        self.mkr_update_timer.timeout.connect(self.updateMarker)

    @Slot()
    def router(self, usbPort, freq, levl, maxl, minl, sweep_end):
        '''take measurement data, ID it by usbPort & route data to update the right spectrum graph(s)'''
        # tuple 1 = (device, number of devices) tuple 2 = (spectrum to update with tuple 1 device's data)
        # 1 tinySA updates all 4 spectra. 2 tinySAs: first updates 1&2, second 3&4; so on as set by routes
        routes = {(0, 1): (self.s0, self.s1, self.s2, self.s3),
                  (0, 2): (self.s0, self.s1),
                  (0, 3): (self.s0),
                  (0, 4): (self.s0),
                  (1, 2): (self.s2, self.s3),
                  (1, 3): (self.s1),
                  (1, 4): (self.s1),
                  (2, 3): (self.s2),
                  (2, 4): (self.s2),
                  (3, 4): (self.s3)}
        dev_count = len(usbInstr.ports)
        for i in range(dev_count):
            if usbInstr.ports[i].device == usbPort:  # find the device number from its USB port
                dev_num = i
        route = routes.get((dev_num, dev_count))  # route is the list of spectrum instances
        try:
            for spectrum in route:  # update each spectrum with the data from the right device
                self.updateGUI(spectrum, freq, levl, maxl, minl, sweep_end)
        except TypeError:
            logging.info('failed to route measurement data to spectrum trace')
            usbInstr.stop(restart=False)

    def setGUI(self):
        # connect GUI controls that don't interfere with restoration of data at startup
        connectPassive()
        self.setStartFreq()
        # set various defaults
        setPreferences()
        bandselect.filterType(False, QtTSA.filterBox.currentText())  # setting the filter overwrites the band
        # connect GUI controls that would interfere with restoration of data at startup ## modify for devices ##
        connectActive()
        QtTSA.waterfall_size.valueChanged.emit(QtTSA.waterfall_size.value())
        self.s0.enable(QtTSA.trace1.isChecked())
        self.s1.enable(QtTSA.trace2.isChecked())
        self.s2.enable(QtTSA.trace3.isChecked())
        self.s3.enable(QtTSA.trace4.isChecked())
        for mkr_num in range(4):
            self.setMarker(mkr_num)

    def set_device_info(self):
        # show device information in GUI
        gui_field = {0: QtTSA.dev0, 1: QtTSA.dev1, 2: QtTSA.dev2, 3: QtTSA.dev3}
        if usbInstr.dev_list:
            for i, device in enumerate(usbInstr.dev_list):
                gui_field.get(i).setText('')
                if device is not None:
                    port = usbInstr.dev_list[i].usbPort[-4:] + " "
                    name = usbInstr.dev_list[i].firmware[0] + " "
                    fwvers = usbInstr.dev_list[i].firmware[1] + "." + usbInstr.dev_list[i].firmware[2] + " "
                    vers = usbInstr.dev_list[i].firmware[3]
                    battery = usbInstr.dev_list[i].volts
                    desc = port + name + battery
                    gui_field.get(i).setText(desc)
                    gui_field.get(i).setToolTip(name + fwvers + vers)
        else:
            for i in range(4):
                gui_field.get(i).setText('')
                gui_field.get(i).setToolTip('')

    def setting_change(self):
        if usbInstr.is_scanning:
            usbInstr.stop(True)

    def set_box_colour(self, pen, box):
        boxes = [QtTSA.trace1, QtTSA.trace2, QtTSA.trace3, QtTSA.trace4]
        tint = str("background-color: '" + pen + "';")
        boxes[box].setStyleSheet(tint)

    def split_scan(self, startF, stopF, points, split):
        # splits the scan startF/stopF/points across multiple devices by setting the spectrum start/stop variables
        if not split or usbInstr.count == 1:
            for spectrum in self.spectra:
                spectrum.startF = startF
                spectrum.stopF = stopF
                spectrum.points = points  # set points per spectrum = future potential for different vals
            return
        points = int(points/usbInstr.count)
        span = int((stopF - startF)/usbInstr.count)
        starts = {1: (startF, startF, startF, startF),
                  2: (startF, startF+span, startF, startF+span),
                  3: (startF, startF+span, startF+2*span, 0),  # what happens to the zero?
                  4: (startF, startF+span, startF+2*span, startF+3*span)}
        for indx, spectrum in enumerate(self.spectra):
            spectrum.startF = starts.get(usbInstr.count)[indx]
            spectrum.stopF = starts.get(usbInstr.count)[indx] + span
            spectrum.points = points

    def join_wf(self):
        # join the waterfall data arrays depending on the number of devices (usbInstr.count)
        if usbInstr.count == 1:
            wfall_data = self.s0.wfall_data
        else:
            join = {2: (self.s0.wfall_data, self.s2.wfall_data),
                    3: (self.s0.wfall_data, self.s1.wfall_data, self.s2.wfall_data),
                    4: (self.s0.wfall_data, self.s1.wfall_data,
                        self.s2.wfall_data, self.s3.wfall_data)}
            if QtTSA.split_scan.isChecked():
                # waterfall display is join of trace frequencies
                wfall_data = np.concatenate(join.get(usbInstr.count), axis=1)
            else:
                # waterfall data is a stack of traces
                wfall_data = np.concatenate(join.get(usbInstr.count), axis=0)
        QtTSA.waterfall.setXRange(0, np.size(wfall_data, axis=1))
        return wfall_data

    def scan(self):  # called by scan button (run/stop)
        # collect all the settings from the GUI and call usbDevice.start, which interfaces to the hardware
        if usbInstr.is_scanning:
            usbInstr.stop(restart=False)
            return
        self.mkr_update_timer.stop()  # stop updating markers on timer as updateGUI does it when scanning

        # set sweep and device-specific control values
        points = self.setPoints()
        time_points = settings.ui.timePoints.value()
        startF = QtTSA.start_freq.value() * 1e6  # freq in Hz
        stopF = QtTSA.stop_freq.value() * 1e6
        depth = QtTSA.memBox.value()
        split = QtTSA.split_scan.isChecked()
        rbw = self.setRBW()
        attn = self.attn()
        lna = self.lna()
        spur = self.spur()
        if not usbInstr.dev_list:
            popUp(QtTSA, 'TinySA not found', 'Ok', 'Critical')
            return
        usbInstr.controls(rbw, attn, lna, spur)

        # For LNB / transverter mode, modify startF and stopF to suit LO freq
        if bandstype.freq != 0:
            startF, stopF = self.freqOffset(startF, stopF)

        self.split_scan(startF, stopF, points, split)

        for indx, spectrum in enumerate(self.spectra):
            # set each spectrum (trace & marker) and box colours
            spectrum.count = 0
            pen = tracecolours.tm.record(indx).value('colour')
            spectrum.set_colour(pen)
            self.set_box_colour(pen, indx)
            # set the data arrays for the monitor and waterfall
            spectrum.monitor_data = np.full((int(time_points), 2), None, dtype=float)
            spectrum.wfall_data = np.full((depth, spectrum.points), None, dtype=float)

        # start device(s) scanning
        usbInstr.start(self.spectra, rbw, depth, split, loop=True)
        self.runButton('Stop')

    def lna(self):
        if QtTSA.lna_box.isChecked():
            QtTSA.atten_box.setValue(0)
            QtTSA.atten_auto.setEnabled(False)  # attenuator and lna are switched so mutually exclusive
            QtTSA.atten_auto.setChecked(False)
            QtTSA.atten_box.setEnabled(False)
            return True
        else:
            QtTSA.atten_auto.setEnabled(True)
            QtTSA.atten_auto.setChecked(True)
            return False

    def attn(self):
        if QtTSA.lna_box.isChecked():  # attenuator and lna are mutually exclusive
            return "0"
        attenuation = QtTSA.atten_box.value()
        if QtTSA.atten_auto.isChecked():
            QtTSA.atten_box.setEnabled(False)
            return "auto"
        else:
            QtTSA.atten_box.setEnabled(True)
            return attenuation

    def spur(self):
        sType = QtTSA.spur_box.currentText()
        return sType

    def setCentreFreq(self):
        startF = QtTSA.centre_freq.value()-QtTSA.span_freq.value()/2
        stopF = QtTSA.centre_freq.value()+QtTSA.span_freq.value()/2
        with QSignalBlocker(QtTSA.start_freq):
            QtTSA.start_freq.setValue(startF)
        with QSignalBlocker(QtTSA.stop_freq):
            QtTSA.stop_freq.setValue(stopF)
        self.setGraphFreq(startF, stopF)
        self.setting_change()

    def setStartFreq(self):
        startF = QtTSA.start_freq.value()  # freq in MHz
        stopF = QtTSA.stop_freq.value()
        if startF > stopF:
            stopF = startF
            with QSignalBlocker(QtTSA.stop_freq):
                QtTSA.stop_freq.setValue(stopF)
        with QSignalBlocker(QtTSA.centre_freq):
            QtTSA.centre_freq.setValue(startF + (stopF - startF) / 2)
        with QSignalBlocker(QtTSA.span_freq):
            QtTSA.span_freq.setValue(stopF - startF)
        self.setGraphFreq(startF, stopF)
        self.setting_change()

    def setGraphFreq(self, startF, stopF):
        QtTSA.graphWidget.setXRange(startF * 1e6, stopF * 1e6)
        if QtTSA.span_freq.value() != 0:
            lowF.line.setValue((startF + QtTSA.span_freq.value()/20) * 1e6)
            highF.line.setValue((stopF - QtTSA.span_freq.value()/20) * 1e6)

    def freqOffset(self, startF, stopF):  # for mixers or LNBs external to TinySA.  Returns a tuple (startF, stopF)
        spanF = stopF - startF
        loF = bandstype.freq
        if bandstype.freq > startF:  # LNB LO is higher in freq than wanted signal
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

    def rbwMask(self, startF, stopF):
        '''calculate a frequency width factor, used to mask readings near each maximum or minimum'''
        if QtTSA.rbw_auto.isChecked():
            # auto rbw is ~7 kHz per 1 MHz scan frequency span
            approx_rbw = 7 * (stopF - startF) / 1e6  # kHz
            # find the nearest lower discrete rbw value
            for i in range(0, rbwtext.tm.rowCount() - 1):
                rbw = float(rbwtext.tm.record(i).value('value'))  # kHz
                if approx_rbw <= float(rbwtext.tm.record(i).value('value')):
                    break
            maskFreq = settings.ui.rbw_x.value() * rbw * 1e3  # Hz
        else:
            # manual rbw setting
            maskFreq = settings.ui.rbw_x.value() * float(QtTSA.rbw_box.currentText()) * 1e3  # Hz
            logging.debug(f'manual rbw masking factor = {maskFreq/1e3}kHz')
        return maskFreq

    def rbwChanged(self):
        if QtTSA.rbw_auto.isChecked():  # can't calculate Points because we don't know what the RBW will be
            QtTSA.rbw_box.setEnabled(False)
            QtTSA.points_auto.setChecked(False)
            QtTSA.points_auto.setEnabled(False)
        else:
            QtTSA.rbw_box.setEnabled(True)
            QtTSA.points_auto.setEnabled(True)
        self.setting_change()
        self.setRBW()

    def setRBW(self):
        if QtTSA.rbw_auto.isChecked():
            rbw = 'auto'
        else:
            rbw = float(QtTSA.rbw_box.currentText())  # ui values are discrete ones in kHz
        return rbw

    def setPoints(self):
        if QtTSA.points_auto.isChecked():
            rbw = float(QtTSA.rbw_box.currentText())
            points = settings.ui.rbw_x.value() * int((QtTSA.span_freq.value()*1000)/(rbw))  # RBW multiplier * freq kHz
            points = np.clip(points, settings.ui.minPoints.value(), settings.ui.maxPoints.value())  # limit points
        else:
            points = QtTSA.points_box.value()
            logging.debug(f'setPoints: points = {QtTSA.points_box.value()}')
        return points

    @Slot()
    def allStopped(self, restart):
        self.runButton('Run')
        self.mkr_update_timer.start(100)
        if restart:
            self.scan()

    # called by router()
    def updateGUI(self, spectrum, freq, levl, maxl, minl, sweep_end):
        # reverse the arrays if in LNB/Mixer mode when LO is above measured freq
        if bandstype.freq > QtTSA.start_freq.value():
            freq = freq[::-1]
            levl = levl[::-1]
            maxl = maxl[::-1]
            minl = minl[::-1]
            logging.info(f'startF = {freq[0]} stopF = {freq[-1]}')
            QtTSA.waterfall.invertX(True)
        else:
            QtTSA.waterfall.invertX(False)

        # check for zero span and update the spectrum graph x-axis if so
        if freq[0] == freq[-1]:
            QtTSA.graphWidget.setLabel('bottom', 'Time')
            freq = np.arange(1, len(freq) + 1, dtype=int)
            QtTSA.graphWidget.setXRange(freq[0], freq[-1])
        else:
            QtTSA.graphWidget.setLabel('bottom', units='Hz')

        # calculate the average value for the trace, using its waterfall data array
        if spectrum.count <= 1:
            avg = levl
        else:
            if spectrum.count < QtTSA.avgBox.value():
                avg = np.nanmean(spectrum.wfall_data[0:spectrum.count], axis=0)
            else:
                avg = np.nanmean(spectrum.wfall_data[0:QtTSA.avgBox.value()], axis=0)

        # update the main spectrum trace
        gui_boxes = {0: QtTSA.t1_type, 1: QtTSA.t2_type, 2: QtTSA.t3_type, 3: QtTSA.t4_type}
        trace_type = gui_boxes.get(self.spectra.index(spectrum)).currentText()
        data = {'Normal': levl, 'Average': avg, 'Max': maxl, 'Min': minl, 'Freeze': levl}
        if trace_type != 'Freeze':
            spectrum.updateTrace(freq, data.get(trace_type))
        logging.debug(f'updating {self.spectra.index(spectrum)}')

        # update the markers
        maskFreq = self.rbwMask(freq[0], freq[-1])
        limits = (maskFreq, highF.line.value(), lowF.line.value(), threshold.line.value())
        for marker in spectrum.mkr_list:
            marker.mkr_update(limits, spectrum.trace.is_visible)

        # update the scan count, monitor graph and phase noise graph once per sweep
        if sweep_end:
            spectrum.count += 1
            timeNow = time.time()
            spectrum.wfall_data = np.roll(spectrum.wfall_data, 1, axis=0)
            spectrum.update_monitor(freq, timeNow)
            if phasenoise.ui.isVisible() and not QtTSA.rbw_auto.isChecked():
                lineIndex = np.argmin(np.abs(freq - (self.s0.trace.m0.line.value())))  # find index of marker 1
                self.phaseNoise.update(lineIndex, freq, levl, float(QtTSA.rbw_box.currentText()))

        # update the waterfall
        wf_height = QtTSA.waterfall_size.value()
        wf_auto = QtTSA.waterfall_auto.isChecked()
        if np.size(spectrum.wfall_data, axis=1) == np.size(levl):
            spectrum.wfall_data[0] = levl  # wf data array is used for averages so must always be updated
            data = self.join_wf()
            if wf_height > 0:
                spectrum.waterfall.setImage(data, autoLevels=wf_auto)

        # update 3D graph if visible
        if QtTSA.stackedWidget.currentWidget() == QtTSA.View3D:
            self.timespectrum.surface.setHorizontalAspectRatio(1)  # keep the xz surface square
            if data is not None:
                self.timespectrum.updater.updateTimeSpectrum(freq, data)
                self.timespectrum.setRange()

        # other updates
        if QtTSA.points_auto.isChecked():
            with QSignalBlocker(QtTSA.points_box):
                QtTSA.points_box.setValue(np.size(freq))

    def runButton(self, action):
        QtTSA.scan_button.setText(action)
        QtTSA.run3D.setText(action)
        QtTSA.scan_button.setEnabled(True)
        QtTSA.run3D.setEnabled(True)

    def set_dev_combo(self, ui_name, dev_name):
        ui_name.port.clear()
        self.dev_ref = []
        for device in usbInstr.dev_list:
            if device and device.firmware[0] == dev_name:
                if device.sweeping:
                    popUp(QtTSA, "Cannot browse whilst a scan is running", 'Ok', 'Info')
                else:
                    with QSignalBlocker(ui_name.port):
                        ui_name.port.addItem(device.usbPort)
                        self.dev_ref.append(device)  # keep a reference to the device for file ops
        device = self.dev_ref[ui_name.port.currentIndex()]
        ui_name.dev_type.setText(device.firmware[0])

    def file_browser(self):
        self.set_dev_combo(filebrowse.ui, 'tinySA4')
        filebrowse.ui.show()
        self.list_files()

    def list_files(self):
        device = self.dev_ref[filebrowse.ui.port.currentIndex()]
        filebrowse.ui.dev_type.setText(device.firmware[0])
        SD = device.listSD()
        filebrowse.ui.listWidget.clear()
        ls = []
        for i in range(len(SD.splitlines())):
            ls.append(SD.splitlines()[i].split(" ")[0])
        filebrowse.ui.listWidget.insertItems(0, ls)

    def save_file(self, saveSingle=True):
        device = self.dev_ref[filebrowse.ui.port.currentIndex()]
        filebrowse.ui.saveProgress.setValue(0)
        SD = device.listSD()
        for i in range(len(SD.splitlines())):
            if not self.directory:  # have not already saved a file, or ask for folder was checked
                self.directory = QFileDialog.getExistingDirectory(caption="Select folder to save SD card file")
            if not self.directory:
                break
            if saveSingle:
                fileName = filebrowse.ui.listWidget.currentItem().text()  # the file selected in the list widget
            else:
                fileName = SD.splitlines()[i].split(" ")[0]
            with open(os.path.join(self.directory, fileName), "wb") as file:
                data = device.readSD(fileName)
                file.write(data)
            filebrowse.ui.saveProgress.setValue(int(100 * (i+1)/len(SD.splitlines())))
            filebrowse.ui.downloadInfo.setText(self.directory)  # show the path where the file was saved
            if filebrowse.ui.askForPath.isChecked():
                self.directory = None
            if saveSingle:
                filebrowse.ui.saveProgress.setValue(100)
                break

    def show_file(self):
        device = self.dev_ref[filebrowse.ui.port.currentIndex()]
        self.memF.seek(0, 0)  # set the memory buffer pointer to the start
        self.memF.truncate()  # clear down the memory buffer to the pointer
        filebrowse.ui.picture.clear()
        fileName = filebrowse.ui.listWidget.currentItem().text()
        device.clearBuffer()  # clear the tinySA serial buffer
        self.memF.write(device.readSD(fileName))  # read the file from the tinySA memory card and store in memory buffer
        if fileName[-3:] == 'bmp':
            pixmap = QPixmap()
            pixmap.loadFromData(self.memF.getvalue())
            filebrowse.ui.picture.setPixmap(pixmap)

    def correction_window(self):
        self.set_dev_combo(offset.ui, 'tinySA4')
        offset.ui.progress.setValue(0)
        offset.ui.show()

    # def sweepTime(self, seconds):
    #     #  0.003 to 60S
    #     command = f'sweeptime {seconds}\r'
    #     self.fifo.put(command)

    def sweep_as_zoomed(self):
        # find the current limits of the (frequency axis) viewbox and set the sweep to them
        xaxis = (QtTSA.graphWidget.getAxis('bottom').range)
        startF = float(xaxis[0]/1e6)
        stopF = float(xaxis[1]/1e6)
        logging.debug(f'mouseScaled: start = {startF} stop = {stopF}')
        QtTSA.start_freq.setValue(startF)
        QtTSA.stop_freq.setValue(stopF)
        self.freq_changed(False)

    def markerToStart(self):
        startF = QtTSA.start_freq.value()
        for spectrum in self.spectra:
            for mkr in spectrum.mkr_list:
                mkr.to_start(startF)

    def markerSpread(self):
        startF = QtTSA.start_freq.value()
        stopF = QtTSA.stop_freq.value()
        for spectrum in self.spectra:
            for index, mkr in enumerate(spectrum.mkr_list):
                mkr.spread(startF, stopF, index + 1)

    def centreTone(self):  # for phase noise graph
        centreF = self.s0.trace.m0.line.value() * 1e-6
        QtTSA.centre_freq.setValue(centreF)

    def updateMarker(self):  # called by timer when not scanning
        startF = QtTSA.start_freq.value() * 1e6  # freq in Hz
        stopF = QtTSA.stop_freq.value() * 1e6
        maskFreq = self.rbwMask(startF, stopF)
        limits = (maskFreq, highF.line.value(), lowF.line.value(), threshold.line.value())
        for spectrum in self.spectra:
            for marker in spectrum.mkr_list:
                marker.mkr_update(limits, spectrum.trace.is_visible)  # update markers that are inside the mask

    def setMarker(self, mkr_num):  # mkr number and type are provided from the GUI field changed event
        m_type = {0: QtTSA.m1_type.currentText(),
                  1: QtTSA.m2_type.currentText(),
                  2: QtTSA.m3_type.currentText(),
                  3: QtTSA.m4_type.currentText()}
        m_track = {0: QtTSA.m1track.value(),
                   1: QtTSA.m2track.value(),
                   2: QtTSA.m3track.value(),
                   3: QtTSA.m4track.value()}
        for spectrum in self.spectra:
            spectrum.mkr_list[mkr_num].set_type(m_type.get(mkr_num), m_track.get(mkr_num))

    def set_preset_marker(self):
        self.s0.del_ps_mkr(QtTSA.graphWidget)
        presetmarker.unlimited()
        for i in range(0, presetmarker.tm.rowCount()):
            try:
                startF = presetmarker.tm.record(i).value('StartF')
                stopF = presetmarker.tm.record(i).value('StopF')
                colour = presetmarker.tm.record(i).value('colour')
                name = presetmarker.tm.record(i).value('name')
                visible = presetmarker.tm.record(i).value('visible')
                on = QtTSA.presetMarker.isChecked()
                if on and visible and stopF in (0, ''):  # it is not a band marker
                    marker = self.s0.set_ps_mkr(QtTSA.graphWidget, startF, colour, name)
                    self.s0.label_ps_mkr(marker, colour, False)
                if on and visible and stopF not in (0, '', startF):  # it is a band marker
                    band_start = self.s0.set_ps_mkr(QtTSA.graphWidget, startF, colour, name)
                    self.s0.label_ps_mkr(band_start, colour, True)
                    band_end = self.s0.set_ps_mkr(QtTSA.graphWidget, stopF, colour, name)
                    self.s0.label_ps_mkr(band_end, colour, True)
            except ValueError:
                logging.info('preset_marker {name} value error')
                continue


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


# class Marker:

#     def setup(self, colour):
#         '''restore the marker frequencies from the configuration database and set starting conditions'''
#         self.line.setValue(numbers.tm.record(0).value(self.guiRef(2)))
#         self.line.label.setColor(colour)
#         self.line.label.setPosition(0.02)
#         self.line.label.setMovable(True)
#         self.line.setPen(color=colour, width=0.5)
#         self.mType()
#         # self.traceLink(self.guiRef(1).value())
#         self.setLevel(self.guiRef(3).value())
#         self.deltaline.hide()
#         self.deltaline.setValue(0)
#         self.deltaF = 0
#         self.deltaline.label.setPosition(0.05)
#         self.deltaline.label.setMovable(True)
#         M2.tplot.setXLink(M1.tplot)
#         M3.tplot.setXLink(M1.tplot)
#         M4.tplot.setXLink(M1.tplot)

#     def setPolarPlot(self):
#         if self.markerType != 'Off':
#             pattern.ui.progress.setValue(0)
#             if pattern.ui.manual.isChecked():
#                 self.samples = []
#             else:
#                 self.sweeptime = []
#                 self.amplitude = []
#             self.runTimer.start()

#     def updatePolarPlot(self):
#         if pattern.ui.manual.isChecked():
#             if len(self.samples) < pattern.scanCount.value():
#                 self.samples.append(self.dBm)
#                 pattern.ui.progress.setValue(int(100 * len(self.samples) / pattern.scanCount.value()))
#                 return
#             else:
#                 if pattern.ui.max.isChecked():
#                     self.amplitude.append(np.max(self.samples))
#                 if pattern.ui.avg.isChecked():
#                     self.amplitude.append(np.average(self.samples))
#                 if pattern.ui.min.isChecked():
#                     self.amplitude.append(np.min(self.samples))
#                 self.sweeptime.append(pattern.heading.value())  # append the current heading (instead of rotation time)
#                 self.runTimer.invalidate()
#                 theta = np.divide((np.multiply(self.sweeptime, np.pi)), 180)  # convert heading in degrees to radians
#         else:
#             if self.sweeptime == []:  # auto plot has just been started
#                 self.sweeptime.append(0)
#             else:
#                 self.sweeptime.append(self.runTimer.elapsed() / 1000)
#             self.amplitude.append(self.dBm)
#             if pattern.ui.clockwise.isChecked():
#                 theta = np.divide((np.multiply(self.sweeptime, 2 * np.pi)), pattern.ui.rotateTime.value())
#                 pattern.ui.heading.setValue(int(360*(theta[-1] / (2 * np.pi))))
#             else:
#                 theta = np.divide((np.multiply(self.sweeptime, -2 * np.pi)), pattern.ui.rotateTime.value())
#                 pattern.ui.heading.setValue(360 + int(360*(theta[-1] / (2 * np.pi))))
#             pattern.ui.progress.setValue(int(100 * (abs(theta[-1]) / (2 * np.pi))))

#         peak = max(np.max(self.amplitude), pattern.ui.refdBm.value())  # peak is maximum when antenna points at Tx
#         factor = 40 - peak  # correction factor to make the max signal amplitude read 40 units on the polar grid
#         dBm = np.round(np.add(self.amplitude, factor), decimals=1)

#         r = np.clip(dBm, 0, 40)  # clip the signal vector to a max amplitude of 40 and minimum of 0
#         x = np.multiply(r, np.sin(theta))
#         y = np.multiply(r, np.cos(theta))
#         self.polar.setData(x, y, pen=self.linked.pen)

#         if self.sweeptime[-1] >= pattern.ui.rotateTime.value():  # rotation is complete
#             self.runTimer.invalidate()
#             if pattern.ui.beamUp.isChecked() and not pattern.ui.manual.isChecked():
#                 self.rotatePolarPlot(r, theta)

#     def rotatePolarPlot(self, r, theta):
#         pkIndex = np.argmax(self.amplitude)  # find the array index of the maximum signal
#         width = 0
#         for i in range(pkIndex, len(self.amplitude) - 1):
#             if self.amplitude[i] == self.amplitude[pkIndex]:
#                 width += 1
#         pkIndex = pkIndex + int(width / 2)  # beam centre is half the width of a symetrical antenna main lobe
#         pkBearing = 2 * np.pi * pkIndex / len(self.amplitude)

#         # calculate new values to rotate display
#         theta = np.subtract(theta, pkBearing)
#         x = np.multiply(r, np.sin(theta))
#         y = np.multiply(r, np.cos(theta))
#         self.polar.setData(x, y, pen=self.linked.pen)


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
        if table == presetFreqs.ui.typeTable:
            record = self.tm.record(self.currentRow)
            bands.filterType(True, record.value('preset'))
            bands.unlimited()
            presetFreqs.ui.psCount.setValue(bands.tm.rowCount())

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
                    record.setValue('preset', bandstype.fetch_ID('preset', presetFreqs.ui.filterBox.currentText()))
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
            gui_row = maps.tm.record(index).value('gui')
            if gui_row.split('.')[0] != 'QtTSA':
                gui = gui_row.split('.')[0] + '.ui.' + gui_row.split('.')[1]  # pyside6 mod
            else:
                gui = gui_row
            column = maps.tm.record(index).value('column')
            self.dwm.addMapping(eval(gui), int(column))

    def unlimited(self):  # remove 256 row limit for QSql Query
        while self.tm.canFetchMore():
            self.tm.fetchMore()

    def showAll(self):
        presetFreqs.ui.typeTable.clearSelection()
        self.tm.setFilter('')
        self.unlimited()
        presetFreqs.ui.psCount.setValue(bands.tm.rowCount())

    def update_row(self, row, **data):
        record = self.tm.record(row)
        for key, value in data.items():
            logging.debug(f'update_row: key = {key} value={value}')
            record.setValue(str(key), value)
        self.tm.setRecord(row, record)
        self.updateModel()

    def read_tables(self):  # read the correction tables from the tinySA and display in a table widget
        device = tinySA.dev_ref[offset.ui.port.currentIndex()]
        offset.ui.dev_type.setText(device.firmware[0])
        if device.sweeping:
            popUp(offset, "Cannot read from tinySA whilst a scan is running", 'Ok', 'Info')
            return
        self.unlimited()
        write_config = QMessageBox.StandardButton.Ok
        if self.tm.rowCount() > 0 and offset.ui.save_box.isChecked():
            message = ('OK to over-write config database table with\rcorrection data from tinySA?')
            write_config = popUp(offset, message, 'OkC', 'Question')
            if write_config == QMessageBox.StandardButton.Ok:
                correction.deleteRow(single=False)
        offset.ui.tsa_table.clear()
        offset.ui.tsa_table.setRowCount(200)
        offset.ui.tsa_table.setColumnCount(4)
        offset.ui.tsa_table.setHorizontalHeaderLabels(['mode', 'entry', 'frequency', 'dB'])
        offset.ui.tsa_table.show()
        offset.ui.tsa_table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeMode.ResizeToContents)

        k = 0
        for i in range(correctiontext.tm.rowCount()):
            # step through each correction mode, fetch its table from the tinySA
            command = 'correction ' + correctiontext.tm.record(i).value('value') + '\r'
            data = device.serialQuery(command)
            mode_table = data.splitlines()[1:]  # make a list of the rows, discard the mode header
            mode_rows = [row.split(' ')[1:] for row in mode_table]  # split each row into a list, discard first col
            for j in range(20):
                # step through each row of the current mode and write the fields to the tablewidget 'tsa_table'
                mode = str(mode_rows[j][0])
                entry = str(mode_rows[j][1])
                frequency = str(mode_rows[j][2])
                dB = str(mode_rows[j][3])
                offset.ui.tsa_table.setItem(k+j, 0, QTableWidgetItem(mode))
                offset.ui.tsa_table.setItem(k+j, 1, QTableWidgetItem(entry))
                offset.ui.tsa_table.setItem(k+j, 2, QTableWidgetItem(frequency))
                offset.ui.tsa_table.setItem(k+j, 3, QTableWidgetItem(dB))

                if offset.ui.save_box.isChecked() and write_config == QMessageBox.StandardButton.Ok:
                    self.insertData(mode=mode, entry=entry, frequency=frequency, dB=dB)
            k += 20

    def upload_correction(self):  # upload the correction table(s) from the config database to the tinySA
        device = tinySA.dev_ref[offset.ui.port.currentIndex()]
        offset.ui.dev_type.setText(device.firmware[0])
        if device.sweeping:
            popUp(offset, "Cannot read from tinySA whilst a scan is running", 'Ok', 'Info')
            return
        self.unlimited()
        offset.ui.progress.setValue(0)
        update_failed = False
        for i in range(self.tm.rowCount()):
            record = self.tm.record(i)
            mode = str(record.value('mode')) + ' '
            entry = str(record.value('entry')) + ' '
            frequency = str(record.value('frequency')) + ' '
            dB = str(record.value('dB'))
            command = 'correction ' + mode + entry + frequency + dB + '\r'
            response = device.serialQuery(command)
            logging.debug(f'upload_correction(): {response}')
            if response != 'updated ' + entry + 'to ' + frequency + dB:
                # the error trapping on the tinySA is not comprehensive so this may not work for all scenarios
                update_failed = True
                logging.info(f'Update failure: {command}')
            else:
                offset.ui.progress.setValue(100 * int(i / (self.tm.rowCount() - 1)))
        if update_failed:
            popUp(offset, "One or more of the updates failed.", 'Ok', 'Critical')

###############################################################################
# respond to GUI signals


def band_changed():
    index = QtTSA.band_box.currentIndex()
    startF = bandselect.tm.record(index).value('StartF')
    stopF = bandselect.tm.record(index).value('StopF')
    if stopF not in (0, '', startF):
        with QSignalBlocker(QtTSA.start_freq):
            QtTSA.start_freq.setValue(startF / 1e6)
        with QSignalBlocker(QtTSA.stop_freq):
            QtTSA.stop_freq.setValue(stopF / 1e6)
        tinySA.setStartFreq()
        # tinySA.freq_changed(False)  # start/stop mode
    else:
        centreF = startF / 1e6
        with QSignalBlocker(QtTSA.centre_freq):
            QtTSA.centre_freq.setValue(centreF)
        with QSignalBlocker(QtTSA.span_freq):
            QtTSA.span_freq.setValue(int(centreF / 10))  # default span to a tenth of the centre freq
        tinySA.setCentreFreq()
        # tinySA.freq_changed(True)  # centre mode
    numbers.dwm.submit()


# def addBand():
#     if QtTSA.m1_type.currentText() == 'Off':
#         message = 'Please enable Marker 1'
#         popUp(QtTSA, message, 'Ok', 'Info')
#         return
#     if QtTSA.m1_type.currentText() != 'Off' and QtTSA.m2_type.currentText() != 'Off':  # Two markers to set a band limit
#         if M1.line.value() >= M2.line.value():
#             message = 'M1 frequency >= M2 frequency'
#             popUp(QtTSA, message, 'Ok', 'Info')
#             return
#         ID = bandstype.fetch_ID('preset', str(QtTSA.filterBox.currentText()))
#         title = "New Frequency Band"
#         message = "Enter a name for the new band."
#         bandName, ok = QInputDialog.getText(None, title, message, QLineEdit.Normal, "")
#         bands.insertData(name=bandName, type=ID, startF=f'{int(M1.line.value())}',
#                          stopF=f'{int(M2.line.value())}', visible=1, colour=colours.fetch_ID('colour', 'green'))


# def addFixed():
#     title = "New fixed frequency Marker"
#     message = "Enter a name for the fixed Marker"
#     fixedMkr, ok = QInputDialog.getText(None, title, message, QLineEdit.Normal, "")
#     bands.insertData(name=fixedMkr, type=12, startF=f'{int(M1.line.value())}',
#                      stopF=0, visible=1, colour=colours.fetch_ID('colour', 'orange'))  # preset type 12 = fixed Marker


def pointsChanged():
    if QtTSA.points_auto.isChecked():
        QtTSA.points_box.setEnabled(False)
        QtTSA.rbw_box.setEnabled(True)
    else:
        QtTSA.points_box.setEnabled(True)
    tinySA.setting_change()


def memChanged():
    depth = QtTSA.memBox.value()
    if depth < QtTSA.avgBox.value():
        QtTSA.avgBox.setValue(depth)


def setPreferences():  # called at startup and when the preferences window is closed
    checkboxes.dwm.submit()
    bands.tm.submitAll()
    threshold.line.setValue(settings.ui.peakThreshold.value())
    best.visible(settings.ui.neg25Line.isChecked())
    maximum.visible(settings.ui.zeroLine.isChecked())
    damage.visible(settings.ui.plus6Line.isChecked())

    if QtTSA.presetMarker.isChecked():
        tinySA.set_preset_marker()


def dialogPrefs():  # called by clicking on the setup > preferences menu
    presetFreqs.ui.show()
    presetFreqs.ui.psCount.setValue(bands.tm.rowCount())


def about():
    message = ('TinySA Ultra GUI programme using Qt and PySide6\
               nAuthor: Ian Jefferson G4IXT\n\nVersion: {} \nConfig: {}'
               .format(app.applicationVersion(), config.databaseName()))
    popUp(QtTSA, message, 'Ok', 'Info')


def clickEvent():
    logging.info('clickEvent')


def correction_filter():
    if offset.ui.filter_box.isChecked():
        sql = 'mode = "' + offset.ui.correction_mode.currentText() + '"'
        correction.tm.setFilter(sql)
    else:
        correction.tm.setFilter('')


##############################################################################
# other methods

def saveFile(frequencies, readings):
    if settings.ui.saveSweep.isChecked():
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
                " and will reset some settings.ui."
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
    usbCheck.stop()
    usbInstr.stop(restart=False)
    if len(usbInstr.ports) != 0:
        # save the marker frequencies
        record = numbers.tm.record(0)
        # record.setValue('m1f', float(M1.line.value()))
        # record.setValue('m2f', float(M2.line.value()))
        # record.setValue('m3f', float(M3.line.value()))
        # record.setValue('m4f', float(M4.line.value()))
        numbers.tm.setRecord(0, record)

    usbInstr.closePort()

    # save the gui field values and checkbox states
    checkboxes.dwm.submit()
    numbers.dwm.submit()
    disconnect(config)
    app.closeAllWindows()
    logging.info('QtTinySA Closed')


def popUp(window, message, button, icon):
    if window is None:
        window = QtTSA
    icons = {'Warn': QMessageBox.Icon.Warning, 'Info': QMessageBox.Icon.Information,
             'Critical': QMessageBox.Icon.Critical, 'Question': QMessageBox.Icon.Question}
    buttons = {'Ok': QMessageBox.StandardButton.Ok, 'Cancel': QMessageBox.StandardButton.Cancel,
               'OkC': QMessageBox.StandardButton.Ok | QMessageBox.StandardButton.Cancel}
    msg = QMessageBox(parent=(window))
    msg.setIcon(icons.get(icon))
    msg.setText(message)
    msg.setStandardButtons(buttons.get(button))
    return msg.exec()


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
    pattern.ui.plotwidget.setAspectLocked(True)
    pattern.ui.plotwidget.hideAxis('bottom')
    pattern.ui.plotwidget.hideAxis('left')
    for i in range(1, rings + 1):
        r = i * radius / rings
        circle = QtWidgets.QGraphicsEllipseItem(-r, -r, 2 * r, 2 * r)
        circle.setPen(pyqtgraph.mkPen('grey', width=0.3))
        pattern.ui.plotwidget.addItem(circle)
    r = radius - (radius/(rings*3))
    circle = QtWidgets.QGraphicsEllipseItem(-r, -r, 2 * r, 2 * r)
    circle.setPen(pyqtgraph.mkPen('red', width=0.3))
    pattern.ui.plotwidget.addItem(circle)

    # Add radial lines
    for angle_deg in range(0, 360, 15):
        angle_rad = np.deg2rad(angle_deg)
        x = radius * np.cos(angle_rad)
        y = radius * np.sin(angle_rad)
        line = QtWidgets.QGraphicsLineItem(0, 0, x, y)
        line.setPen(pyqtgraph.mkPen('grey', width=0.5))
        pattern.ui.plotwidget.addItem(line)


def startPolarPlot():
    if not fading.ui.isVisible():
        popUp(QtTSA, 'Measurement requires Signal Level Monitor window to be open', 'Ok', 'Critical')
    # else:
    #     M1.setPolarPlot()
    #     M2.setPolarPlot()
    #     M3.setPolarPlot()
    #     M4.setPolarPlot()


def connectActive():
    '''Connect signals from controls that send messages to tinySA or use trace data.  Called by setGUI().'''

    QtTSA.atten_box.valueChanged.connect(tinySA.setting_change)
    QtTSA.atten_auto.clicked.connect(tinySA.setting_change)
    QtTSA.spur_box.currentIndexChanged.connect(tinySA.setting_change)
    QtTSA.lna_box.clicked.connect(tinySA.setting_change)
    QtTSA.setRange.clicked.connect(tinySA.sweep_as_zoomed)

    # frequencies
    QtTSA.start_freq.valueChanged.connect(tinySA.setStartFreq)
    QtTSA.stop_freq.valueChanged.connect(tinySA.setStartFreq)
    QtTSA.centre_freq.valueChanged.connect(tinySA.setCentreFreq)  # centre/span mode
    QtTSA.span_freq.valueChanged.connect(tinySA.setCentreFreq)  # centre/span mode
    QtTSA.band_box.currentIndexChanged.connect(band_changed)

    QtTSA.rbw_auto.clicked.connect(tinySA.rbwChanged)
    QtTSA.rbw_box.currentIndexChanged.connect(tinySA.rbwChanged)
    QtTSA.points_auto.stateChanged.connect(pointsChanged)
    QtTSA.points_box.valueChanged.connect(pointsChanged)

    # QtTSA.sampleRepeat.valueChanged.connect(tinySA.sampleRep)

    # 3D graph controls
    QtTSA.timeSpectrum.clicked.connect(lambda: QtTSA.stackedWidget.setCurrentWidget(QtTSA.View3D))
    QtTSA.analyser.clicked.connect(lambda: QtTSA.stackedWidget.setCurrentWidget(QtTSA.ViewNormal))

    # filebrowse
    filebrowse.ui.download.clicked.connect(lambda: tinySA.save_file(True))
    filebrowse.ui.saveAll.clicked.connect(lambda: tinySA.save_file(False))
    filebrowse.ui.listWidget.itemClicked.connect(tinySA.show_file)

    # Sweep time
    # QtTSA.sweepTime.valueChanged.connect(lambda: tinySA.sweepTime(QtTSA.sweepTime.value()))

    # level calibration
    offset.ui.correction_mode.currentTextChanged.connect(correction_filter)
    offset.ui.filter_box.stateChanged.connect(correction_filter)
    offset.ui.read_button.clicked.connect(correction.read_tables)
    offset.ui.upload_button.clicked.connect(correction.upload_correction)


def connectPassive():
    # Connect signals from GUI controls that don't cause messages to go to the tinySA

    QtTSA.memBox.valueChanged.connect(memChanged)

    # Quit
    QtTSA.actionQuit.triggered.connect(app.closeAllWindows)
    QtTSA.scan_button.clicked.connect(tinySA.scan)
    QtTSA.run3D.clicked.connect(tinySA.scan)

    # # marker setting within span range
    QtTSA.mkr_start.clicked.connect(tinySA.markerToStart)
    QtTSA.mkr_centre.clicked.connect(tinySA.markerSpread)

    # # marker tracking level
    QtTSA.m1track.valueChanged.connect(lambda: tinySA.setMarker(0))
    QtTSA.m2track.valueChanged.connect(lambda: tinySA.setMarker(1))
    QtTSA.m3track.valueChanged.connect(lambda: tinySA.setMarker(2))
    QtTSA.m4track.valueChanged.connect(lambda: tinySA.setMarker(3))

    # marker type changes
    QtTSA.m1_type.currentTextChanged.connect(lambda: tinySA.setMarker(0))
    QtTSA.m2_type.currentTextChanged.connect(lambda: tinySA.setMarker(1))
    QtTSA.m3_type.currentTextChanged.connect(lambda: tinySA.setMarker(2))
    QtTSA.m4_type.currentTextChanged.connect(lambda: tinySA.setMarker(3))

    # # frequency band and fixed markers
    QtTSA.presetMarker.clicked.connect(tinySA.set_preset_marker)
    # QtTSA.presetLabel.clicked.connect()
    # QtTSA.addBandPreset.clicked.connect(addBand)
    # QtTSA.addFix.clicked.connect(addFixed)
    QtTSA.filterBox.currentTextChanged.connect(tinySA.set_preset_marker)

    # trace checkboxes
    QtTSA.trace1.stateChanged.connect(tinySA.s0.enable)
    QtTSA.trace2.stateChanged.connect(tinySA.s1.enable)
    QtTSA.trace3.stateChanged.connect(tinySA.s2.enable)
    QtTSA.trace4.stateChanged.connect(tinySA.s3.enable)

    # preset freqs and settings
    presetFreqs.ui.addPs.clicked.connect(bands.addRow)
    presetFreqs.ui.deletePs.clicked.connect(lambda: bands.deleteRow(True))
    presetFreqs.ui.deleteAll.clicked.connect(lambda: bands.deleteRow(False))
    presetFreqs.ui.freqTable.clicked.connect(lambda: bands.tableClicked(presetFreqs.ui.freqTable))
    presetFreqs.ui.typeTable.clicked.connect(lambda: bandstype.tableClicked(presetFreqs.ui.typeTable))
    presetFreqs.ui.addPsType.clicked.connect(bandstype.addRow)
    presetFreqs.ui.deletePsType.clicked.connect(bandstype.deletePsType)
    presetFreqs.ui.clearFilter.clicked.connect(bands.showAll)

    presetFreqs.ui.finished.connect(setPreferences)  # update database checkboxes table on dialogue window close

    presetFreqs.ui.exportPs.pressed.connect(lambda: bands.exportData(''))
    presetFreqs.ui.importPs.pressed.connect(lambda: bands.importData(''))

    QtTSA.filterBox.currentTextChanged.connect(lambda: bandselect.filterType(False, QtTSA.filterBox.currentText()))
    QtTSA.actionPresets.triggered.connect(dialogPrefs)  # open preferences dialogue when its menu is clicked
    QtTSA.actionSettings.triggered.connect(settings.ui.show)
    QtTSA.actionCorrection.triggered.connect(tinySA.correction_window)

    # preferences
    QtTSA.actionAbout_QtTinySA.triggered.connect(about)

    # Waterfall
    QtTSA.waterfall_size.valueChanged.connect(setSize)

    # Measurement menu
    QtTSA.actionPhNoise.triggered.connect(phasenoise.ui.show)
    QtTSA.actionFading.triggered.connect(fading.ui.show)
    QtTSA.actionPattern.triggered.connect(pattern.ui.show)

    # phase noise
    phasenoise.ui.centre.clicked.connect(tinySA.centreTone)

    # File menu
    QtTSA.actionBrowse_TinySA.triggered.connect(tinySA.file_browser)
    filebrowse.ui.port.currentIndexChanged.connect(tinySA.list_files)

    # polar pattern
    pattern.ui.measure.clicked.connect(startPolarPlot)

    # correction
    offset.ui.export_button.clicked.connect(lambda: correction.exportData(''))
    offset.ui.import_button.clicked.connect(lambda: correction.importData(''))

    # 3D
    QtTSA.zoom.valueChanged.connect(lambda: tinySA.timespectrum.zoom(QtTSA.zoom.value()))
    QtTSA.x_rotation.valueChanged.connect(lambda: tinySA.timespectrum.rotateX(QtTSA.x_rotation.value()))


###############################################################################
# Instantiate classes

# create QApplication for the GUI
app = QtWidgets.QApplication([])
app.setApplicationName('QtTinySA')
app.setApplicationVersion(' v1.3.22')

loader = CustomLoader()
QtTSA = loader.load("spectrum.ui", None)
presetFreqs = CustomDialogue(app_dir('bands.ui'))
settings = CustomDialogue(app_dir('settings.ui'))
filebrowse = CustomDialogue(app_dir('filebrowse.ui'))
phasenoise = CustomDialogue(app_dir('phasenoise.ui'))
fading = CustomDialogue(app_dir('fading.ui'))
pattern = CustomDialogue(app_dir('pattern.ui'))
offset = CustomDialogue(app_dir('offset.ui'))

# Markers
multiplot = pyqtgraph.GraphicsLayout()  # for plotting marker signal level over time
fading.ui.grView.setCentralItem(multiplot)

# limit lines
best = Limit('gold', None, -25, movable=False)
maximum = Limit('red', None, 0, movable=False)
damage = Limit('red', None, 6, movable=False)
threshold = Limit('cyan', None, settings.ui.peakThreshold.value(), movable=True)
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

# # pyqtgraph settings for waterfall and histogram display
QtTSA.waterfall.setDefaultPadding(padding=0.005)
QtTSA.waterfall.getPlotItem().hideAxis('bottom')
QtTSA.waterfall.setLabel('left', '.', **{'color': '#FFF', 'font-size': '2pt'})
QtTSA.waterfall.invertY(True)

QtTSA.histogram.setDefaultPadding(padding=0)
QtTSA.histogram.plotItem.invertY(True)
QtTSA.histogram.getPlotItem().hideAxis('bottom')
QtTSA.histogram.getPlotItem().hideAxis('left')

# widget settings for Phase Noise
phasenoise.ui.plotWidget.setYRange(-120, -40)
phasenoise.ui.plotWidget.plotItem.showGrid(x=True, y=True, alpha=0.5)
phasenoise.ui.plotWidget.plotItem.setLogMode(x=True)
phasenoise.ui.plotWidget.setLabel('bottom', 'Offset Frequency', units='Hz')
phasenoise.ui.plotWidget.setLabel('left', 'Phase Noise', units='dBc/Hz')

# (maxTickLevel=0, tickAlpha=int)

# pyqtgraph settings for antenna pattern display
createPolarGrid(4, 40)


###############################################################################
# set up the application
logging.info(f'{app.applicationName()}{app.applicationVersion()}')

# Database and models for configuration settings
config = connect("QtTSAprefs.db", "settings", 1315)  # third parameter is the database version

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
bands.tm.setRelation(6, QSqlRelation("SVGColour", "ID","colour"))  # set "marker" column to a colours choice combo box
presets = QSqlRelationalDelegate(presetFreqs.ui.freqTable)
presetFreqs.ui.freqTable.setItemDelegate(presets)
colHeader = presetFreqs.ui.freqTable.horizontalHeader()
colHeader.setSectionResizeMode(QtWidgets.QHeaderView.ResizeMode.ResizeToContents)
bands.tm.select()

# populate the preset Types table in the preset frequencies window
bandstype = ModelView('freqtype', config, ())
bandstype.tm.select()
presetFreqs.ui.typeTable.setModel(bandstype.tm)
presetFreqs.ui.typeTable.hideColumn(0)  # hide primary key so user can't change it

# populate the correction values table in the correction window
correction = ModelView('correction', config, (0, 1, 2, 3))
c_header = offset.ui.c_table.horizontalHeader()
c_header.setSectionResizeMode(QtWidgets.QHeaderView.ResizeMode.ResizeToContents)
correction.tm.select()
offset.ui.c_table.setModel(correction.tm)
offset.ui.c_table.hideColumn(0)

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
presetFreqs.ui.freqTable.setModel(bands.tm)
presetFreqs.ui.freqTable.hideColumn(0)  # ID

# connect the settings window trace colours widget to the data model
tracecolours = ModelView('trace', config, (0, 1))
tracecolours.tm.setRelation(2, QSqlRelation('SVGColour', 'ID', 'colour'))
tracecolours.tm.setEditStrategy(QSqlRelationalTableModel.EditStrategy.OnFieldChange)
tracesettings = QSqlRelationalDelegate(settings.ui.colourTable)
settings.ui.colourTable.setItemDelegate(tracesettings)
traceHeader = settings.ui.colourTable.horizontalHeader()
traceHeader.setSectionResizeMode(QtWidgets.QHeaderView.ResizeMode.ResizeToContents)
settings.ui.colourTable.setModel(tracecolours.tm)
settings.ui.colourTable.hideColumn(0)  # ID
settings.ui.colourTable.verticalHeader().setVisible(True)
tracecolours.tm.select()

# settingstext = ModelView('settings', config, ())

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
offset.ui.correction_mode.setModel(correctiontext.tm)
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

# try to open a USB connection to hardware....... need to check if it works in Windows now
usbInstr = USBdevice()
tinySA = Analyser()
# usbInstr.probe()

usbCheck = QtCore.QTimer()
usbCheck.timeout.connect(usbInstr.probe)
usbCheck.start(500)

tinySA.setGraphs()
tinySA.setGUI()
tinySA.setSignals()

###############################################################################
# run the application until the user closes it
if settings.ui.bold_text.isChecked():
    app.setStyleSheet("QWidget { font-weight: bold; }")  # enhancement issue 118

try:
    app.exec()
finally:
    exit_handler()  # close cleanly
