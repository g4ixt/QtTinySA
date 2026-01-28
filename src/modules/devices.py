#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jan 13 09:45:56 2026
"""

# Copyright 2026 Ian Jefferson G4IXT
# SPDX-License-Identifier: GPL-3.0-or-later

import logging
import serial
import time
import queue
import struct
import numpy as np
from platform import system
from PySide6.QtCore import QObject, QElapsedTimer, Signal, Slot, QRunnable
from serial.tools import list_ports
from datetime import datetime


class USBdevice(QObject):
    def __init__(self):
        super().__init__()
        # self.usb = None
        self.ports = []
        self.firmware = None
        self.setSignals()

    def setSignals(self):
        # these signals forward the signals from the devices to the analyser class (& are connected in there)
        self.signals = WorkerSignals()
        self.sigs = {"result": self.signals.result,
                     "finished": self.signals.finished,
                     "save": self.signals.saveResults,
                     "reset": self.signals.resetGUI,
                     "ends": self.signals.sweepEnds}

    def probe(self):
        VID = (0x0483, 0x1d50)  # 1155 tinySA/NanoVNA, limeSDR
        PID = (0x5740, 0x6108)  # 22336 tinySA/NanoVNA, limeSDR
        usbPorts = list_ports.comports()

        # detect devices as they connect
        for port in usbPorts:
            if port.vid in VID and port.pid in PID and port not in self.ports:
                self.ports.append(port)
                logging.info(f'found {port.product} on {port.device}')

        # detect devices that disconnect
        for port in self.ports:
            if port not in usbPorts:
                logging.info(f'{port.product} has disconnected from {port.device}')
                self.ports.remove(port)

    def connect(self):
        # try to set USB connections to different hardware....... need to check if it works in Windows now
        self.dev0 = self.dev1 = self.dev2 = self.dev3 = None

        for port in self.ports:
            func = {"tinySA": Tiny(port.device, port.product, self.sigs, basic=True),
                    "tinySA4": Tiny(port.device, port.product, self.sigs, basic=False),
                    "NanoVnaPro Virtual ComPort": Nano(port.device, port.product, self.sigs),
                    "LimeSDR-USB": Lime(port.device, port.product, self.sigs)}
            if self.dev0 is None:
                self.dev0 = func[port.product]
                self.dev0.test(port.device)
                continue
            if self.dev1 is None and len(self.ports) > 1:
                self.dev1 = func[port.product]
                self.dev1.test(port.device)
                continue
            if self.dev2 is None and len(self.ports) > 2:
                self.dev2 = func[port.product]
                continue
            if self.dev3 is None and len(self.ports) > 3:
                self.dev3 = func[port.product]

    def stop(self):
        x = 0
        
    # logging.info(f'dev0 is a {self.dev0.product}')
    # logging.info(f'dev1 is a {self.dev1.product}')
    # settings.ui.deviceBox.addItem(self.identify(port) + " on " + port.device)
    # def identify(self, port):
    #     # Windows returns no information to pySerial list_ports.comports()
    #     if system() == 'Linux' or system() == 'Darwin':
    #         return port.product
    #     else:
    #         return 'USB device'S-2026-2837


class WorkerSignals(QObject):
    error = Signal(str)
    result = Signal(np.ndarray, np.ndarray, np.ndarray, np.ndarray, float)
    fullSweep = Signal(np.ndarray, np.ndarray)
    saveResults = Signal(np.ndarray, np.ndarray)
    resetGUI = Signal(np.ndarray, np.ndarray)
    finished = Signal()
    sweepEnds = Signal(np.ndarray)


class Worker(QRunnable):
    '''Worker threads so that functions can run outside GUI event loop'''

    def __init__(self, fn, *args):
        super(Worker, self).__init__()
        self.fn = fn
        self.args = args
        self.signals = WorkerSignals()

    @Slot()
    def run(self):
        '''Initialise the runner'''
        logging.info(f'{self.fn.__name__} thread running')
        self.fn(*self.args)
        logging.info(f'{self.fn.__name__} thread stopped')


class Tiny(QObject):
    def __init__(self, usbPort, product, sigs, basic=False):
        super().__init__()
        self.fifo = queue.SimpleQueue()
        self.usb = None
        self.product = product
        self.sweeping = None
        self.basic = basic
        self.setScale()
        self.setSignals(sigs)
        self.scanMemory = 10  # test, need to get this from the GUI

    def setSignals(self, sigs):
        self.signals = WorkerSignals()
        self.signals.result.connect(sigs["result"])
        self.signals.finished.connect(sigs["finished"])
        self.signals.saveResults.connect(sigs["save"])
        self.signals.resetGUI.connect(sigs["reset"])
        self.signals.sweepEnds.connect(sigs["ends"])

    def test(self, usbPort):  # tests tinySA comms and initialise if found
        try:
            self.usb = serial.Serial(usbPort, baudrate=576000)
            logging.info(f'Serial port {usbPort} open: {self.usb.isOpen()}')
        except serial.SerialException:
            logging.info('Serial port exception. Is your username in the "dialout" group?')
            # popUp(QtTSA, 'Serial port exception', 'Ok', 'Critical')
        if self.usb:
            for i in range(4):  # try 4 times to communicate with tinySA over USB serial
                firmware = self.version()
                if firmware[:6] == 'tinySA':
                    logging.info(f'{usbPort} test {i} reports firmware version {firmware[:16]}')
                    break
                else:
                    time.sleep(1)
            # split firmware into a list of [device, major version number, minor version number, other stuff]
            self.firmware = firmware.replace('_', '-').split('-')
            if firmware[:6] == 'tinySA':
                sd = str.splitlines(self.listSD())
                if "ID.bmp 307322" in sd:
                    logging.info(f'ID found = {sd[-1]}')
                if firmware[0] == 'tinySA4' and float(self.firmware[1][-3:] + self.firmware[2]) < 1.4177:
                    logging.info('for fastest scan speed, upgrade firmware to v1.4-177 or later')
                if self.firmware[1][0] == "v":
                    return self.firmware  # setForDevice needs this info
                else:
                    logging.info(f'{usbPort} test found unexpected firmware {firmware}')
            else:
                logging.info(f'firmware {firmware} on {usbPort} is not a tinySA')

    def close(self):
        if self.usb:
            self.usb.close()
            logging.info(f'Serial port open: {self.usb.isOpen()}')
            self.usb = None

    # def clearBuffer(self):
    #     # self.usb.timeout = 1
    #     while self.usb.inWaiting():
    #         self.usb.read_all()  # keep the serial buffer clean
    #         time.sleep(0.01)

    def sweepTimeout(self, frequencies, rbwTxt, spur):  # freqs are in Hz
        startF = frequencies[0]
        stopF = frequencies[-1]
        points = np.size(frequencies)
        if rbwTxt =="auto":
            # rbw auto setting from tinySA: ~7 kHz per 1 MHz scan frequency span
            rbw = (stopF - startF) * 7e-6
        else:
            rbw = float(rbwTxt)
        rbw = np.clip(rbw, 0.2, 850)  # apply limits
        # timeout can be very long - use a heuristic approach
        # 1st summand is the scanning time, 2nd summand is the USB transfer overhead
        timeout = ((stopF - startF) / 20e3) / (rbw ** 2) + points / 500
        if (spur == 'on' and stopF > 8 * 1e8) or spur == 'auto':
            timeout *= 2  # scan time doubles with spur on or spur auto above 800 MHz
        # transfer is done in blocks of 20 points, this is the timeout for one block
        timeout = timeout * 20 / points + 1  # minimum is 1 second
        logging.debug(f'sweepTimeout = {timeout:.2f} s')
        return timeout

    def measurement(self, startF, stopF, frequencies, readings, maxima, minima, loop=True):  # run in separate thread
        sweepCount = 0
        updateTimer = QElapsedTimer()
        points = np.size(frequencies)
        self.threadRunning = True
        firstRun = True
        self.sweeping = True
        # version = int(self.firmware[2])  # just the firmware version number

        # self.runTimer.start()  # debug
        # logging.debug(f'elapsed time = {self.runTimer.nsecsElapsed()/1e6:.3f}mS')  # debug

        updateTimer.start()  # used to trigger the signal that sends measurements to updateGUI()
        while self.sweeping:

            # if LNB:  # LNB / Transverter Mode
            #     startF, stopF = self.freqOffset(frequencies)
            # else:
            #     command = f'scanraw {int(frequencies[0])} {int(frequencies[-1])} {int(points)} 3\r'

            if loop:
                command = f'scanraw {int(startF)} {int(stopF)} {int(points)} 3\r'
            else:
                command = f'scanraw {int(startF)} {int(stopF)} {int(points)} 1\r'

            self.usb.timeout = 10  # self.sweepTimeout(frequencies)

            # firmware versions before 4.177 don't support auto-repeating scanraw so command must be sent each sweep
            if firstRun or not loop:
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
                    if loop:
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
                    frequencies, readings, maxima, minima = self.set_arrays()  # reads GUI values !!!
                    points = np.size(frequencies)
                    self.signals.resetGUI.emit(frequencies, readings)
                    self.usbSend()  # send all the queued commands in the FIFO buffer to the TinySA
                    updateTimer.start()
                    break

                timeElapsed = updateTimer.nsecsElapsed()  # how long this batch of measurements has been running, nS

                # Send the sesults to updateGUI if an update is due
                if timeElapsed/1e6 > 100:  # mS needs to be settings.ui.intervalBox.value():
                    self.signals.result.emit(frequencies, readings, maxima, minima, timeElapsed)  # send to updateGUI()
                    updateTimer.start()

        self.usb.read(2)  # discard the command prompt that the tinySA sends when sweeping ends
        self.threadRunning = False
        self.signals.finished.emit()

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
        logging.debug(f'serialWrite: command = {command}')
        self.usb.write(command.encode())
        self.usb.read_until(b'ch> ')  # skip command echo and prompt

    def clearBuffer(self):
        while self.usb.inWaiting():
            self.usb.read_all()  # keep the serial buffer clean
            time.sleep(0.01)

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

    def spur(self, sType):
        # sType = QtTSA.spur_box.currentText()
        if sType == 'auto' and self.basic:  # tinySA3 (basic) has no auto spur mode
            sType = 'on'
        command = 'spur ' + sType + '\r'
        self.fifo.put(command)

    def setTime(self):
        if not self.basic:  # and settings.ui.syncTime.isChecked():
            dt = datetime.now()
            y = dt.year - 2000
            command = f'time b 0x{y}{dt.month:02d}{dt.day:02d} 0x{dt.hour:02d}{dt.minute:02d}{dt.second:02d}\r'
            self.fifo.put(command)

    def setScale(self):
        self.scale = 174
        if self.basic:
            self.scale = 128

    def example(self):
        self.fifo.put('example\r')

    def setSweep(self, start, stop):  # only used to set a default on the tinySA
        if start is not None:
            self.serialWrite("sweep start %d\r" % start)
        if stop is not None:
            self.serialWrite("sweep stop %d\r" % stop)

    def sampleRep(self, rep):
        # sets the number of repeat measurements at each frequency point to the value in the GUI
        # command = f'repeat {QtTSA.sampleRepeat.value()}\r'
        command = f'repeat {rep}\r'
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

    def dialogBrowse(self):  # browse which device?
        if self.threadRunning:
            return
        elif self.usb:
            SD = self.listSD()
            ls = []
            for i in range(len(SD.splitlines())):
                ls.append(SD.splitlines()[i].split(" ")[0])


    @Slot()
    def saveFile(self, saveSingle=True):
        filebrowse.ui.saveProgress.setValue(0)
        SD = self.listSD()
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
                data = self.readSD(fileName)
                file.write(data)
            filebrowse.ui.saveProgress.setValue(int(100 * (i+1)/len(SD.splitlines())))
            filebrowse.ui.downloadInfo.setText(self.directory)  # show the path where the file was saved
            if filebrowse.ui.askForPath.isChecked():
                self.directory = None
            if saveSingle:
                filebrowse.ui.saveProgress.setValue(100)
                break

    def fileShow(self):
        self.memF.seek(0, 0)  # set the memory buffer pointer to the start
        self.memF.truncate()  # clear down the memory buffer to the pointer
        filebrowse.ui.picture.clear()
        fileName = filebrowse.ui.listWidget.currentItem().text()
        self.clearBuffer()  # clear the tinySA serial buffer
        self.memF.write(self.readSD(fileName))  # read the file from the tinySA memory card and store in memory buffer
        if fileName[-3:] == 'bmp':
            pixmap = QPixmap()
            pixmap.loadFromData(self.memF.getvalue())
            filebrowse.ui.picture.setPixmap(pixmap)

    def lna(self, on=True):
        if self.basic:
            return
        if on:
            command = 'lna on\r'
            self.fifo.put('attenuate 0\r')
        else:
            command = 'lna off\r'
            self.fifo.put('attenuate auto\r')
        self.fifo.put(command)

    def attenuator(self, setting):
        if setting != "auto":
            command = f'attenuate {str(setting)}\r'
        else:
            command = f'attenuate {setting}\r'
        self.fifo.put(command)

    def setRBW(self, rbw):  # may be called by measurement thread as well as normally ## stop doing that ########
        logging.debug(f'rbw = {rbw}')
        command = f'rbw {self.rbw}\r'
        self.fifo.put(command)

    @Slot()
    def threadEnds(self):
        if int(self.firmware[2]) >= 177:  # the firmware version number
            self.serialWrite('abort\r')
        self.runButton('Run')
        self.fifoTimer.start(500)  # start watching for commands

class Nano(QObject):
    def __init__(self, usbPort, product, sigs):
        super().__init__()
        self.usb = usbPort
        self.product = product
        self.setSignals(sigs)
        # nano vna

    def setSignals(self, sigs):
        self.signals = WorkerSignals()
        self.signals.result.connect(sigs["result"])
        self.signals.finished.connect(sigs["finished"])
        self.signals.saveResults.connect(sigs["save"])
        self.signals.resetGUI.connect(sigs["reset"])
        self.signals.sweepEnds.connect(sigs["ends"])

    def test(self, usbPort):  # tests NanoVNA comms and initialise if found
        try:
            self.usb = serial.Serial(usbPort, baudrate=576000)
            logging.info(f'Serial port {usbPort} open: {self.usb.isOpen()}')
        except serial.SerialException:
            logging.info('Serial port exception. Is your username in the "dialout" group?')
            # popUp(QtTSA, 'Serial port exception', 'Ok', 'Critical')
        if self.usb:
            for i in range(4):  # try 4 times to communicate with tinySA over USB serial
                firmware = self.version()
                if firmware:
                    logging.info(f'{usbPort} test {i} reports firmware version {firmware}')
                    break
            else:
                time.sleep(1)

    def version(self):
        version = self.serialQuery('version\r')
        return version

    def serialQuery(self, command):
        self.usb.write(command.encode())
        self.usb.read_until(command.encode() + b'\n')  # skip command echo
        response = self.usb.read_until(b'ch> ')  # until prompt
        logging.debug(f'serialQuery: response = {response}')
        return response[:-6].decode()  # remove prompt

    # all commands

    # help:                lists all the registered commands
    # reset:               usage: reset
    # cwfreq:              usage: cwfreq {frequency(Hz)}
    # saveconfig:          usage: saveconfig
    # clearconfig:         usage: clearconfig {protection key}
    # data:                usage: data [array]
    # frequencies:         usage: frequencies
    # scan:                usage: scan {start(Hz)} {stop(Hz)} [points] [outmask]
    # sweep:               usage: sweep [start(Hz)] [stop(Hz)] [points]
    # touchcal:            usage: touchcal
    # touchtest:           usage: touchtest
    # pause:               usage: pause
    # resume:              usage: resume
    # cal:                 usage: cal [load|open|short|thru|done|reset|on|off]
    # save:                usage: save {id}
    # recall:              usage: recall {id}
    # trace:               usage: trace [0|1|2|3|all] [{format}|scale|refpos|channel|off] [{value}]
    # marker:              usage: marker [1|2|3|4] [on|off|{index}]
    # edelay:              usage: edelay {value}
    # pwm:                 usage: pwm {0.0-1.0}
    # beep:                usage: beep on/off
    # lcd:                 usage: lcd X Y WIDTH HEIGHT FFFF
    # capture:             usage: capture
    # version:             usage: Show NanoVNA version
    # info:                usage: NanoVNA-F V2 info
    # SN:                  usage: NanoVNA-F V2 Serial Numbel


class Lime(QObject):
    def __init__(self, usbPort, product, sigs):
        super().__init__()
        self.setSignals(sigs)
        # soapy

    def setSignals(self, sigs):
        self.signals = WorkerSignals()
        self.signals.result.connect(sigs["result"])
        self.signals.finished.connect(sigs["finished"])
        self.signals.saveResults.connect(sigs["save"])
        self.signals.resetGUI.connect(sigs["reset"])
        self.signals.sweepEnds.connect(sigs["ends"])

    def test(self):
        try:
            logging.info('pyvisa port')
        except: # some other exception
            logging.info('pyvisa port exception.')


class RTL(QObject):
    def __init__(self, usbPort, product):
        super().__init__()
        # soapy


class SiglentSA(QObject):
    def __init__(self, visaPort, product, sigs):
        super().__init__()
        self.setSignals(sigs)
        # pyvisa

    def setSignals(self, sigs):
        self.signals = WorkerSignals()
        self.signals.result.connect(sigs["result"])
        self.signals.finished.connect(sigs["finished"])
        self.signals.saveResults.connect(sigs["save"])
        self.signals.resetGUI.connect(sigs["reset"])
        self.signals.sweepEnds.connect(sigs["ends"])