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
from PySide6.QtCore import QObject, QElapsedTimer, QTimer, Signal, Slot, QRunnable, QThreadPool
from serial.tools import list_ports
from datetime import datetime

threadpool = QThreadPool()


class USBdevice(QObject):
    stopped = Signal(bool)
    update_info = Signal(bool, int)

    def __init__(self):
        super().__init__()
        self.ports = []
        # self.firmware = None
        self.setSignals()
        self.run_connect = False
        self.is_scanning = False
        self.cnx_count = 0  # used to count 'connected' devices, not 'enabled' devices
        self.dev_list = None

    def setSignals(self):
        self.signals = WorkerSignals()
        # dev_sigs forward the signals from devices to the analyser class in QtTinySA.py (& are connected in there)
        self.dev_sigs = {"result": self.signals.result,
                         "save": self.signals.saveResults,
                         "ends": self.signals.sweepEnds,
                         "error": self.signals.error}

    def probe(self):
        VID = (0x0483, 0x1d50, 0x04b4)  # 1155 tinySA/NanoVNA, limeSDR, NanoVNA V2 +4
        PID = (0x5740, 0x6108, 0x0008)  # 22336 tinySA/NanoVNA, limeSDR, NanoVNA V2 +4
        usbPorts = list_ports.comports()
        # detect devices as they connect
        for port in usbPorts:
            if port.vid in VID and port.pid in PID and port not in self.ports:
                self.ports.append(port)
                self.run_connect = True
                logging.info(f'found {port.product} on {port.device}')
        # detect devices that have been turned off or lost contact
        for port in self.ports:
            if port not in usbPorts:
                self.disconnect(port.device)
                self.ports.remove(port)
        if self.run_connect:
            self.connect()

    def connect(self):
        # try to set USB connections to different hardware... need to check if it works in Windows now
        self.dev0 = self.dev1 = self.dev2 = self.dev3 = None
        self.dev_list = [self.dev0, self.dev1, self.dev2, self.dev3]  # all of which are set as None
        self.run_connect = False
        self.cnx_count = 0
        for index, port in enumerate(self.ports):
            # iterate through the ports, instantiate device classes and test serial comms
            if self.dev_list[index] is None and len(self.ports) > index:
                # instantiate device class & write its instance to the list, overwriting 'None'
                if port.product == "tinySA":
                    self.dev_list[index] = Tiny(port.device, port.product, self.dev_sigs, index, basic=True)
                if port.product == "tinySA4":
                    self.dev_list[index] = Tiny(port.device, port.product, self.dev_sigs, index, basic=False)
                if port.product == "LimeSDR-USB":
                    self.dev_list[index] = Lime(port.device, port.product, self.dev_sigs, index)
                if port.product == "NanoVnaPro Virtual ComPort":
                    self.dev_list[index] = Nano(port.device, port.product, self.dev_sigs, index)
                if port.product == "CDC-ACM Demo":
                    self.dev_list[index] = Nano(port.device, port.product, self.dev_sigs, index)
                self.cnx_count += 1
                # test using its specific commands and store results in its class instance
                test = self.dev_list[index].test(port.device)
                if test is True:
                    self.update_info.emit(True, index)
                else:
                    logging.info(f'test of {port} failed')

    def disconnect(self, usbPort):
        for i, device in enumerate(self.dev_list):
            if device and device.usbPort == usbPort:
               #logging.info(f'{port.product} has disconnected from {port.device}')
               #logging.info(f'removing disconnected {device.name} {usbPort} device instance')
                logging.info(f'{device.name} {device.sn} has disconnected from {device.usbPort}')
                self.stop(restart=False)
                device.close()
                del device
                self.dev_list[i] = None
                self.cnx_count -= 1
                self.update_info.emit(False, i)
        # self.update_info.emit()

    def closePort(self):
        for device in self.dev_list:
            if device:
                device.close()

    def renumber(self, num_on):
        count = 0
        for device in self.dev_list:
            if device is not None:
                if device.enabled:
                    device.id = count
                    count += 1
                else:
                    device.id = num_on + 1
            

    def start(self, spectra, rbw, depth, loop):
        for index, device in enumerate(self.dev_list):  # dev_list contains the device class instances
            if device is not None:
                if device.enabled:
                    device.usbSend()
                    startF = spectra[index].startF
                    stopF = spectra[index].stopF
                    points = spectra[index].points
                    device.sa = Worker(device.measurement, startF, stopF, points, rbw, loop)
                    device.fifoTimer.stop()
                    device.sweeping = True
                    threadpool.start(device.sa)
                    self.is_scanning = True
                    self.update_info.emit(False, -1)

    def controls(self, rbw, attn, lna, spur):
        for device in self.dev_list:
            if device is not None:
                device.set_ctrls(rbw, attn, lna, spur)  # device specific

    def stop(self, restart=False):
        for device in self.dev_list:  # dev_list contains the device class instances
            if device:
                if device.sweeping:
                    device.sweeping = False  # the measurement threads keep looping if this is True
        for device in self.dev_list:
            if device:
                if device.threadRunning:
                    logging.debug('waiting for measurement thread to stop')
                    time.sleep(0.1)
        self.is_scanning = False
        self.update_info.emit(False, -1)
        self.stopped.emit(restart)

    # def identify(self, port):
    #     # Windows returns no information to pySerial list_ports.comports()
    #     if system() == 'Linux' or system() == 'Darwin':
    #         return port.product
    #     else:
    #         return 'USB device'S-2026-2837


class WorkerSignals(QObject):
    error = Signal(object, str, str, str)
    result = Signal(np.ndarray, np.ndarray, np.ndarray, np.ndarray, int, str, bool)
    fullSweep = Signal(np.ndarray, np.ndarray)
    saveResults = Signal(np.ndarray, np.ndarray)
    sweepEnds = Signal(np.ndarray)
    stop_worker = Signal()


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
        logging.debug(f'{self.fn.__name__} thread running')
        self.fn(*self.args)
        logging.debug(f'{self.fn.__name__} thread stopped')


class Tiny(QObject):
    def __init__(self, usbPort, product, sigs, index, basic=False):
        super().__init__()
        self.usb = None
        self.usbPort = usbPort
        self.firmware = None
        self.name = None
        self.volts = 0
        self.sweeping = None
        self.threadRunning = False
        self.enabled = False
        self.basic = basic
        self.setSignals(sigs)
        self.setDevice(usbPort)
        self.id = index
        self.sn = None

    def setDevice(self, usbPort):
        self.setScale()
        # self.scanMemory = 10  # test, need to get this from the GUI
        try:
            self.usb = serial.Serial(usbPort, baudrate=576000)
            logging.debug(f'Serial port {usbPort} open: {self.usb.isOpen()}')
        except serial.SerialException:
            logging.info('Serial port exception. Is your username in the "dialout" group?')
            # popUp(QtTSA, 'Serial port exception', 'Ok', 'Critical')
            return
        self.clearBuffer()
        self.setCmdQ()
        self.setAbort(True)
        self.setTime()

    def setSignals(self, sigs):
        self.signals = WorkerSignals()
        self.signals.result.connect(sigs["result"])
        self.signals.saveResults.connect(sigs["save"])
        self.signals.sweepEnds.connect(sigs["ends"])
        self.signals.error.connect(sigs["error"])

    def setCmdQ(self):
        self.fifo = queue.SimpleQueue()
        self.fifoTimer = QTimer(self)
        self.fifoTimer.timeout.connect(self.usbSend)  # sends queued commands to tinySA (when not scanning)
        self.fifoTimer.start(500)  # 500mS

    def test(self, usbPort):  # tests tinySA comms
        if self.usb:
            for i in range(4):  # try 4 times to communicate with tinySA over USB serial
                version = self.version()
                if version is not None:
                    firmware = str.splitlines(version)
                    self.firmware = firmware[0].split('_')[-1]
                    # logging.info(f'{usbPort} test {i} reports {self.firmware}')
                    break
                else:
                    time.sleep(0.5)
            if version is not None:
                self.volts = self.battery()
                info = self.info()
                self.sn = self.serial_num().split(' ')[-1]
                self.name = str.splitlines(info)[0]
                logging.info(f'Connected {self.name} {self.sn} {self.firmware} on {usbPort}')
                return True
            else:
                return False

    def set_ctrls(self, rbw, attn, lna, spur):
        self.clearBuffer()
        self.attenuator(attn)
        self.lna(lna)
        self.spur(spur)
        self.setRBW(rbw)

    def close(self):
        if self.usb:
            self.usb.close()
            logging.debug(f'Close: Serial port {self.usbPort} open: {self.usb.isOpen()}')
            self.usb = None

    def sweepTimeout(self, frequencies, rbwTxt, spur):  # freqs are in Hz
        startF = frequencies[0]
        stopF = frequencies[-1]
        points = np.size(frequencies)
        if rbwTxt == "auto":
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

    def measurement(self, startF, stopF, points, rbw, depth, loop=True):  # run in separate thread
        if self.basic:
            rbw = np.clip(rbw, 3, 600)
            startF = np.clip(startF, 100000, 960000000)
            stopF = np.clip(stopF, 100000, 960000000)
        updateTimer = QElapsedTimer()
        self.threadRunning = True
        firstRun = True

        # create freq array here.  Non-Tiny devices may send list of freqs measured, so this preserves compatibility
        freq = np.linspace(startF, stopF, points, dtype=np.int64)
        levl = np.full(points, -140, dtype=float)
        maxl = np.full(points, -140, dtype=float)
        minl = np.full(points, 0, dtype=float)

        # if LNB:  # LNB / Transverter Mode
        #     startF, stopF = self.freqOffset(frequencies)

        # version = int(self.firmware[2])  # just the firmware version number

        # self.runTimer.start()  # debug
        # logging.debug(f'elapsed time = {self.runTimer.nsecsElapsed()/1e6:.3f}mS')  # debug

        updateTimer.start()  # used to trigger the signal that sends measurements to updateGUI()
        while self.sweeping:

            if loop:
                command = f'scanraw {int(startF)} {int(stopF)} {int(points)} 3\r'
            else:
                command = f'scanraw {int(startF)} {int(stopF)} {int(points)} 1\r'

            self.usb.timeout = 1  # self.sweepTimeout(frequencies)

            # firmware versions before 4.177 don't support auto-repeating scanraw so command must be sent each sweep
            if firstRun or not loop:
                try:
                    self.usb.write(command.encode())
                    self.usb.read_until(command.encode() + b'\n{')  # skip command echo
                    dataBlock = ''
                except serial.SerialException:
                    logging.info('serial port exception')
                    self.signals.error.emit(None, 'serial port exception', 'Ok', 'Critical')
                    self.sweeping = False
                    self.usb.reset_input_buffer()
                    break

            # read the measurement data from the tinySA
            for point in range(points):
                try:
                    dataBlock = (self.usb.read(3))  # read a block of 3 bytes of data
                    logging.debug(f'dataBlock: {dataBlock}\n')
                except serial.SerialException:
                    self.sweeping = False
                    self.signals.error.emit(None, 'serial port exception', 'Ok', 'Critical')
                    self.usb.reset_input_buffer()
                    break
                if dataBlock == b'}':  # from FW165 jog button press returns different value
                    logging.info('screen touched or jog button pressed')
                    self.signals.error.emit(None, 'screen touched or jog button pressed', 'Ok', 'Info')
                    self.sweeping = False
                    self.usb.reset_input_buffer()
                    break
                try:
                    c, data = struct.unpack('<' + 'cH', dataBlock)
                except struct.error:
                    logging.info('data error')
                    self.signals.error.emit(None, 'serial data error', 'Ok', 'Critical')
                    self.sweeping = False
                    self.usb.reset_input_buffer()
                    break
                levl[point] = (data / 32) - self.scale  # scale 0..4095 -> -128..-0.03 dBm

                if not self.sweeping:  # end without completing sweep
                    self.usb.reset_input_buffer()
                    break

                # If it's the final point of this sweep, set up for the next sweep
                if point == points - 1:
                    np.fmax(levl, maxl, out=maxl)  # compare current level with max and min
                    np.fmin(levl, minl, out=minl)  # and save them back on themselves
                    if loop:
                        if self.usb.read(2) != b'}{':  # the end of scan marker character is '}{'
                            logging.info('QtTinySA display is out of sync with tinySA frequency')
                            self.signals.error.emit(None, 'QtTinySA display out of sync', 'Ok', 'Critical')
                            self.sweeping = False
                            self.usb.reset_input_buffer()
                            break
                        firstRun = False
                timeElapsed = updateTimer.nsecsElapsed()  # how long this batch of measurements has been running, nS
                if timeElapsed/1e6 > 100:  # mS needs to be settings.ui.intervalBox.value():
                    # send the measurement data to router() in the Analyser class
                    self.signals.result.emit(freq, levl, maxl, minl, self.id, self.sn, False)
                    updateTimer.start()

            # also send the measurement data to router() at the end of each sweep
            self.signals.result.emit(freq, levl, maxl, minl, self.id, self.sn, True)
        try:
            self.usb.read(2)  # discard the command prompt that the tinySA sends when sweeping ends
            self.serialWrite('abort\r')
            self.usb.reset_input_buffer()
        except serial.SerialException:
            logging.info('tinySA serial comms failed')
            self.signals.error.emit(None, 'tinySA serial comms failed', 'Ok', 'Critical')
            self.usb.reset_input_buffer()
        self.threadRunning = False

    @Slot()
    def usbSend(self):
        while self.fifo.qsize() > 0:
            command = self.fifo.get(block=True, timeout=1)
            logging.debug(f' command = {command}')
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
        self.usb.reset_input_buffer()
        # while self.usb.inWaiting():
        #     self.usb.read_all()  # keep the serial buffer clean
        #     time.sleep(0.01)

    def pause(self):
        self.fifo.put('pause\r')

    def resume(self):
        self.fifo.put('resume\r')

    def reset(self):
        self.fifo.put('reset\r')

    def battery(self):
        vbat = self.serialQuery('vbat\r')
        vbat = vbat[:1] + "." + vbat[1:2] + "V"
        return vbat

    def setAbort(self, on=True):
        if on:
            command = 'abort on\r'
        else:
            command = 'abort off\r'
        self.fifo.put(command)

    def abort(self):
        self.serialWrite('abort\r')
        # self.clearBuffer()

    def version(self):
        version = self.serialQuery('version\r')
        # version = 'tinySA4_v1.4-199-gde12ba2'  # for testing ultra
        # version = 'tinySA_v1.4-175-g1419a93'   # for testing basic
        return version

    def info(self):
        info = self.serialQuery('info\r')
        return info

    def serial_num(self):
        sn = self.serialQuery('deviceid\r')
        return sn

    def spur(self, sType):
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

    def lna(self, on=True):
        if self.basic:  # it has no LNA
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
        command = f'rbw {rbw}\r'
        self.fifo.put(command)

    # @Slot()
    # def threadEnds(self):
    #     if int(self.firmware[2]) >= 177:  # the firmware version number
    #         self.serialWrite('abort\r')
    #     self.fifoTimer.start(500)  # start watching for commands

    # @Slot()
    # def saveFile(self, saveSingle=True):
    #     filebrowse.ui.saveProgress.setValue(0)
    #     SD = self.listSD()
    #     for i in range(len(SD.splitlines())):
    #         if not self.directory:  # have not already saved a file, or ask for folder was checked
    #             self.directory = QFileDialog.getExistingDirectory(caption="Select folder to save SD card file")
    #         if not self.directory:
    #             break
    #         if saveSingle:
    #             fileName = filebrowse.ui.listWidget.currentItem().text()  # the file selected in the list widget
    #         else:
    #             fileName = SD.splitlines()[i].split(" ")[0]
    #         with open(os.path.join(self.directory, fileName), "wb") as file:
    #             data = self.readSD(fileName)
    #             file.write(data)
    #         filebrowse.ui.saveProgress.setValue(int(100 * (i+1)/len(SD.splitlines())))
    #         filebrowse.ui.downloadInfo.setText(self.directory)  # show the path where the file was saved
    #         if filebrowse.ui.askForPath.isChecked():
    #             self.directory = None
    #         if saveSingle:
    #             filebrowse.ui.saveProgress.setValue(100)
    #             break

    # def fileShow(self):
    #     self.memF.seek(0, 0)  # set the memory buffer pointer to the start
    #     self.memF.truncate()  # clear down the memory buffer to the pointer
    #     filebrowse.ui.picture.clear()
    #     fileName = filebrowse.ui.listWidget.currentItem().text()
    #     self.clearBuffer()  # clear the tinySA serial buffer
    #     self.memF.write(self.readSD(fileName))  # read the file from the tinySA memory card and store in memory buffer
    #     if fileName[-3:] == 'bmp':
    #         pixmap = QPixmap()
    #         pixmap.loadFromData(self.memF.getvalue())
    #         filebrowse.ui.picture.setPixmap(pixmap)

class Nano(QObject):
    def __init__(self, usbPort, product, sigs, index):
        super().__init__()
        self.usb = None
        self.usbPort = usbPort
        self.product = product
        self.firmware = None
        self.sweeping = None
        self.setSignals(sigs)
        self.threadRunning = False
        self.enabled = False
        self.setDevice(usbPort)
        self.id = index
        self.sn = None

    def setDevice(self, usbPort):
        try:
            self.usb = serial.Serial(usbPort, baudrate=576000, timeout=3)
            logging.debug(f'Serial port {usbPort} open: {self.usb.isOpen()}')
        except serial.SerialException:
            logging.info('Serial port exception. Is your username in the "dialout" group?')
            # popUp(QtTSA, 'Serial port exception', 'Ok', 'Critical')
            return
        self.clearBuffer()
        self.setCmdQ()
        self.signals.error.emit(None, 'nanoVNA support is experimental.  Readings are never calibrated', 'Ok', 'Warn')

    def close(self):
        if self.usb:
            self.usb.close()
            logging.debug(f'Close: Serial port {self.usbPort} open: {self.usb.isOpen()}')
            self.usb = None

    def clearBuffer(self):
        self.usb.reset_input_buffer()
        # while self.usb.inWaiting():
        #     self.usb.read_all()  # keep the serial buffer clean
        #     time.sleep(0.01)

    @Slot()
    def usbSend(self):
        while self.fifo.qsize() > 0:
            command = self.fifo.get(block=True, timeout=1)
            logging.debug(f' command = {command}')
            self.serialWrite(command)

    def serialQuery(self, command):
        self.usb.write(command.encode())
        self.usb.read_until(command.encode() + b'\n')  # skip command echo
        response = self.usb.read_until(b'ch> ')  # until prompt
        return response[:-6].decode()  # remove prompt

    def serialWrite(self, command):
        logging.debug(f'serialWrite: command = {command}')
        self.usb.write(command.encode())
        self.usb.read_until(b'ch> ')  # skip command echo and prompt
 
    def setCmdQ(self):
        self.fifo = queue.SimpleQueue()
        self.fifoTimer = QTimer(self)
        self.fifoTimer.timeout.connect(self.usbSend)  # sends queued commands to Nano (when not scanning)
        self.fifoTimer.start(500)  # 500mS

    def setSignals(self, sigs):
        self.signals = WorkerSignals()
        self.signals.result.connect(sigs["result"])
        self.signals.saveResults.connect(sigs["save"])
        self.signals.sweepEnds.connect(sigs["ends"])
        self.signals.error.connect(sigs["error"])

    def test(self, usbPort):  # tests tinySA comms
        if self.usb:
            for i in range(4):  # try 4 times to communicate with tinySA over USB serial
                version = self.version()
                if version is not None:
                    self.firmware = version
                    break
                else:
                    time.sleep(0.5)
            if version is not None:
                info = self.info()
                self.sn = self.serial_num()
                self.name = str.splitlines(info)[0].split(' ')[-1]
                logging.info(f'Connected {self.name} {self.sn} v{self.firmware} on {usbPort}')
                return True
            else:
                return False

    def measurement(self, startF, stopF, points, rbw, depth, loop=True):  # run in separate thread
        # Nano points range capability is 11 to 301
        chunk_points = 100
        points = round(int(points/chunk_points)) * chunk_points  # set points to a whole multiple of chunk
        self.threadRunning = True
        
        # # create freq array here.  Non-Tiny devices may send list of freqs measured, so this preserves compatibility
        levl = np.full(points, -140, dtype=float)
        maxl = np.full(points, -140, dtype=float)
        minl = np.full(points, 0, dtype=float)
        
        # the freqs measured by the nano !== freq but near enough for testing
        freq = np.linspace(startF, stopF, points, dtype=np.int64)
        
        cr = b'\r'
        lf = b'\n'
        crlf = cr + lf
        prompt = b'ch> '
    
        while self.sweeping:
            # updateTimer.start()  # used to trigger the signal that sends measurements to updateGUI()
            
            for i in range(0, points, chunk_points):   
                start = freq[i]
                if i + chunk_points < points:
                    stop = freq[i + chunk_points]
                else:
                    stop = freq[-1]
                scan = f'scan {int(start)} {int(stop)} {int(chunk_points)} 7'  # opt 7 = freq, S11, S21
                
                # send scan command and options terminated by CR
                try:
                    self.usb.timeout = 0  # don't block
                    self.usb.write(scan.encode() + cr)
                    while self.usb.in_waiting == 0:
                        time.sleep(0.000001)
    
                    self.usb.timeout = None  # block until response is read
                    response = self.usb.read_until(scan.encode() + crlf)
                    chunk = self.usb.read_until(crlf + prompt)
                    chunk = chunk[:-len(crlf + prompt)].decode()
                    response = chunk.split()
                    response = np.loadtxt(response, delimiter=' ', dtype=float)
                    response = np.reshape(response, (-1, 5))
                except (serial.SerialException, ValueError):
                    self.sweeping = False
                    self.usb.reset_input_buffer()
                    break
                
                dB = 20 * np.log10(abs(response[0:, 1])) # RL from s11
                try:
                    if i + chunk_points <= points:
                        levl[i:i+chunk_points] = dB
                    else:
                        levl[i:-1] = dB[0:len(levl[i:-1])]
                    np.fmax(levl, maxl, out=maxl)  # compare current level with max and min
                    np.fmin(levl, minl, out=minl)  # and save them back on themselves
                    if not self.sweeping:
                        self.usb.reset_input_buffer()
                        break
                except ValueError:
                    # may happen on auto-restart after settings change
                    continue
                
                logging.debug(f'{levl}')
                
                #timeElapsed = updateTimer.nsecsElapsed()  # how long this batch of measurements has been running, nS
                #if timeElapsed/1e6 > 100:  # mS needs to be settings.ui.intervalBox.value():
                    # send the measurement data to router() in the Analyser class
                self.signals.result.emit(freq, levl, maxl, minl, self.id, self.sn, False)
                    #updateTimer.start()
            # also send the measurement data to router() at the end of each sweep
            self.signals.result.emit(freq, levl, maxl, minl, self.id, self.sn, True)
            self.usb.reset_input_buffer()
        self.threadRunning = False

    def set_ctrls(self, rbw, attn, lna, spur):
        self.clearBuffer()

    def version(self):
        version = self.serialQuery('version\r')
        return version

    def info(self):
        info = self.serialQuery('info\r')
        return info
    
    def serial_num(self):
        sn = self.serialQuery('SN\r')
        return sn
    
    # all commands

    # help:                lists all the registered commands
    # reset:               usage: reset
    # cwfreq:              usage: cwfreq {frequency(Hz)}
    # saveconfig:          usage: saveconfig
    # clearconfig:         usage: clearconfig {protection key}
    # data:                usage: data [array]
    # frequencies:         usage: frequencies
    # scan:                usage: scan {start(Hz)} {stop(Hz)} [points] [outmask]
    #                               points range 11 to 301
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

# data [0-6]: Dumps specific sweep data?
# 0:S11, 1:S21, 2:Directivity, 3:Source Match, 4:Reflection Tracking
# 5:Transmission Tracking, 6:Isolation


class Lime(QObject):
    def __init__(self, usbPort, product, sigs, index):
        super().__init__()
        self.setSignals(sigs)
        self.enabled = False
        self.id = index
        # soapy

    def setSignals(self, sigs):
        self.signals = WorkerSignals()
        self.signals.result.connect(sigs["result"])
        self.signals.saveResults.connect(sigs["save"])
        self.signals.sweepEnds.connect(sigs["ends"])

    def test(self):
        try:
            logging.info('pyvisa port')
        except: # some other exception
            logging.info('pyvisa port exception.')


class RTL(QObject):
    def __init__(self, usbPort, product, index):
        super().__init__()
        self.enabled = False
        self.id = index
        # soapy


class SiglentSA(QObject):
    def __init__(self, visaPort, product, sigs, index):
        super().__init__()
        self.setSignals(sigs)
        self.enabled = False
        self.id = index
        # pyvisa

    def setSignals(self, sigs):
        self.signals = WorkerSignals()
        self.signals.result.connect(sigs["result"])
        self.signals.saveResults.connect(sigs["save"])
        self.signals.sweepEnds.connect(sigs["ends"])