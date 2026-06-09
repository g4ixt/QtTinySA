#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jan 13 09:45:56 2026
"""

# Copyright 2026 Ian Jefferson G4IXT
# SPDX-License-Identifier: GPL-3.0-or-later

import os
import logging
import serial
import time
import queue
import struct
import numpy as np
from PySide6.QtCore import QObject, QElapsedTimer, QTimer, Signal, Slot, QRunnable, QThreadPool
# from PySide6.QtWidgets import QFileDialog
from serial.tools import list_ports
from datetime import datetime
from platform import system

threadpool = QThreadPool()

class USBdevice(QObject):
    stopped = Signal(bool)
    update_info = Signal(str, int, int, str)
    dev_enable = Signal(int, bool)

    def __init__(self):
        super().__init__()
        self.ports = []
        # self.firmware = None
        self.setSignals()
        self.setDevices()
        self.run_connect = False
        self.is_scanning = False
        self.loaded_files = 0

    def setSignals(self):
        self.signals = WorkerSignals()
        # dev_sigs forward the signals from devices to the router in QtTinySA.py (& are connected in there)
        self.dev_sigs = {"result": self.signals.result,
                         "save": self.signals.save,
                         "error": self.signals.error,
                         "progress": self.signals.progress}

    def setDevices(self):
        # self.cnx_count = 0  # used to count 'connected' devices, not 'enabled' devices
        self.dev_list = None
        self.rec_0 = Recorder(self.dev_sigs)
        self.rec_1 = Recorder(self.dev_sigs)
        self.rec_2 = Recorder(self.dev_sigs)
        self.rec_3 = Recorder(self.dev_sigs)
        # each instance of 'recorder' is used for a single spectrum analyser's measurement results
        self.recorders = (self.rec_0, self.rec_1, self.rec_2, self.rec_3)

    def probe(self):
        VID = (0x0483, 0x1d50, 0x04b4)  # 1155 tinySA/NanoVNA, limeSDR, NanoVNA V2 +4
        PID = (0x5740, 0x6108, 0x0008)  # 22336 tinySA/NanoVNA, limeSDR, NanoVNA V2 +4
        usbPorts = list_ports.comports()
        # detect devices as they connect
        for port in usbPorts:
            if port.vid in VID and port.pid in PID and port not in self.ports:
                self.ports.append(port)
                self.run_connect = True
                logging.debug(f'found {self.identify(port)} on {port.device}')

        # detect devices that have been turned off or lost contact
        for port in self.ports:
            if port not in usbPorts:
                self.disconnect(port.device)
                self.ports.remove(port)
        # only run connect() if new devices were found
        if self.run_connect:
            self.connect()

    def identify(self, port):
        # Windows returns no description information to pySerial list_ports.comports()
        if system() == 'Linux' or system() == 'Darwin':
            return port.product
        else:
            return 'tinySA4'

    def connect(self):
        # try to set USB connections to different hardware... need to check if it works in Windows now
        self.dev0 = self.dev1 = self.dev2 = self.dev3 = None
        self.dev_list = [self.dev0, self.dev1, self.dev2, self.dev3]  # all of which are initially set as None above
        self.run_connect = False
        for dev_id, port in enumerate(self.ports):
            description = self.identify(port)
            # iterate through the ports in the list, instantiate device classes and test serial comms
            if self.dev_list[dev_id] is None and len(self.ports) > dev_id:
                # instantiate a device class, replacing 'None' with the instance, in, e.g. self.dev2
                if description == "tinySA":
                    self.dev_list[dev_id] = Tiny(port.device, description, self.dev_sigs, dev_id, basic=True)
                if description == "tinySA4":
                    self.dev_list[dev_id] = Tiny(port.device, description, self.dev_sigs, dev_id, basic=False)
                if description == "LimeSDR-USB":
                    self.dev_list[dev_id] = Lime(port.device, description, self.dev_sigs, dev_id)
                if description == "NanoVnaPro Virtual ComPort":
                    self.dev_list[dev_id] = Nano(port.device, description, self.dev_sigs, dev_id)
                if description == "CDC-ACM Demo":
                    self.dev_list[dev_id] = Nano(port.device, description, self.dev_sigs, dev_id)
                # test using its specific commands and store results in its class instance
                test = self.dev_list[dev_id].test(port.device)
                if test is True:
                    self.set_sa_info(dev_id)
                    self.dev_enable.emit(dev_id, True)
                else:
                    logging.info(f'test of {port} failed')
                    self.update_info.emit('', dev_id, -1, '') # (name, id, sn, port)
                    self.dev_enable.emit(dev_id, False)

    def disconnect(self, usbPort):
        for i, device in enumerate(self.dev_list):
            if device and device.usbPort == usbPort:
                logging.info(f'{device.name} {device.sn} has disconnected from {device.usbPort}')
                self.update_info.emit('', i, -1, '') # (name, dev_id, sn, port
                self.dev_enable.emit(i, False)
                self.stop(restart=False)
                device.close()
                del device  # delete the class instance
                self.dev_list[i] = None

    def closePort(self):
        for device in self.dev_list:
            if device:
                device.close()

    def set_sa_info(self, i):
        try:
            name = self.dev_list[i].name
            sn = self.dev_list[i].sn
            usbPort = self.dev_list[i].usbPort
            self.update_info.emit(name, i, sn, usbPort)
        except AttributeError:
            return
    
    def set_rec_info(self, i):
        name = 'File ' + str(i)
        sn = self.recorders[i].sn
        file = self.recorders[i].file
        self.update_info.emit(name, i, sn, file)
        
    def renumber(self, num_enabled):
        # re-number the devices (id) so that only enabled ones are numbered below the enabled count
        count = 0
        for device in self.dev_list:
            if device is not None:
                if device.enabled:
                    device.id = count
                    count += 1
                else:
                    # set it to a value above the range of counted devices
                    device.id = num_enabled + 1
            
    def start(self, spectra, rbw, depth, maxF, interval, split, loop):
        for index, device in enumerate(self.dev_list):  # dev_list contains the device class instances
            if device is not None:
                if device.enabled:
                    device.usbSend()
                    startF = spectra[index].startF
                    stopF = spectra[index].stopF
                    points = spectra[index].points
                    logging.debug(f'usbInstr.start: startF={startF} stopF={stopF} pts={points}')
                    device.sa = Worker(device.measurement, startF, stopF, points, rbw, depth, maxF, interval, split, loop)
                    device.fifoTimer.stop()
                    device.sweeping = True
                    threadpool.start(device.sa)
                    self.is_scanning = True

    def controls(self, rbw, attn, lna, spur):
        for device in self.dev_list:
            if device is not None:
                device.set_ctrls(rbw, attn, lna, spur)  # device specific

    def stop(self, restart=False):
        if not self.dev_list:
            return
        for device in self.dev_list:  # dev_list contains the device class instances
            if device is not None:
                if device and device.sweeping:
                        device.sweeping = False  # the measurement threads keep looping if this is True
        for device in self.dev_list:
            if device and device.threadRunning:
                logging.debug('waiting for measurement thread to stop')
                time.sleep(0.1)
        self.is_scanning = False
        self.stopped.emit(restart)

    def read_file(self, file):  # run in separate thread to avoid blocking GUI for large files
        i = self.loaded_files
        recording = self.recorders[i]
        for j in range(i, 4):
            self.update_info.emit('', j, 0, '')
        recording.data_arr = np.load(file)
        file_name = file.split('/')[-1]
        logging.info(f'imported {file_name} for playback')
        recording.id = i
        recording.sn = self.recorders[i].data_arr[0, 0]
        recording.file = file_name
        self.set_rec_info(i)
        self.loaded_files += 1


class WorkerSignals(QObject):
    error = Signal(object, str, str, str)
    result = Signal(np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, int, int, float, bool, bool)
    save = Signal(np.ndarray, np.ndarray, int, int, bool)
    progress = Signal(int)
    # fullSweep = Signal(np.ndarray, np.ndarray)
    # sweepEnds = Signal(np.ndarray)
    # stop_worker = Signal()


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
    def __init__(self, usbPort, product, sigs, dev_id, basic=False):
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
        self.id = dev_id  # 0 to 3, where 0 = first device to connect on USB ports, etc.
        self.sn = 0

    def setDevice(self, usbPort):
        self.setScale()
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
        self.signals.save.connect(sigs["save"])
        # self.signals.sweepEnds.connect(sigs["ends"])
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
                logging.debug(f'Tiny {self.id} test {i} version = {version}')
                if version is not None:
                    firmware = str.splitlines(version)
                    self.firmware = firmware[0].split('_')[-1]
                    logging.debug(f'{usbPort} test {i} reports {self.firmware}')
                    break
                else:
                    time.sleep(0.5)
                    
            if version is not None:
                self.volts = self.battery()
                info = self.info()
                self.sn = int(self.serial_num().split(' ')[-1])
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

    def measurement(self, startF, stopF, points, rbw, depth, maxF, interval, split, loop=True):  # run in separate thread
        if self.basic:
            rbw = np.clip(rbw, 3, 600)
            maxF = min(960000000, maxF)
            startF = np.clip(startF, 100000, maxF)
            stopF = np.clip(stopF, 100000, maxF)
        else:
            startF = np.clip(startF, 100000, maxF)
            stopF = np.clip(stopF, 100000, maxF)
        updateTimer = QElapsedTimer()
        self.threadRunning = True
        firstRun = True
        # create freq array here.  Non-Tiny devices may send list of freqs measured, so this preserves compatibility
        freq = np.linspace(startF, stopF, points, dtype=np.int64)
        levl = np.full(points, -140, dtype=float)
        maxl = np.full(points, -140, dtype=float)
        minl = np.full(points, 0, dtype=float)
        buffer = np.full((depth, points), np.nan, dtype=float)  # used for waterfall and calculating averages

        # self.runTimer.start()  # debug
        # logging.debug(f'elapsed time = {self.runTimer.nsecsElapsed()/1e6:.3f}mS')  # debug

        updateTimer.start()  # used to trigger the signal that sends measurements to updateGUI()
        while self.sweeping:
            if loop:
                command = f'scanraw {int(startF)} {int(stopF)} {int(points)} 3\r'
            else:
                command = f'scanraw {int(startF)} {int(stopF)} {int(points)} 1\r'

            self.usb.timeout = 1  # should be from self.sweepTimeout(frequencies)

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
                    logging.info(f'data error {c} {data}')
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
                    
                    buffer = np.roll(buffer, 1, axis=0)
                    buffer[0] = levl
                    
                    if loop:
                        if self.usb.read(2) != b'}{':  # the end of scan marker character is '}{'
                            logging.info('QtTinySA display is out of sync with tinySA frequency')
                            self.signals.error.emit(None, 'QtTinySA display out of sync', 'Ok', 'Critical')
                            self.sweeping = False
                            self.usb.reset_input_buffer()
                            break
                        firstRun = False
                timeElapsed = updateTimer.nsecsElapsed()  # how long this batch of measurements has been running, nS
                if timeElapsed/1e6 > interval:  # GUI update interval mS
                    # send the measurement data to router() in the Analyser class
                    self.signals.result.emit(freq, levl, maxl, minl, buffer, self.id, self.sn, 0, split, False)
                    updateTimer.start()

            # also send the measurement data to router() at the end of each sweep
            self.signals.result.emit(freq, levl, maxl, minl, buffer, self.id, self.sn, 0, split, True)
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

class Nano(QObject):
    def __init__(self, usbPort, product, sigs, dev_id):
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
        self.id = dev_id  # this is the device_ID, 0 to 3, where 0 = first device to connect on USB ports, etc.
        self.sn = 0

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
        self.signals.save.connect(sigs["save"])
        # self.signals.sweepEnds.connect(sigs["ends"])
        self.signals.error.connect(sigs["error"])

    def test(self, usbPort):  # tests tinySA comms
        if self.usb:
            for i in range(4):  # try 4 times to communicate with tinySA over USB serial
                version = self.version()
                logging.debug(f'Nano test {i} version = {version}')
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

    def measurement(self, startF, stopF, points, rbw, depth, maxF, split, loop=True):  # run in separate thread
        # Nano points range capability is 11 to 301 so chop into subsets
        for i in range(11, 301, 1):
            rem = points - (i * int(points / i))
            if rem >= 11:  # because the final chunk must be >11 since Nano can't measure fewer
                subset = i
                break
        
        maxF = min(3 * 1e9, maxF)  # nanoVNA-FV2 max F is 3GHz
        startF = np.clip(startF, 50000, maxF)
        stopF = np.clip(stopF, 50000, maxF)

        self.threadRunning = True
        
        # # create freq array here.  Non-Tiny devices may send list of freqs measured, so this preserves compatibility
        levl = np.full(points, -140, dtype=float)
        maxl = np.full(points, -140, dtype=float)
        minl = np.full(points, 0, dtype=float)
        buffer = np.full((depth, points), np.nan, dtype=float)  # used for waterfall and calculating averages
        
        # the freqs measured by the nano !== linspace but near enough to start
        freq = np.linspace(startF, stopF, points, dtype=np.int64)
        
        cr = b'\r'
        lf = b'\n'
        crlf = cr + lf
        prompt = b'ch> '
    
        while self.sweeping:
            # updateTimer.start()  # used to trigger the signal that sends measurements to updateGUI()
            
            for i in range(0, points, subset):   
                start = freq[i]
                if i + subset < points:
                    stop = freq[i + subset]
                else:
                    stop = freq[-1]
                scan = f'scan {int(start)} {int(stop)} {int(subset)} 7'  # opt 7 = freq, S11, S21
                
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
                dB = 20 * np.log10(abs(response[0:, 1])) # Return Loss from s11
                try:
                    # append measurements to levl list and correct the freq array with actual nano freqs
                    if i + subset < points:
                        levl[i:i+subset] = dB
                        freq[i:i+subset] = response[0:, 0]
                    else:
                        levl[i:-1] = dB[0:len(levl[i:-1])]
                        freq[i:-1] = response[0:len(levl[i:-1]), 0]
                    np.fmax(levl, maxl, out=maxl)  # compare current level with max and min
                    np.fmin(levl, minl, out=minl)  # and save them back on themselves

                    if not self.sweeping:
                        self.usb.reset_input_buffer()
                        break
                except ValueError:
                    # may happen on auto-restart after settings change
                    logging.info('nano: value error')
                    continue
                
                logging.debug(f'levl = {np.shape(levl)} freq = {np.shape(freq)}')
                self.signals.result.emit(freq, levl, maxl, minl, buffer, self.id, self.sn, 0, split, False)

            buffer = np.roll(buffer, 1, axis=0)
            buffer[0] = levl
            # also send the measurement data to router() at the end of each sweep
            self.signals.result.emit(freq, levl, maxl, minl, buffer, self.id, self.sn, 0, split, True)
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
        sn = self.serialQuery('SN\r')[:-8]
        return int(sn)
    
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
    def __init__(self, usbPort, product, sigs, dev_id):
        super().__init__()
        self.setSignals(sigs)
        self.enabled = False
        self.id = dev_id  # this is the device_ID, 0 to 3, where 0 = first device to connect on USB ports, etc.
        # soapy

    def setSignals(self, sigs):
        self.signals = WorkerSignals()
        self.signals.result.connect(sigs["result"])
        self.signals.save.connect(sigs["save"])
        # self.signals.sweepEnds.connect(sigs["ends"])

    def test(self):
        try:
            logging.info('pyvisa port')
        except: # some other exception
            logging.info('pyvisa port exception.')


class RTL(QObject):
    def __init__(self, usbPort, product, dev_id):
        super().__init__()
        self.enabled = False
        self.id = dev_id  # this is the device_ID, 0 to 3, where 0 = first device to connect on USB ports, etc.
        # soapy


class SiglentSA(QObject):
    def __init__(self, visaPort, product, sigs, dev_id):
        super().__init__()
        self.setSignals(sigs)
        self.enabled = False
        self.id = dev_id  # this is the device_ID, 0 to 3, where 0 = first device to connect on USB ports, etc.
        # pyvisa

    def setSignals(self, sigs):
        self.signals = WorkerSignals()
        self.signals.result.connect(sigs["result"])
        self.signals.save.connect(sigs["save"])
        # self.signals.sweepEnds.connect(sigs["ends"])


class Recorder(QObject):
    '''data_array structure
            row 0 col 0 = hardware serial num last 8 digits
            rows: 0=measurement point frequencies from col 1; row 1 onwards from col 1=dBm readings
            cols: 0=times from row 1 onwards; 1 onwards from row 1=points dBm values
       10e6 fields in array gives ~32MB mem/file size and ~6h for 4 devices at 101 points'''
    def __init__(self, sigs):
        super().__init__()
        self.sweeping = False
        self.recording = False
        self.threadRunning = False
        self.enabled = False
        self.setSignals(sigs)
        self.data_arr = np.full((2,2), np.nan, dtype=np.float32)
        self.row_count = 0
        self.MAX_FIELDS = 10e6
        self.dev_num = 0
        self.id = 0  # 0 to 3, where 0=player for the first measurement recording file that was loaded, etc.
        self.sn = 0
        self.name = ''
        self.rec_time = 0
        self.speed = 1
        
    def setSignals(self, sigs):
        self.signals = WorkerSignals()
        self.signals.result.connect(sigs["result"])
        self.signals.save.connect(sigs["save"])
        self.signals.progress.connect(sigs["progress"])
   
    def player(self, depth, dev_id, interval, target, play_clicked, split):
        '''runs in a thread, sending data to the router from a file loaded into self.data_arr'''
        self.threadRunning = True
        scans = np.shape(self.data_arr)[0] - 1
        points = np.shape(self.data_arr)[1] - 1
        freq = self.data_arr[0, 1:]  # freqs are in row 0 from col 1 onwards
        levl = np.full(points, -140, dtype=float)
        times = self.data_arr[1:, 0]  # time stamps are in col 0 from row 1 onwards
        maxl = np.full(points, -140, dtype=float)
        minl = np.full(points, 0, dtype=float)
        buffer = np.full((depth, points), None, dtype=float)  # used for waterfall and calculating averages 
        updateTimer = QElapsedTimer()
        sweep_time = 1
        if play_clicked:
            target_row = scans
            row = int(scans * target/100)
        else:
        # the slider is controlling playback
            target_row = int(scans * target/100)
            row = target_row - 1
        if scans > 1:
            sweep_time = float(times[1] - times[0])
        buffer_start = max(0, target_row - depth)
        buffer = self.data_arr[buffer_start:target_row, 1:]

        updateTimer.start()
        while self.sweeping and row < target_row:
            if row > 1:
                sweep_time = float(times[row] - times[row - 1])
            timestamp = float(times[row]) - sweep_time  # time is stamped at the end of a sweep
            levl = self.data_arr[row, 1:]  # first column in the array is a timestamp
            np.fmax(levl, maxl, out=maxl)  # compare current level with max and min
            np.fmin(levl, minl, out=minl)  # and save them back on themselves
            if play_clicked:
                buffer = np.roll(buffer, 1, axis=0)
                buffer[0] = levl
                time.sleep(self.speed*sweep_time)
            else:
                time.sleep(interval / 1e3)  # interval is in mS
            timestamp += sweep_time
            timeElapsed = updateTimer.nsecsElapsed() / 1e6  # how long the player has been running, mS
            if timeElapsed >= interval:
            # send the measurement data to router() in the Analyser class
                self.signals.result.emit(freq, levl, maxl, minl, buffer, self.id, self.sn, timestamp, split, False)
                updateTimer.start()
            if not self.sweeping:
                break
            row += 1
        self.sweeping = False
        self.threadRunning = False   

    def record(self, freq, levl, ser_num):  # called once each sweep by a signal from router() in analyser
        logging.debug(f'record: ser_num = {ser_num}')
        if self.row_count == 0:
            # set row 0 col 0 to serial num and col 1 onward to frequency values for each point
            self.data_arr[0, 0] = ser_num
            self.data_arr[0, 1:] = freq
        self.row_count += 1
        if self.row_count + 1 < np.size(self.data_arr, axis=0) and self.recording:
            self.data_arr[self.row_count, 0] = time.time()
            self.data_arr[self.row_count, 1:] = levl
        
    def save_recording(self, folder):
            # save a copy of arr to file; omit all-NaN rows to minimise file size for short recordings
            logging.info(f'saving recording from {self.sn} dev{self.id} at row {self.row_count}')
            timestamp = time.strftime('%Y-%m-%d-%H%M%S')
            file_name = str(timestamp + '_s' + str(self.sn) + '_d' + str(self.id))
            file_name = os.path.join(folder, file_name)
            nan_rows = np.isnan(self.data_arr).all(axis=1)
            copy_arr = self.data_arr[~nan_rows].copy()
            saver = Worker(self.save_npy, file_name, copy_arr)  # large files can take 5s or more to save
            threadpool.start(saver)
            self.row_count = 0
            self.reset_arr()
    
    def save_npy(self, file_name, data_arr):
        # saving as text would take 5x as long and have a 5x larger file
        np.save(file_name, data_arr, allow_pickle=False)
        
    def reset_arr(self):
        self.data_arr = np.full_like(self.data_arr, np.nan, dtype=np.float64)
           
    def configure(self, points, dev_id, dev_count):
        self.dev_id = dev_id
        rows = int(self.MAX_FIELDS / (dev_count * points))  # give a file size of ~32MB for 1 device
        self.data_arr = np.full((rows, points+1), np.nan, dtype=np.float64)
