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

    def __init__(self):
        super().__init__()
        self.ports = []
        # self.firmware = None
        self.setSignals()
        self.run_connect = False
        self.is_scanning = False

    def setSignals(self):
        self.signals = WorkerSignals()
        # dev_sigs forward the signals from devices to the analyser class in QtTinySA.py (& are connected in there)
        self.dev_sigs = {"result": self.signals.result,
                         "save": self.signals.saveResults,
                         "ends": self.signals.sweepEnds}

    def probe(self):
        VID = (0x0483, 0x1d50)  # 1155 tinySA/NanoVNA, limeSDR
        PID = (0x5740, 0x6108)  # 22336 tinySA/NanoVNA, limeSDR
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
                logging.info(f'{port.product} has disconnected from {port.device}')
                self.disconnect(port.device)
                self.ports.remove(port)

        if self.run_connect:
            self.connect()

    def connect(self):  # imperfect, need to handle test fails and different devices reconnecting
        # try to set USB connections to different hardware....... need to check if it works in Windows now
        self.dev0 = self.dev1 = self.dev2 = self.dev3 = None
        self.dev_list = [self.dev0, self.dev1, self.dev2, self.dev3]  # all of which are set as None
        self.run_connect = False

        for index, port in enumerate(self.ports):
            dev_type = {"tinySA": Tiny(port.device, port.product, self.dev_sigs, basic=True),
                        "tinySA4": Tiny(port.device, port.product, self.dev_sigs, basic=False),
                        "LimeSDR-USB": Lime(port.device, port.product, self.dev_sigs),
                        "NanoVnaPro Virtual ComPort": Nano(port.device, port.product, self.dev_sigs)}

            # iterate through the usb ports with appropriate VID/PID devices connected and test serial comms
            if self.dev_list[index] is None and len(self.ports) > index:
                self.dev_list[index] = dev_type[port.product]  # instantiate device class & write instance to list
                hardware = self.dev_list[index]
                hardware.test(port.device)  # invoke the test function of the device class instance

        #  self.dev_list[] now contains up to 4 device class instance objects, or None values.

    def disconnect(self, usbPort):  # imperfect
        for device in self.dev_list:
            if device and device.usbPort == usbPort:
                logging.info('removing disconnected device instance')
                device.close()
                del device
                device = None

    def closePort(self):
        for device in self.dev_list:
            if device:
                device.close()

    def start(self, startF, stopF, points, rbw, depth, split, loop):
        freq_list = self.multi_split(startF, stopF, points, rbw, split)
        for index, device in enumerate(self.dev_list):  # dev_list contains the device class instances
            if device is not None:
                device.usbSend()
                startF = freq_list[index][0]
                stopF = freq_list[index][1]
                points = freq_list[index][2]
                device.sa = Worker(device.measurement, startF, stopF, points, rbw, loop)
                device.fifoTimer.stop()
                device.sweeping = True
                threadpool.start(device.sa)
                self.is_scanning = True

    def controls(self, rbw, attn, lna, spur):
        for device in self.dev_list:
            if device is not None:
                device.set_ctrls(rbw, attn, lna, spur)  # device specific

    def multi_split(self, startF, stopF, points, rbw, split):
        # splits the scan startF/stopF/points across multiple devices, or returns a list of identical tuples
        count = np.count_nonzero(self.dev_list)
        split_list = []
        split_points = int(points/count)
        split_span = int((stopF - startF)/count)
        for i in range(count):
            if not split:
                split_list.append((startF, stopF, points))
            else:
                if i == 0:
                    startF = startF
                    stopF = startF + split_span
                else:
                    startF = startF + split_span
                    stopF = startF + split_span
                split_list.append((startF, stopF, split_points))
        return split_list

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
        self.stopped.emit(restart)

    # def identify(self, port):
    #     # Windows returns no information to pySerial list_ports.comports()
    #     if system() == 'Linux' or system() == 'Darwin':
    #         return port.product
    #     else:
    #         return 'USB device'S-2026-2837


class WorkerSignals(QObject):
    error = Signal(str)
    result = Signal(object, np.ndarray, np.ndarray, np.ndarray, np.ndarray, bool)
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
    def __init__(self, usbPort, product, sigs, basic=False):
        super().__init__()
        self.usb = None
        self.usbPort = usbPort
        self.firmware = None
        self.sweeping = None
        self.threadRunning = False
        self.basic = basic
        self.setSignals(sigs)
        self.setDevice(usbPort)

    def setDevice(self, usbPort):
        self.setScale()
        # self.scanMemory = 10  # test, need to get this from the GUI
        try:
            self.usb = serial.Serial(usbPort, baudrate=576000)
            logging.info(f'Serial port {usbPort} open: {self.usb.isOpen()}')
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

    def setCmdQ(self):
        self.fifo = queue.SimpleQueue()
        self.fifoTimer = QTimer(self)
        self.fifoTimer.timeout.connect(self.usbSend)  # sends queued commands to tinySA (when not scanning)
        self.fifoTimer.start(500)  # 500mS

    def test(self, usbPort):  # tests tinySA comms and initialise if found
        self.is_tinySA = False
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
                self.is_tinySA = True
                sd_card = str.splitlines(self.listSD())
                if "ID.bmp 307322" in sd_card:
                    logging.info(f'ID found = {sd_card[-1]}')

                if firmware[0] == 'tinySA4' and float(self.firmware[1][-3:] + self.firmware[2]) < 1.4177:
                    logging.info('for fastest scan speed, upgrade firmware to v1.4-177 or later')

                if self.firmware[1][0] == "v":
                    return self.firmware  # setForDevice needs this info
                else:
                    logging.info(f'{usbPort} test found unexpected firmware {firmware}')
            else:
                logging.info(f'firmware {firmware} on {usbPort} is not a tinySA')

    def set_ctrls(self, rbw, attn, lna, spur):
        self.attenuator(attn)
        self.lna(lna)
        self.spur(spur)
        self.setRBW(rbw)

    def close(self):
        if self.usb:
            self.usb.close()
            logging.info(f'Close: Serial port {self.usbPort} open: {self.usb.isOpen()}')
            self.usb = None

    def clearBuffer(self):
        # self.usb.timeout = 1
        while self.usb.inWaiting():
            self.usb.read_all()  # keep the serial buffer clean
            time.sleep(0.01)

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
        sweepCount = 0
        updateTimer = QElapsedTimer()
        self.threadRunning = True
        firstRun = True

        # create freq array here.  Non-Tiny devices may send list of freqs measured, so this preserves compatibility
        freq = np.linspace(startF, stopF, points, dtype=np.int64)
        levl = np.full(points, -140, dtype=float)
        maxl = np.full(points, -140, dtype=float)
        minl = np.full(points, 0, dtype=float)
        wfall = np.full((int(depth), points), None, dtype=float)  # used by waterfall and 3D graph

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
                levl[point] = (data / 32) - self.scale  # scale 0..4095 -> -128..-0.03 dBm

                if not self.sweeping:  # end without completing sweep
                    break

                # If it's the final point of this sweep, set up for the next sweep
                if point == points - 1:
                    np.fmax(levl, maxl, out=maxl)  # compare current level with max and min
                    np.fmin(levl, minl, out=minl)  # and save them back on themselves
                    if loop:
                        if self.usb.read(2) != b'}{':  # the end of scan marker character is '}{'
                            logging.info('QtTinySA display is out of sync with tinySA frequency')
                            self.sweeping = False
                            break
                        sweepCount += 1
                        firstRun = False
                timeElapsed = updateTimer.nsecsElapsed()  # how long this batch of measurements has been running, nS
                if timeElapsed/1e6 > 100:  # mS needs to be settings.ui.intervalBox.value():
                    # send the measurement data to router() in the Analyser class
                    self.signals.result.emit(self.usbPort, freq, levl, maxl, minl, False)
                    updateTimer.start()

            # sweep ended: update the markers on the trace this device provides, via the router()
            # timeNow = time.time()
            wfall = np.roll(wfall, 1, axis=0)  # roll it round 1 row & write levl to row 0
            wfall[0] = levl
            self.signals.result.emit(self.usbPort, freq, levl, maxl, minl, True)

        self.usb.read(2)  # discard the command prompt that the tinySA sends when sweeping ends
        self.threadRunning = False
        self.serialWrite('abort\r')
        self.clearBuffer()

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
        # self.clearBuffer()

    def version(self):
        version = self.serialQuery('version\r')
        # version = 'tinySA4_v1.4-199-gde12ba2'  # for testing ultra
        # version = 'tinySA_v1.4-175-g1419a93'   # for testing basic
        return version

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
    def __init__(self, usbPort, product, sigs):
        super().__init__()
        self.usb = usbPort
        self.product = product
        self.setSignals(sigs)
        # self.device_id = self.ports.index(usbPort)
        # nano vna

    def setSignals(self, sigs):
        self.signals = WorkerSignals()
        self.signals.result.connect(sigs["result"])
        self.signals.saveResults.connect(sigs["save"])
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
        self.signals.saveResults.connect(sigs["save"])
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
        self.signals.saveResults.connect(sigs["save"])
        self.signals.sweepEnds.connect(sigs["ends"])