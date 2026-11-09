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
import usb.core
from PySide6.QtCore import QObject, QElapsedTimer, QTimer, Signal, Slot, QRunnable, QThreadPool
from serial.tools import list_ports
from datetime import datetime
from platform import system

from modules.utility import FakePortInfo

SOAPY = True
try:
    from modules.soapy_receiver_mp import SoapyReceiverProcess
    from modules.soapy_receiver_mp import calculate_fft_size
    from modules.soapy_receiver_mp import list_devices
except ImportError:
    SOAPY = False
    logging.info('SoapySDR Python bindings not found. Install if required.')

threadpool = QThreadPool()

SERIAL_HW = {(0x0483, 0x5740): 'tinySA',
             (0x04b4, 0x0008): 'Nano-VNA',
             (0x1781, 0x0a44): 'tinySA4'}

SOAPY_HW = {(0x0bda, 0x2838): 'RTL-SDR',
            (0x1d50, 0x6108): 'LimeSDR-USB',
            (0x0403, 0x601f): 'LimeSDR-Mini',
            (0x1d50, 0x6089): 'HackRF-One'}

class USBdevice(QObject):
    stopped = Signal(bool)
    update_info = Signal(int, str, str, bool)  # number, description, tooltip, state
    dev_enable = Signal(int, bool)

    def __init__(self, sdr_enable):
        super().__init__()
        self.ports = []
        self.setSignals()
        self.setDevices()
        self.run_connect = False
        self.is_scanning = False
        self.loaded_files = 0
        self.num_enabled = 0
        self.soapy_devs = []
        self.blacklist = []
        self.probe_ports = QTimer()
        self.probe_ports.timeout.connect(self.probe)
        self.sdr_enable = sdr_enable

    def setSignals(self):
        self.signals = WorkerSignals()

        # dev_sigs forward the signals from devices to the router in QtTinySA.py (& are connected in there)
        self.dev_sigs = {"result": self.signals.result,
                         "save": self.signals.save,
                         "error": self.signals.error,
                         "progress": self.signals.progress,
                         "status": self.signals.status}

    def setDevices(self):
        self.devices = []

        # each instance of 'recorder' is used for a single spectrum analyser's measurement results
        self.rec_0 = Recorder(self.dev_sigs)
        self.rec_1 = Recorder(self.dev_sigs)
        self.rec_2 = Recorder(self.dev_sigs)
        self.rec_3 = Recorder(self.dev_sigs)
        self.recorders = (self.rec_0, self.rec_1, self.rec_2, self.rec_3)

    def probe(self):
        self.probe_serial_connects()
        self.probe_serial_disconnects()
        
        if self.sdr_enable.isChecked():
            self.probe_soapy_connects()
        

    def probe_serial_connects(self):
        # detect usb-serial (tinySA / nanoVNA) devices as they connect to the usb bus
        usbPorts = list_ports.comports()
        for port in usbPorts:
            key = (port.vid, port.pid)
            if key in SERIAL_HW and port not in self.ports:
                # make a list of recognised devices
                logging.info(f'Found serial device {hex(key[0])} {hex(key[1])} {SERIAL_HW[key]} on {port.device}')
                self.ports.append(port)
                self.connect(port, driver=None)
           
    def probe_serial_disconnects(self):
        # detect usb-serial port devices that have been turned off or lost contact
        usbPorts = list_ports.comports()
        for port in self.ports:
            key = (port.vid, port.pid)
            if key in SERIAL_HW and port not in usbPorts:
                self.disconnect(port.device)
                self.ports.remove(port)
                
                # update the GUI info for all devices
                self.clear_all_gui_dev_info()
                if self.devices:
                    for device in self.devices:
                        port_num = self.port_to_num(device.usbPort)
                        self.set_gui_dev_info(device, port_num)
        
    def probe_soapy_connects(self):
        '''soapy decices communicate using three multiprocessing queues:
            data_queue, calibration_queue, event_queue.
            There is also a command queue but that is used internally by the soapy rx process.'''
        # detect and connect soapy devices
        new_device = False
        for device in usb.core.find(find_all=True):
            vid = device.idVendor
            pid = device.idProduct
            if (vid, pid) in SOAPY_HW and (vid, pid) not in self.blacklist:
                # if a device has already failed, it's blacklisted so we don't keep logging failure messages
                # need to add a way to show/reset them
                try:
                    dsn = device.serial_number
                    details = (SOAPY_HW[(vid, pid)], dsn)
                    if details in self.soapy_devs:
                        continue
                    else:
                        self.soapy_devs.append((SOAPY_HW[(vid, pid)], dsn))
                        logging.info(f'usb.core found {SOAPY_HW[(vid, pid)]} serial number {dsn}')
                        new_device = True
                except ValueError:
                    logging.info(f'{SOAPY_HW[(vid, pid)]} failed serial number query and was blacklisted')
                    self.blacklist.append((vid, pid))
                    continue
        if new_device:
            if SOAPY:  # and if "sdr_enable" is checked in 'preferences'
                devices = list_devices()
                for device in devices:
                    driver = device.args.get("driver", "unknown")
                    name = device.args.get("name", driver)
                    label = device.args.get("label", driver)
                    sn = device.args.get("serial", "0")
                    dev_id = name + "_" + sn
                    port = FakePortInfo(device=dev_id, name=name, description=label, serial_number=sn, product=name)
                    if port not in self.ports:
                        logging.info(f'Soapy recognised {name} serial number {sn} with {driver} driver')
                        # logging.info(f'Attempting to connect {name}')
                        self.ports.append(port)
                        self.connect(port, driver=driver)
            else:
                logging.info('Soapy module not installed or it is disabled in preferences')
        
    def probe_soapy_disconnects(self):
        x = 0
        
    def identify(self, port):
        # Windows returns no description information to pySerial list_ports.comports()
        if system() == 'Linux' or system() == 'Darwin' or port.product in ['LimeSDR-USB', 'LimeSDR Mini']:
            return port.product
        else:
            logging.info('OS is windows')
            return 'tinySA4'

    def port_to_num(self, usbPort):
        # port number sets the order devices are used and the gui list order
        port_names = [port.device for port in self.ports]
        logging.debug(f'port names = {port_names}')
        port_num = port_names.index(usbPort)
        return port_num

    def connect(self, port, driver):    
        '''set up the specific device measurement function depending on the description from the usb port probe'''
        
        if len(self.devices) == 4:
            logging.info('connect: cannot connect more than 4 devices')
            return

        port_num = self.port_to_num(port.device)
        description = self.identify(port)
        sa = None

        # instantiate an appropriate device class and append it to the self.devices list
        if description == "tinySA":
            sa = Tiny(port.device, description, self.dev_sigs, basic=True)
        if description == "tinySA4":
            sa = Tiny(port.device, description, self.dev_sigs, basic=False)
        if description in ["NanoVnaPro Virtual ComPort", "CDC-ACM Demo"]:
            sa = Nano(port.device, description, self.dev_sigs)
        
        if description in ['LimeSDR-USB', 'LimeSDR Mini']:
                sa = SoapySDRDevice(port.device, description, driver, self.dev_sigs)
        if description in ['hackrf']:
                sa = SoapySDRDevice(port.device, description, driver, self.dev_sigs)    
        if description in ['rtlsdr']:
                sa = SoapySDRDevice(port.device, description, driver, self.dev_sigs)

        if sa is not None:
            self.devices.append(sa)
            # wait for device power-up; test using its unique commands; store values in its class instance
            time.sleep(0.1)
            test = sa.test(port.device)
            if test is True:
                self.set_gui_dev_info(sa, port_num)
            else:
                logging.info(f'test of {port} failed')
                self.disconnect(port)
                self.devices.remove(sa)
        else:
            # take this inadequate step and fix it properly later
            logging.warning(f'device {description} is not supported')

    def disconnect(self, usbPort):
        for device in self.devices:
            if device.usbPort == usbPort:
                logging.info(f'{device.name} {device.sn} has disconnected from {device.usbPort}')
                self.stop(restart=False)  # stop running thread; doesn't return until thread has stopped
                device.close()
                port_num = self.port_to_num(usbPort)
                self.clear_gui_dev_info(port_num)
                self.devices.remove(device)  # delete the class instance from the list
                break

    def closePort(self):
        if self.devices:
            for device in self.devices:
                    device.close()
    
    def set_gui_dev_info(self, device, port_num):
        state = device.enabled
        sn = str(device.sn)
        description = device.name[:8]
        port = device.usbPort
        # tooltip = device.name + '\n' + 's/n ' + sn + '\n' + port
        tooltip = device.name + '\n' + 's/n ' + sn + '\n'
        self.update_info.emit(port_num, description, tooltip, state)

    def clear_gui_dev_info(self, port_num):
        self.update_info.emit(port_num, '', '', False)
        
    def clear_all_gui_dev_info(self):
        for i in range(4):
            self.update_info.emit(i, '', '', False)

    def toggle_dev_state(self, port_num, state):
        port = self.ports[port_num].device
        if self.devices:
            for device in self.devices:
                if device.usbPort == port:
                    device.enabled = state
            self.count_enabled()
        if self.is_scanning:
            self.stop(restart=True)

    def count_enabled(self):
        count = 0
        if self.devices:
            for device in self.devices:
                if device.enabled: count += 1
        self.num_enabled = count

    def set_rec_info(self, i):
        name = 'File ' + str(i)
        sn = self.recorders[i].sn
        file = self.recorders[i].file
        # self.update_info.emit(name, i, sn, file)

    def start(self, spectra, rbw, rbw_auto, depth, maxF, interval, split, sdr_settings, loop):
        if self.devices:
            # self.devices contains the device class instances but not necessarily in port order
            for device in self.devices:
                if device.enabled:
                    indx = self.port_to_num(device.usbPort)  # fetch the device number from its associated port
                    device.usbSend()
                    startF = spectra[indx].startF
                    stopF = spectra[indx].stopF
                    points = spectra[indx].points
                    logging.debug(f'usbInstr.start: startF={startF} stopF={stopF} pts={points}')
                    device.sa = Worker(device.measurement, startF, stopF, points,
                                       rbw, rbw_auto, depth, maxF, interval, split, sdr_settings, loop)
                    device.fifoTimer.stop()
                    device.sweeping = True
                    threadpool.start(device.sa)
                    self.is_scanning = True

    def controls(self, rbw, rbw_auto, attn, lna, spur, sdr_settings):
        for device in self.devices:
            device.set_ctrls(rbw, rbw_auto, attn, lna, spur, sdr_settings)  # device specific

    def stop(self, restart=False):
        if not self.devices:
            return
        for device in self.devices:  # devices contains the device class instances
                if device.sweeping:
                        device.sweeping = False  # the measurement threads keep looping if this is True
        for device in self.devices:
            if device.threadRunning:
                logging.debug('waiting for measurement thread to stop')
                time.sleep(0.1)
        self.is_scanning = False
        self.stopped.emit(restart)

    def load_player(self, file):  # run in separate thread to avoid blocking GUI for large files
        # create a player for this file
        player = FilePlayer(self.dev_sigs)

        # load the file into the data array of the new player's file server
        input_data = np.load(file)
        player.data_arr = np.nan_to_num(input_data, nan=np.nan, posinf=np.nan, neginf=np.nan) # eliminate infinites
        file_name = file.split('/')[-1]
        player.sn = player.data_arr[0, 0]
        player.name = file_name[8:15]
        port_name = 'F' + str(self.loaded_files)
        dev = file_name
        port = FakePortInfo(device=port_name, description=file_name, serial_number=player.sn)
        player.usbPort = port.device
        logging.info(f'{port_name} {file_name} from {player.sn} loaded as player {player.name}')
        
        if self.loaded_files == 4:
            old_player = self.devices.pop(0)  # pop the old player's instance from the start of the device list
            if old_player.sweeping:
                old_player.sweeping = False
            old_port = self.ports.pop(0)
        
        self.devices.append(player)  # append the new player's instance to the device list
        self.ports.append(port)

        # update the GUI info for all playback files
        self.clear_all_gui_dev_info()
        if self.devices:
            for device in self.devices:
                port_num = self.port_to_num(device.usbPort)
                self.set_gui_dev_info(device, port_num)
        if self.loaded_files < 4:
            self.loaded_files += 1

    def close_player(self):
        self.loaded_files = 0
        self.clear_all_gui_dev_info()


class WorkerSignals(QObject):
    error = Signal(object, str, str, str)
    result = Signal(np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, str, int, str, bool, bool)
    save = Signal(np.ndarray, np.ndarray, int, int, bool)
    progress = Signal(int)
    status = Signal(str)
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


class Buffer():
    """Provides a fast-updating buffer array to the measurement classes for waterfall/3D and trace averaging"""
    
    def __init__(self, rows=500, cols=2048):
        self.rows = rows
        self.cols = cols

        # 1. Allocate the static memory block once
        # self.array = np.zeros((self.rows, self.cols), dtype=np.float32)  # needs to be nan
        self.array = np.full((depth, fft_size), np.nan, dtype=np.float32)

        # 2. Track where the NEXT row should be written
        self.indx = 0

    def write_row(self, fft_magnitude):
        """np.roll has overhead rolling maybe 1e6 data items; this writes a single row of maybe 30k items
           and we keep track of where we are with an index pointer instead of top row always latest"""

        # Overwrite the row at the current index (Fast, zero-allocation)
        self.array[self.indx, :] = fft_magnitude

        # Keep a pointer to the next row, wrapping back to 0 if it hits the end using modulo division
        self.indx = (self.indx + 1) % self.rows

    def sort_chrono(self):
        """Roll the array to time order using the index. Called when needed to re-draw.
            np.roll is fine here because it only runs when rendering, not on every SDR sample"""
        return np.roll(self.array, -self.indx, axis=0)

    def resize(self, new_rows, new_cols):
        """Safely handle dynamic size changes when scan settings change."""
        if (new_rows, new_cols) != (self.rows, self.cols):
            self.rows = new_rows
            self.cols = new_cols
            self.array = np.zeros((self.rows, self.cols), dtype=np.float32)  # needs to be nan
            self.indx = 0

    def average(self, alpha):
        x = alpha
        return # something


class Tiny(QObject):
    def __init__(self, usbPort, product, sigs, basic=False):
        super().__init__()
        self.usb = None
        self.usbPort = usbPort
        self.firmware = None
        self.name = None
        self.volts = 0
        self.sweeping = None
        self.threadRunning = False
        self.enabled = True
        self.basic = basic
        self.setSignals(sigs)
        self.setDevice(usbPort)
        self.sn = 0

    def setDevice(self, usbPort):
        self.setScale()
        try:
            self.usb = serial.Serial(usbPort, baudrate=576000)
            logging.debug(f'Serial port {usbPort} open: {self.usb.isOpen()}')
        except serial.SerialException:
            logging.info('Serial port exception. Is your username in the "dialout" group?')
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
                logging.debug(f'TinySA on {usbPort} test {i} version = {version}')
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
                logging.info(f'Connected {self.name} sn={self.sn} fw={self.firmware}')
                self.enabled = True
                return True
            else:
                self.enabled = False
                return False

    def set_ctrls(self, rbw, rbw_auto, attn, lna, spur, sdr_settings):
        self.clearBuffer()
        self.attenuator(attn)
        self.lna(lna)
        self.spur(spur)
        if rbw_auto:
            self.setRBW('auto')
        else:
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

    def measurement(self, startF, stopF, points, rbw, rbw_auto, depth, maxF, interval, split, sdr_settings, loop=True):
        # runs in separate thread
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
        levl = np.full(points, -140, dtype=np.float32)
        maxl = np.full(points, -140, dtype=np.float32)
        minl = np.full(points, 0, dtype=np.float32)
        buffer = np.full((depth, points), np.nan, dtype=np.float32)  # used for waterfall and calculating averages

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
                    logging.info(f'data error, dataBlock={dataBlock!r}')
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
                        try:
                            if self.usb.read(2) != b'}{':  # the end of scan marker character is '}{'
                                logging.info('QtTinySA display is out of sync with tinySA frequency')
                                self.signals.error.emit(None, 'QtTinySA display out of sync', 'Ok', 'Critical')
                                self.sweeping = False
                                self.usb.reset_input_buffer()
                                break
                        except serial.SerialException:
                            self.sweeping = False
                            self.signals.error.emit(None, 'serial port exception', 'Ok', 'Critical')
                            self.usb.reset_input_buffer()
                            break
                        firstRun = False
                timeElapsed = updateTimer.nsecsElapsed()  # how long this batch of measurements has been running, nS
                if timeElapsed/1e6 > interval:  # GUI update interval mS
                    # send the measurement data to router() in the Analyser class
                    # self.signals.result.emit(freq, levl, maxl, minl, buffer, self.usbPort, self.sn, 0, split, False)
                    self.signals.result.emit(freq, levl, maxl, minl, buffer, self.usbPort, self.sn, self.name, split, False)
                    updateTimer.start()

            # also send the measurement data to router() at the end of each sweep
            # self.signals.result.emit(freq, levl, maxl, minl, buffer, self.usbPort, self.sn, 0, split, True)
                    self.signals.result.emit(freq, levl, maxl, minl, buffer, self.usbPort, self.sn, self.name, split, True)
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
        try:
            self.usb.write(command.encode())
            self.usb.read_until(command.encode() + b'\n')  # skip command echo
            response = self.usb.read_until(b'ch> ')  # until prompt
            logging.debug(f'serialQuery: response = {response}')
            return response[:-6].decode()  # remove prompt
        except UnicodeDecodeError:
            logging.info('serialQuery: ignored UnicodeDecodeError')


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
    # def __init__(self, usbPort, product, sigs, dev_id):
    def __init__(self, usbPort, product, sigs):
        super().__init__()
        self.usb = None
        self.usbPort = usbPort
        self.product = product
        self.firmware = None
        self.sweeping = None
        self.setSignals(sigs)
        self.threadRunning = False
        self.enabled = True
        self.setDevice(usbPort)
        # self.id = dev_id  # this is the device_ID, 0 to 3, where 0 = first device to connect on USB ports, etc.
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
                # Connected {self.name} sn={self.sn} fw={self.firmware} with ID={self.id}
                logging.info(f'Connected {self.name} sn={self.sn} fw={self.firmware} on {usbPort}')
                self.enabled = True
                return True
            else:
                self.enabled = False
                return False

    def measurement(self, startF, stopF, points, rbw, rbw_auto, depth, maxF, interval, split, sdr_settings, loop=True):  # run in separate thread
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
                # self.signals.result.emit(freq, levl, maxl, minl, buffer, self.id, self.sn, 0, split, False)
                self.signals.result.emit(freq, levl, maxl, minl, buffer, self.usbPort, self.sn, 0, split, False)

            buffer = np.roll(buffer, 1, axis=0)
            buffer[0] = levl
            # also send the measurement data to router() at the end of each sweep
            # self.signals.result.emit(freq, levl, maxl, minl, buffer, self.id, self.sn, 0, split, True)
            self.signals.result.emit(freq, levl, maxl, minl, buffer, self.usbPort, self.sn, 0, split, True)          
            self.usb.reset_input_buffer()
        self.threadRunning = False

    def set_ctrls(self, rbw, rbw_auto, attn, lna, spur, sdr_settings):
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
    # SN:                  usage: NanoVNA-F V2 Serial Number

# data [0-6]: Dumps specific sweep data?
# 0:S11, 1:S21, 2:Directivity, 3:Source Match, 4:Reflection Tracking
# 5:Transmission Tracking, 6:Isolation




class SDRplay(QObject):
    '''
    SoapySDR Support

    IQ Balance
    
    The driver defaults to true via the iqcorr_ctrl parameter. It can be manually toggled via hardware arguments:
    iqcorr_ctrl=true
    
    DC Removal
    
    • The hardware structure handles DC offset removal natively via the standard SoapySDR corrections framework.
	• Control string: dc_offset_mode=true or via standard setDCOffsetMode() API commands
    
    sdr.setDCOffsetMode(SOAPY_SDR_RX, 0, True)  # Enables DC removal mode

    RX Gain
    
    • The RSPdx exposes specific gain controls across its multi-stage design:
	• IFGR (IF Gain Reduction): Adjusts the baseband gain stage (typically ranges from 20 to 59 dB).
	• RFGR (RF Gain Reduction / LNA State): Selectable step-attenuation levels (ranges from 0 to 27 dB, mapping to
      the specific LNA attenuation steps for the chosen frequency band).
	• rfgain_sel: Numeric profile selection (0 to 27) that acts as an overall target state 
    
    sdr.setGain(SOAPY_SDR_RX, 0, "IFGR", 35.0)   # IF Gain Reduction stage
    sdr.setGain(SOAPY_SDR_RX, 0, "RFGR", 12.0)   # RF (LNA) Gain Reduction
    sdr.setGainMode(SOAPY_SDR_RX, 0, True)       # Turn on AGC
    
    FIR Selection
    
    • You cannot upload custom arbitrary FIR taps directly via SoapySDR, but you can select the analog filter
      bandwidths natively supported by the RSPdx's internal chip.
	• Selectable bandwidth options using bandwidth= 0.2, 0.3, 0.6, 1.536, 5, 6, 7, and 8 MHz.
    
    # Select an anti-aliasing hardware filter bandwidth (in Hz)
    sdr.setBandwidth(SOAPY_SDR_RX, 0, 200000.0)  # 200 kHz preset filter
    sdr.setBandwidth(SOAPY_SDR_RX, 0, 8000000.0)  # 8 MHz preset filter
    
    
    Antenna Switch	Yes	  antenna="Antenna A"	Selects physical inputs Antenna A, B, or C (HDR port).
    Bias-T	Yes	  biasT_ctrl=true	Provides 4.7V DC to power external LNAs on Antenna B.
    
    sdr.setAntenna(SOAPY_SDR_RX, 0, "Antenna C")  # Switches physical input to HDR port (Antenna C)
    sdr.writeSetting("biasT_ctrl", "true")         # Directs 4.7V DC down Antenna B path

    
    Hardware Notch Filters
    
    The RSPdx features dedicated physical broadcast filter traps that can be clicked on or off via argument strings:
	• rfnotch_ctrl=true/false (Broadcast FM / AM Notch Filter)
	• dabnotch_ctrl=true/false (DAB Band Notch Filter)
    
    sdr.writeSetting("rfnotch_ctrl", "true")    # Engage AM/FM Broadcast Notch filter
    sdr.writeSetting("dabnotch_ctrl", "true")   # Engage DAB Band Notch filter'''


class SoapySDRDevice(QObject):
    def __init__(self, port, description, driver, sigs):
        super().__init__()       
        self.sdr = SoapyReceiverProcess()
        self.setSignals(sigs)
        self.usbPort = port  # a fake port for code compatibility
        self.enabled = True
        self.sweeping = False
        self.threadRunning = False
        self.name = None
        self.sn = 0
        self.driver = driver
        self.fifoTimer = QTimer(self)  # not used, for compatibility

    def setSignals(self, sigs):
        self.signals = WorkerSignals()
        self.signals.save.connect(sigs["save"])
        # self.sdr.error.connect(sigs["error"])
        # self.sdr.status.connect(sigs["status"])
        self.signals.result.connect(sigs["result"])
        self.signals.error.connect(sigs["error"])

    def set_ctrls(self, rbw, rbw_auto, attn, lna, spur, sdr_settings):
        self.sdr.set_device(self.driver, 
                            {"driver": self.driver,"name": self.name,"serial": self.sn},
                            port_in_use=self.usbPort)
        
        # set common controls
        self.sdr.set_kaiser_beta(sdr_settings.get("sdr_kaiser"))
        self.sdr.set_auto_couple(sdr_settings.get("sdr_link_span_rbw"))
        self.sdr.set_max_fft_size(2 ** sdr_settings.get("sdr_fft"))

        # set hardware-specific controls
        if self.driver in ("lime", "limesuiteng"):
            self._set_lime(sdr_settings)
        
        if self.driver == "hackrf":
            self._set_hackrf(sdr_settings)
            
        if self.driver == "rtlsdr":
            self._set_rtl(sdr_settings)
         
    def _set_lime(self, sdr_settings):
            self.sdr.set_iq_balance_mode(sdr_settings.get("lime_iq_balance"))
            self.sdr.set_antenna(sdr_settings.get("lime_rx_ant"))
            self.sdr.set_rx_gain(sdr_settings.get("lime_lna_gain"))  # needs to be updated once soapy_rx updated
            tia = sdr_settings.get("lime_tia_gain")
            pga = sdr_settings.get("lime_pga_gain")            
            dc = sdr_settings.get("lime_dc")
            if dc == "Hardware":
                self.sdr.set_dc_offset_removal(mode="hardware")
            elif dc == "Both":
                self.sdr.set_dc_offset_removal(mode="both")
            elif dc == "Software":
                self.sdr.set_dc_offset_removal(mode="software")

    def _set_lime_gfir(self, sdr_settings):
            # The Lime's low-pass real-time baseband FIR filter must be set after sample rate
            segment_span = self.sdr.segment_span_hz
            gfir_ratio = sdr_settings.get("lime_gfir_bw") / 100
            gfir_bw = segment_span * gfir_ratio
            if sdr_settings.get("lime_gfir_enable"):
                self.sdr.set_gfir_lpf_bandwidth(gfir_bw)
            else:
                # set it to zero which is hard coded in soapy_receiver to write "DISABLE_GFIR_LPF"
                self.sdr.set_gfir_lpf_bandwidth(0)  

    def _set_hackrf(self, sdr_settings):
            IF_gain = sdr_settings.get("hack_if_gain")
            BB_gain = sdr_settings.get("hack_bb_gain")
            filter_bw = sdr_settings.get("hack_filter")
            rx = sdr_settings.get("hack_rx")
            amp = sdr_settings.get("hack_amp")  # needs an update to soapy_receiver
            # and we need to do something with those

            self.sdr.set_antenna("TX/RX")

    def _set_rtl(self, sdr_settings):
            max_rate = sdr_settings.get("rtl_max_rate")
            agc = sdr_settings.get("rtl_agc_box")
            bias_T = sdr_settings.get("rtl_bias")
            # and we need to do something with those
            
            rx_gain = sdr_settings.get("rtl_gain")
            self.sdr.set_rx_gain(rx_gain)

    def _set_SDRplay(self, sdr_settings):
        '''
        SoapySDR Support
    
        IQ Balance
        
        The driver defaults to true via the iqcorr_ctrl parameter. It can be manually toggled via hardware arguments:
        iqcorr_ctrl=true
        
        DC Removal
        
        • The hardware structure handles DC offset removal natively via the standard SoapySDR corrections framework.
        	• Control string: dc_offset_mode=true or via standard setDCOffsetMode() API commands
        
        sdr.setDCOffsetMode(SOAPY_SDR_RX, 0, True)  # Enables DC removal mode
    
        RX Gain
        
        • The RSPdx exposes specific gain controls across its multi-stage design:
            FGR (IF Gain Reduction): Adjusts the baseband gain stage (typically ranges from 20 to 59 dB).
            FGR (RF Gain Reduction / LNA State): Selectable step-attenuation levels (ranges from 0 to 27 dB, mapping to
            the specific LNA attenuation steps for the chosen frequency band).
            fgain_sel: Numeric profile selection (0 to 27) that acts as an overall target state 
        
        sdr.setGain(SOAPY_SDR_RX, 0, "IFGR", 35.0)   # IF Gain Reduction stage
        sdr.setGain(SOAPY_SDR_RX, 0, "RFGR", 12.0)   # RF (LNA) Gain Reduction
        sdr.setGainMode(SOAPY_SDR_RX, 0, True)       # Turn on AGC
        
        FIR Selection
        
        • You cannot upload custom arbitrary FIR taps directly via SoapySDR, but you can select the analog filter
          bandwidths natively supported by the RSPdx's internal chip.
          electable bandwidth options using bandwidth= 0.2, 0.3, 0.6, 1.536, 5, 6, 7, and 8 MHz.
        
        # Select an anti-aliasing hardware filter bandwidth (in Hz)
        sdr.setBandwidth(SOAPY_SDR_RX, 0, 200000.0)  # 200 kHz preset filter
        sdr.setBandwidth(SOAPY_SDR_RX, 0, 8000000.0)  # 8 MHz preset filter
        
        
        Antenna Switch	Yes	  antenna="Antenna A"	Selects physical inputs Antenna A, B, or C (HDR port).
        Bias-T	Yes	  biasT_ctrl=true	Provides 4.7V DC to power external LNAs on Antenna B.
        
        sdr.setAntenna(SOAPY_SDR_RX, 0, "Antenna C")  # Switches physical input to HDR port (Antenna C)
        sdr.writeSetting("biasT_ctrl", "true")         # Directs 4.7V DC down Antenna B path
    
        Hardware Notch Filters
        
        The RSPdx features dedicated physical broadcast filter traps that can be clicked on or off via argument strings:
            fnotch_ctrl=true/false (Broadcast FM / AM Notch Filter)
            dabnotch_ctrl=true/false (DAB Band Notch Filter)
        
        sdr.writeSetting("rfnotch_ctrl", "true")    # Engage AM/FM Broadcast Notch filter
        sdr.writeSetting("dabnotch_ctrl", "true")   # Engage DAB Band Notch filter'''


    def _set_arr(self, depth):
        # fetch the actual sweep and FFT parameters, which are calculated after setting RBW
        fft_size = self.sdr.fft_size
        swp_size = self.sdr.composite_size
        num_segs = self.sdr.num_segments
        
        maxl = np.full(fft_size, -140, dtype=float)
        minl = np.full(fft_size, 0, dtype=float)
        buffer = np.full((depth, fft_size), np.nan, dtype=float)  # used for waterfall and calculating averages
        return maxl, minl, buffer
      
    def measurement(self, startF, stopF, points, rbw, rbw_auto, depth, maxF, interval, split, sdr_settings, loop):
        '''SDR fft size is calculated and optimised based on span, RBW and kaiser window beta.
           In segments mode the segments slightly overlap so the array size is bigger than for composite'''
        self.threadRunning = True
        # span = stopF - startF

        if sdr_settings.get("sdr_mode_s"):
            mode = "segments"
            self.sdr.set_offset_tuning(int(span / 4))
        else:
            mode = "composite"
        # self.sdr.set_sweep(startF, stopF, output_mode=mode)
        
        span = stopF - startF
        self.sdr.set_span(span)
        centreF = (startF + int(span / 2))
        self.sdr.set_center_frequency(centreF)      
         
        # rbw must be set after the sample rate
        self.sdr.set_rbw(rbw * 1000)
        
        # lime GFIR must be set after the sample rate
        if self.driver in ("lime", "limesuiteng"):
            self._set_lime_gfir(sdr_settings)
      
        # fetch the actual sweep and FFT parameters, which are calculated after setting RBW
        fft_size = self.sdr.fft_size
        maxl, minl, buffer = self._set_arr(depth)
        
        self.sdr.start()
        time.sleep(3)
        
        while self.sweeping:
            if not self.sdr.event_queue.empty():
                event = self.sdr.event_queue.get_nowait()
                logging.info(f'SoapySDR event: {event[1]}')
                if event[0] == "status":
                    # a status message alerts to a possible FFT size change
                    new_fft_size = self.sdr.fft_size
                    if new_fft_size != fft_size:
                        maxl, minl, buffer = self._set_arr(depth)

            try:
                # fetch data from the mp queue.  Blocks, consuming no cpu cycles, until data pops up in the queue
                freqs, power, identifier, port_in_use = self.sdr.data_queue.get(timeout=5)
                try:
                    np.fmax(power, maxl, out=maxl)  # compare current level with max and min
                    np.fmin(power, minl, out=minl)  # and save them back on themselves
                    buffer[:] = np.roll(buffer, 1, axis=0)
                    buffer[0] = power
                except ValueError:
                    logging.info(f'Lime measurement: skipped array size mismatch')
                
                # send the measurement data to router() in the Analyser class
                # sn is a string for Lime but the 'result' signal expects int so send 100 for now
                ser = 100
                self.signals.result.emit(freqs, power, maxl, minl, buffer, self.usbPort, ser, self.name, split, True)
            except queue.Empty:
                logging.info('no data from SDR after 5 seconds; stopped')
                break
              
        self.sdr.stop()
        self.sdr.wait()
        self.threadRunning = False
     
    def test(self, port):
        port_list = port.split('_')
        self.name = port_list[0]
        self.sn = port_list[1]
        logging.info(f'name= {self.name} sn = {self.sn}')
        return True

    @Slot()
    def usbSend(self):
        # for compatibility
        return
    
    def close(self):
        if self.sdr is not None:
            self.sweeping = False   # tell measurement()'s loop to exit, if a scan is currently running
            self.sdr.shutdown(timeout=5.0)
            logging.debug(f'Close: Lime SDR {self.usbPort} stopped')
            self.sdr = None

class SiglentSA(QObject):
    def __init__(self, visaPort, product, sigs):
        super().__init__()
        self.setSignals(sigs)
        self.enabled = True
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
        self.recording = False
        self.threadRunning = False
        self.setSignals(sigs)
        self.data_arr = np.full((2,2), np.nan, dtype=np.float32)
        self.row_count = 0
        self.dev_num = 0
        self.sn = 0
        self.name = ''
        self.rec_time = 0
        
    def setSignals(self, sigs):
        self.signals = WorkerSignals()
        # self.signals.result.connect(sigs["result"])
        self.signals.save.connect(sigs["save"])
        self.signals.progress.connect(sigs["progress"])
   
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
            save_time = time.strftime('%Y-%m-%d-%H%M%S')
            file_name = str(save_time + '_s' + str(self.sn) + '_d' + str(self.id))
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
           
    def configure_array(self, points, dev_id, dev_count):
        self.dev_id = dev_id
        rows = int(self.MAX_FIELDS / (dev_count * points))  # give a file size of ~32MB for 1 device
        self.data_arr = np.full((rows, points+1), np.nan, dtype=np.float64)


class FilePlayer(QObject):
    '''data_array structure
            row 0 col 0 = hardware serial num last 8 digits
            rows: 0=measurement point frequencies from col 1; row 1 onwards from col 1=dBm readings
            cols: 0=times from row 1 onwards; 1 onwards from row 1=points dBm values
       10e6 fields in array gives ~32MB mem/file size and ~6h for 4 devices at 101 points'''
    def __init__(self, sigs):
        super().__init__()
        self.sweeping = False
        self.threadRunning = False
        self.enabled = True
        self.setSignals(sigs)
        self.data_arr = np.full((2,2), np.nan, dtype=np.float32)
        self.row_count = 0
        self.sn = 0
        self.name = ''
        self.device = None
        self.speed = 1
        self.usbPort = None
        
    def setSignals(self, sigs):
        self.signals = WorkerSignals()
        self.signals.result.connect(sigs["result"])
        # self.signals.save.connect(sigs["save"])
        self.signals.progress.connect(sigs["progress"])
   
    def server(self, depth, interval, slider_position, playing, split):
        '''runs in a thread, serving data to the router from a file loaded into self.data_arr'''
        self.threadRunning = True
        updateTimer = QElapsedTimer()
        
        # set the data arrays and the scan parameters for this player instance
        scans = np.shape(self.data_arr)[0] - 1
        points = np.shape(self.data_arr)[1] - 1
        freq = self.data_arr[0, 1:]  # freqs are in row 0 from col 1 onwards
        levl = np.full(points, -140, dtype=np.float32)
        times = self.data_arr[1:, 0]  # time stamps are in col 0 from row 1 onwards
        maxl = np.full(points, -140, dtype=np.float32)
        minl = np.full(points, 0, dtype=np.float32)
        sweep_time = 1
        buffer = np.full((depth, points), None, dtype=np.float32)  # used for waterfall and averages 

        if scans > 1:
            sweep_time = float(times[1] - times[0])
        slider_row = int(scans * slider_position/100) + 1
        row = slider_row
        if playing:
            final_row = scans
        else:
            # the slider is controlling playback, so only update a single row
            final_row = min(row + 1, scans)

        # fill the buffer with data from its start up to the slider row
        if slider_row >=2:
            if slider_row > depth:
                slice_start = slider_row - depth
            else:
                slice_start = 1
            buffer[0:slider_row - 1, :] = self.data_arr[slice_start:slider_row, 1:]

        # start reading data from the recording file and send it to the router()
        updateTimer.start()
        while self.sweeping and row < final_row:
            if row > 1:
                sweep_time = float(times[row] - times[row - 1])
            timestamp = float(times[row]) - sweep_time  # time is stamped at the end of a sweep
            levl = self.data_arr[row, 1:]  # col 0 of the array is a timestamp, so values start at col 1
            np.fmax(levl, maxl, out=maxl)  # compare current level with max and min
            np.fmin(levl, minl, out=minl)  # and save them back on themselves
            if playing:
                buffer = np.roll(buffer, 1, axis=0)
                buffer[0] = levl
                pause = min(1, sweep_time / self.speed)  # clip playback sweep time to max 1 second
                time.sleep(pause)
            else:
                time.sleep(interval / 1e3)  # interval is in mS
            timestamp += sweep_time
            timeElapsed = updateTimer.nsecsElapsed() / 1e6  # how long the player has been running, mS
            row += 1
            if timeElapsed >= interval:
                identifier = time.ctime(timestamp)
            # send the measurement data only at the frequency set by interval
                self.signals.result.emit(freq, levl, maxl, minl, buffer,
                                         self.usbPort, self.sn, identifier, split, True)
                updateTimer.start()
                if playing:
                    self.signals.progress.emit(100 * row/scans)
        self.sweeping = False
        self.threadRunning = False   

    def close(self):
        # needed for code compatibility
        return
    