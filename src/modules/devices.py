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
from platform import system
from PySide6.QtCore import QObject
from serial.tools import list_ports
from datetime import datetime


class USBdevice(QObject):
    def __init__(self):
        super().__init__()
        # self.usb = None
        self.ports = []
        self.firmware = None

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

    def control(self):
        # try to open a USB connection to hardware....... need to check if it works in Windows now
        self.dev0 = self.dev1 = self.dev2 = self.dev3 = None
        for port in self.ports:
            func = {"tinySA": Tiny(port.device, basic=True),
                    "tinySA4": Tiny(port.device, basic=False),
                    "NanoVnaPro Virtual ComPort": Nano(port.device),
                    "LimeSDR-USB": Lime(port.device)}
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

    # settings.ui.deviceBox.addItem(self.identify(port) + " on " + port.device)
    # def identify(self, port):
    #     # Windows returns no information to pySerial list_ports.comports()
    #     if system() == 'Linux' or system() == 'Darwin':
    #         return port.product
    #     else:
    #         return 'USB device'


class Tiny(QObject):
    def __init__(self, usbPort, basic=False):
        super().__init__()
        self.fifo = queue.SimpleQueue()
        self.usb = None
        self.basic = basic

    def test(self, usbPort):  # tests comms and initialise if found
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
        if sType == 'auto' and not self.basic:  # tinySA3 (basic) has no auto spur mode
            sType = 'on'
        command = 'spur ' + sType + '\r'
        self.fifo.put(command)

    def setTime(self):
        if not self.basic:  # and settings.ui.syncTime.isChecked():
            dt = datetime.now()
            y = dt.year - 2000
            command = f'time b 0x{y}{dt.month:02d}{dt.day:02d} 0x{dt.hour:02d}{dt.minute:02d}{dt.second:02d}\r'
            self.fifo.put(command)

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


class Nano(QObject):
    def __init__(self, usbPort):
        super().__init__()
        self.usb = usbPort
        # nano vna

    def test(self, usbPort):  # tests comms and initialise if found
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
        # version = 'tinySA4_v1.4-199-gde12ba2'  # for testing ultra
        # version = 'tinySA_v1.4-175-g1419a93'   # for testing basic
        return version

    def serialQuery(self, command):
        self.usb.write(command.encode())
        self.usb.read_until(command.encode() + b'\n')  # skip command echo
        response = self.usb.read_until(b'ch> ')  # until prompt
        logging.debug(f'serialQuery: response = {response}')
        return response[:-6].decode()  # remove prompt


class Lime(QObject):
    def __init__(self, usbPort):
        super().__init__()
        # soapy


class RTL(QObject):
    def __init__(self, usbPort):
        super().__init__()
        # soapy


class SiglentSA(QObject):
    def __init__(self, visaPort):
        super().__init__()
        # pyvisa
