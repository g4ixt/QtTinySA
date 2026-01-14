#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jan 13 09:45:56 2026

@author: ian
"""

# Copyright 2026 Ian Jefferson G4IXT
# SPDX-License-Identifier: GPL-3.0-or-later

import system
import logging
import serial
import time
import queue
from PySide6.QtCore import QObject
from serial.tools import list_ports
from datetime import datetime

VID = [0x0483, 0x1d50]  # 1155 tinySA or NanoVNA, limeSDR
PID = [0x5740, 0x6108]  # 22336 tinySA or NanoVNA, limeSDR

class usbPort(QObject):
    def __init__(self):
        super().__init__()
        self.usb = None
        self.ports = []

    # def probeUSB(self):
    #     # triggered by self.usbCheck QTimer - if tinySA wasn't found checks repeatedly for device, i.e.'hotplug'
    #     if len(self.ports) == 0:
    #         self.openUSB()
    #     else:
    #         for i in range(len(self.ports)):
    #             if self.identify(self.ports[i])[:6] in ('tinySA', 'USB de'):
    #                 # self.usbCheck.stop()
    #                 usbCheck.stop()
    #             else:
    #                 self.openUSB()

    def probeUSB(self):
        # Find com ports matching hardware ID
        if len(self.ports) == 0:
            usbPorts = list_ports.comports()
            for port in usbPorts:
                if port.vid == VID and port.pid == PID:
                    if port not in self.ports:
                        self.ports.append(port)
            return self.ports()
                        # settings.ui.deviceBox.addItem(self.identify(port) + " on " + port.device)

        #     if len(self.ports) == 1:  # found only one device so just test it
        #         # usbCheck.stop()
        #         self.testUSB(self.ports[0])
        #         return  # some info

        #     if len(self.ports) > 1:  # several devices were found
        #         pass
        #     #     settings.ui.deviceBox.insertItem(0, "Select device")
        #     #     settings.ui.deviceBox.setCurrentIndex(0)
        #     #     popUp(QtTSA, "Several devices detected.  Choose device in Settings > Preferences", 'Ok', 'Info')
        #     #     usbCheck.stop()
        # else:
        #     for i in range(len(self.ports)):
        #         if self.identify(self.ports[i])[:6] in ('tinySA', 'USB de'):
        #             # self.usbCheck.stop()
        #             usbCheck.stop()

    def testUSB(self, port):  # tests comms and initialises tinySA if found
        try:
            self.usb = serial.Serial(port.device, baudrate=576000)
            logging.info(f'Serial port {port.device} open: {self.usb.isOpen()}')
        except serial.SerialException:
            logging.info('Serial port exception. A possible cause is that your username is not in the "dialout" group.')
            # popUp(QtTSA, 'Serial port exception', 'Ok', 'Critical')
        if self.usb:
            for i in range(4):  # try 4 times to communicate with tinySA over USB serial
                firmware = self.version()  # is fn from main
                if firmware[:6] == 'tinySA':
                    logging.info(f'{port.device} test {i} : {firmware[:16]}')
                    break
                else:
                    time.sleep(1)
            # split firmware into a list of [device, major version number, minor version number, other stuff]
            self.firmware = firmware.replace('_', '-').split('-')
            if firmware[:6] == 'tinySA':
                if firmware[0] == 'tinySA4' and float(self.firmware[1][-3:] + self.firmware[2]) < 1.4177:
                    logging.info('for fastest possible scan speed, upgrade firmware to v1.4-177 or later')
                if self.firmware[1][0] == "v":
                    return self.firmware # setForDevice needs this info
                else:
                    logging.info(f'{port.device} test found unexpected firmware {firmware}')
            else:
                logging.info(f'firmware {firmware} for {self.identify(port)} on {port.device} is not a tinySA')

    def identify(self, port):
        # Windows returns no information to pySerial list_ports.comports()
        if system() == 'Linux' or system() == 'Darwin':
            return port.product
        else:
            return 'USB device'

    def closeUSB(self):
        if self.usb:
            self.usb.close()
            logging.info(f'Serial port open: {self.usb.isOpen()}')
            self.usb = None

class TinySA(QObject):
    def __init__(self):
        super().__init__()
        self.fifo = queue.SimpleQueue()

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
        if sType == 'auto' and not self.tinySA4:  # tinySA3 (basic) has no auto spur mode
            sType = 'on'
        command = 'spur ' + sType + '\r'
        self.fifo.put(command)

    def setTime(self):
        # if self.tinySA4 and settings.ui.syncTime.isChecked():
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

class LimeSDR(QObject):
    def __init__(self):
        super().__init__()
        # soapy

class RTLSDR(QObject):
    def __init__(self):
        super().__init__()
        # soapy
        
class SiglentSA(QObject):
    def __init__(self):
        super().__init__()
        # pyvisa