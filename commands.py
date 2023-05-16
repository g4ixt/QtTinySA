#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Created on 13 May 2023
@author: Ian Jefferson G4IXT

This uses and expands on the code from tinysa_scanraw.py from Nanovna-tools by 'Ho-Ro'
https://github.com/Ho-Ro/nanovna-tools

"""

import numpy as np
import serial
from serial.tools import list_ports
import time
import struct
import logging
import sys

# tinysa USB IDs
VID = 0x0483
PID = 0x5740

F_LOW = 88e6
F_HIGH = 90e6
POINTS = 450
logging.basicConfig(format="%(message)s", level=logging.INFO)

# Get tinysa device automatically
def getport() -> str:
    device_list = list_ports.comports()
    for device in device_list:
        if device.vid == VID and device.pid == PID:
            return device.device
    raise OSError("device not found")

def setTinySA(s_port, command):
    with serial.Serial(port=s_port, baudrate=115200) as tinySA:
        tinySA.timeout = 1

        while tinySA.inWaiting():
            tinySA.read_all()  # keep the serial buffer clean
            time.sleep(0.1)

        logging.debug(command)
        tinySA.write(command)
        tinySA.read_until(b'ch> ')  # skip command echo and prompt


# return 1D numpy array with power as dBm
def get_tinysa_dBm(s_port, f_low=F_LOW, f_high=F_HIGH, points=POINTS, rbw=0) -> np.array:
    with serial.Serial(port=s_port, baudrate=115200) as tinySA:
        tinySA.timeout = 1
        while tinySA.inWaiting():
            tinySA.read_all()  # keep the serial buffer clean
            time.sleep(0.1)

        span_k = (f_high - f_low) / 1e3
        if 0 == rbw:  # calculate from scan range and steps
            rbw_k = span_k / points  # RBW / kHz
        else:
            rbw_k = rbw / 1e3

        if rbw_k < 2:
            rbw_k = 2
        elif rbw_k > 850:
            rbw_k = 850

        rbw_command = f'rbw {int(rbw_k)}\r'.encode()
        logging.debug(rbw_command)
        tinySA.write(rbw_command)
        tinySA.read_until(b'ch> ')  # skip command echo and prompt

        # set timeout accordingly - can be very long - use a heuristic approach
        timeout = int(span_k / (rbw_k * rbw_k) + points / 1e3 + 5)
        tinySA.timeout = timeout

        logging.debug(f'frequency step: {int( span_k / ( points-1 ) )} kHz\n')
        logging.debug(f'RBW: {int(rbw_k)} kHz\n')
        logging.debug(f'serial timeout: {timeout} s\n')

        scan_command = f'scanraw {int(f_low)} {int(f_high)} {int(points)}\r'.encode()
        tinySA.write(scan_command)
        tinySA.read_until(b'{')  # skip command echoes
        raw_data = tinySA.read_until(b'}ch> ')
        tinySA.write('rbw auto\r'.encode())  # switch to auto RBW for faster tinySA screen update

    raw_data = struct.unpack('<' + 'xH'*points, raw_data[:-5])  # ignore trailing '}ch> '
    raw_data = np.array(raw_data, dtype=np.uint16)
    # tinySA:  SCALE = 128
    # tinySA4: SCALE = 174rbw_command = f'rbw {(rbw)}\r'.encode()
    SCALE = 174
    dBm_power = (raw_data / 32) - SCALE  # scale 0..4095 -> -128..-0.03 dBm
    logging.debug(dBm_power)
    return dBm_power

command = f'spur auto\r'.encode()
setTinySA(getport(), command)

t_start = time.time()
for i in range(100):
    meas_power = get_tinysa_dBm(getport(), F_LOW, F_HIGH, POINTS, 100000)

t_end = time.time()

# create a 1D numpy array with scan frequencies
frequencies = np.linspace(F_LOW, F_HIGH, POINTS)
logging.debug(frequencies)

# for (freq, dBm) in zip(frequencies, meas_power):  # iterate over array of (freq, dBm) tuples
#     line = f'{freq:.0f}, {dBm:.1f}'
#    logging.debug(line)

duration = t_end - t_start
logging.info(f'scan duration: {duration:.1f} s\n')
