#!/usr/bin/python

import numpy as np
import serial
from serial.tools import list_ports
import time
import struct
import argparse
import sys

# tinysa USB IDs
VID = 0x0483
PID = 0x5740

F_LOW = 90e6
F_HIGH = 100e6
POINTS = 101


# Get tinysa device automatically
def getport() -> str:
    device_list = list_ports.comports()
    for device in device_list:
        if device.vid == VID and device.pid == PID:
            return device.device
    raise OSError("device not found")


# return 1D numpy array with power as dBm
def get_tinysa_dBm( s_port, f_low=F_LOW, f_high=F_HIGH, points=POINTS, rbw=0, verbose=None ) -> np.array:
    with serial.Serial( port=s_port, baudrate=115200 ) as tinySA:
        tinySA.timeout = 1
        while tinySA.inWaiting():
            tinySA.read_all() # keep the serial buffer clean
            time.sleep( 0.1 )

        span_k = ( f_high - f_low ) / 1e3
        if 0 == rbw: # calculate from scan range and steps
            rbw_k = span_k / points # RBW / kHz
        else:
            rbw_k = rbw / 1e3

        if rbw_k < 3:
            rbw_k = 3
        elif rbw_k > 600:
            rbw_k = 600

        rbw_command = f'rbw {int(rbw_k)}\r'.encode()
        tinySA.write( rbw_command )
        tinySA.read_until( b'ch> ' ) # skip command echo and prompt

        # set timeout accordingly - can be very long - use a heuristic approach
        timeout = int( span_k / ( rbw_k * rbw_k ) + points / 1e3 + 5)
        tinySA.timeout = timeout
        print("timeout =", timeout)

        if verbose:
            sys.stderr.write( f'frequency step: {int( span_k / ( points-1 ) )} kHz\n' )
            sys.stderr.write( f'RBW: {int(rbw_k)} kHz\n' )
            sys.stderr.write( f'serial timeout: {timeout} s\n' )

        scan_command = f'scanraw {int(f_low)} {int(f_high)} {int(points)}\r'.encode()
        tinySA.write( scan_command )
        tinySA.read_until( b'{' ) # skip command echoes
        raw_data = tinySA.read_until( b'}ch> ' )
        tinySA.write( 'rbw auto\r'.encode() ) # switch to auto RBW for faster tinySA screen update

    raw_data = struct.unpack( '<' + 'xH'*points, raw_data[:-5] ) # ignore trailing '}ch> '
    raw_data = np.array( raw_data, dtype=np.uint16 )
    # tinySA:  SCALE = 128
    # tinySA4: SCALE = 174
    SCALE = 174
    print(raw_data)
    dBm_power = (raw_data / 32) - SCALE # scale 0..4095 -> -128..-0.03 dBm
    return dBm_power


# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser( description='Get a raw scan from tinySA, formatted as csv (freq, power)')
ap.add_argument( '-d', '--device', dest = 'device', default=None, help = 'connect to serial device' )
ap.add_argument( '-s', '--start', type=float, default=F_LOW, help=f'start frequency, default = {F_LOW} Hz' )
ap.add_argument( '-e', '--end', type=float, default=F_HIGH, help=f'end frequency, default = {F_HIGH} Hz' )
ap.add_argument( '-p', '--points', type=int, default=POINTS, help=f'Number of sweep points, default = {POINTS}' )
ap.add_argument( '-r', '--rbw', type=float, default=0,
                help='resolution bandwidth / Hz, default = 0 (calculate RBW from scan steps)')
ap.add_argument( '-c', '--comma', action='store_true', help='use comma as decimal separator' )
ap.add_argument( '-v', '--verbose', action='store_true', help='provide info about scan parameter and timing' )
options = ap.parse_args()

t_start = time.time()
meas_power = get_tinysa_dBm( options.device or getport(),
                             options.start, options.end, options.points, options.rbw, options.verbose )
t_end = time.time()

# create a 1D numpy array with scan frequencies
frequencies = np.linspace( options.start, options.end, options.points )

for (freq, dBm) in zip( frequencies, meas_power ): # iterate over array of (freq, dBm) tuples
    line = f'{freq:.0f}, {dBm:.1f}'
    if options.comma: # e.g. Germany uses comma as decimal separator and semicolon as field separator
        line = line.replace(',', ';').replace('.', ',')
    print( line )

duration = t_end - t_start
sys.stderr.write( f'scan duration: {duration:.1f} s\n' )
