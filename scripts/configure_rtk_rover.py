#!/usr/bin/env python
"""
Configure a NovAtel OEMV series to receive RTCMv3 corrections on COM2. Should work on
just about any RTK-capable NovAtel, but this is only tested on the ProPakv3.
It is assumed that we are connected to the receiver on COM1 and that corrections 
will be received through COM2 @ 9600 baud. First the receiver is reconfigured
to communicate at 115200 baud on COM1, regardless of current setting.

It is suggested that the receiver be reset prior to running this script, but 
that is not strictly necessary. This is only due to the fact that the way in 
which various modes and smoothing. Factory reset may be performed by issuing the
following command in a serial terminal:
  
  freset standard

Responses from the receiver are printed to screen. Successful commands return
the following (when connected to COM1 on the receiver):


  <OK
  [COM1]

Usage: run `configure_rtk_rover.py -h` for current usage.

The known station names are shown below


Robert Cofield
2016-03-28
"""
import sys
from serial import Serial
from time import sleep
import argparse

# parse command line arguments
parser = argparse.ArgumentParser(description="Configure a NovAtel RTK base station")
parser.add_argument('-p', '--port', dest='port', default='/dev/ttyUSB0', help="What serial port COM1 of the NovAtel is plugged into. (Default /dev/ttyUSB0)")
parser.add_argument('-b', '--baud', dest='baud', default=9600, help="What baud rate to use in communicating with the NovAtel on the receiver's COM1. (Default 9600)")
args = parser.parse_args()

port_name = args.port
cur_baud = args.baud

print 'initially using: ', port_name, cur_baud
with Serial(port_name, cur_baud, timeout=0.1) as port:
  print 'Unlogging...'
  port.write('unlogall\r\n')
  rsp = port.readall()
  print rsp
  if rsp != '\r\n<OK\r\n[COM1]':
    raise Exception('Unexpected response. Try running again.')
  print 'Factory reset..'
  print 'Setting COM1 to 115200 baud..'
  port.write('com com1 115200 n 8 1 n\r\n')

cur_baud = 115200
print 'Reopening at 115200 baud...'
with Serial(port_name, cur_baud, timeout=0.3) as port:
  print 'setting COM2 to 9600 baud...'
  port.write('com com2 9600 n 8 1 n\r\n')
  print port.readall()
  print 'setting interface mode for COM2...'
  port.write('rtksource rtcmv3 any\r\n')
  port.write('interfacemode com2 rtcmv3 rtcmv3 on\r\n')
  print port.readall()
