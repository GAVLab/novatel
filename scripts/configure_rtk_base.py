#!/usr/bin/env python
"""
Configure a NovAtel OEMV series to be a static RTK base station. Should work on
just about any RTK-capable NovAtel, but this is only tested on the ProPakv3.
It is assumed that we are connected to the receiver on COM1 and that corrections 
will be broadcast throught COM2 @ 9600 baud. First the receiver is reconfigured
to communicate at 115200 baud on COM1, regardless of current setting. RTCMv3 
correction format will be used.

Base station elevation must be MSL (not ellipsoidal)!!

It is suggested that the receiver be reset prior to running this script, but 
that is not strictly necessary. This is only due to the fact that the way in 
which various modes and smoothing. Factory reset may be performed by issuing the
following command in a serial terminal:
  
  freset standard

Responses from the receiver are printed to screen. Successful commands return
the following (when connected to COM1 on the receiver):


  <OK
  [COM1]

Usage: run `configure_rtk_base.py -h` for current usage.

The known station names are shown below


Robert Cofield
2016-03-28
"""
import sys
from serial import Serial
from time import sleep
import argparse


base_station_locations = {
  
  # Tyler Sherer surveyed this location
  'woltosz_pinwheel':    (32.605363688183651, -85.486936034004174, 201.7449376863390),

  # Tyler Sherer surveyed this location
  'woltosz_patch':       (32.605379434999030, -85.486936383548795, 201.6900996648771),

  # Brently Nelson surveyed this over only an hour. The accuracy may be off
  'ncat_1hr_avg':        (32.59542536,        -85.29604671,        202.8817096),

  # Taken by Willis Walker and Brently Nelson in 2014 using a Trimble survey 
  # unit which was connected to the Auburn ALA1 CORS server. The accuracy may 
  # be off since that constitutes a ~10 km distance.
  'ncat_trimble_survey': (32.59541973313201,  -85.29604294782064,  202.8817096)

}

# parse command line arguments
parser = argparse.ArgumentParser(description="Configure a NovAtel RTK base station")
parser.add_argument('name', default=None, help="""The code name of the base station. 
Options are: %s ..
If one of these is not supplied, a manual location for the base station using 
decimal degrees and MSL elevation is possible. Use the syntax `lat,lon,alt`
""" % (', '.join(base_station_locations.keys()))
)
parser.add_argument('-p', '--port', dest='port', default='/dev/ttyUSB0', help="What serial port COM1 of the NovAtel is plugged into. (Default /dev/ttyUSB0)")
parser.add_argument('-b', '--baud', dest='baud', default=9600, help="What baud rate to use in communicating with the NovAtel on the receiver's COM1. (Default 9600)")
args = parser.parse_args()

port_name = args.port
cur_baud = args.baud

if args.name not in  base_station_locations:
  print 'Valid base station name not given, checking to see if a valid position was supplied instead.'
  try:
    loc = tuple([float(val) for val in args.name.split(',')])
  except:
    print 'Valid location for base station was not provided.'
else:
  loc = base_station_locations[args.name]


print 'initially using port', port_name, 'at', cur_baud, 'baud'
with Serial(port_name, cur_baud, timeout=0.1) as port:
  print 'Unlogging...'
  port.write('unlogall\r\n')
  rsp = port.readall()
  print rsp
  if rsp != '\r\n<OK\r\n[COM1]':
    raise Exception('Unexpected response. Try running again, or changing the baud rate.')
  print 'Setting COM1 to 115200 baud..'
  port.write('com com1 115200 n 8 1 n\r\n')

cur_baud = 115200
print 'Reopening at 115200...'
with Serial(port_name, cur_baud, timeout=0.3) as port:
  print 'setting COM2 to 9600 baud'
  port.write('com com2 9600 n 8 1 n\r\n')
  print port.readall()
  print 'setting interface mode for COM2...'
  port.write('interfacemode com2 rtcmv3 rtcmv3 on\r\n')
  print port.readall()
  print 'setting RTK base station LLA coordinates for %s:' % (args.name)
  port.write('fix position %f %f %f\r\n' % loc)
  print port.readall()
  print 'Requesting outgoing RTCMv3 logs on COM2...'
  port.write('log com2 rtcm1004 ontime 1 0 hold\r\n')
  port.write('log com2 rtcm1005 ontime 1 0 hold\r\n')
  print port.readall()