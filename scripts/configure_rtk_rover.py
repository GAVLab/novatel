#!/usr/bin/env python
"""
  python configure_novatel_rtk_base.py [port] [current baud]

connect to COM1 on NovAtel
first do a factory reset on the receiver

"""
import sys
from serial import Serial
from time import sleep

port_name = sys.argv[1]
cur_baud = int(sys.argv[2])
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
