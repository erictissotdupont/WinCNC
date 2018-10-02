#!/usr/bin/python
import os
import socket
import sys
import serial
import time
import thread
import string
import Queue

from time import sleep

# http://svn.python.org/projects/python/branches/pep-0384/Demo/sockets/
# https://wiki.python.org/moin/TcpCommunication


# Arbitrary non-privileged port
BROADCAST_PORT = 50042
DATA_PORT = 50043
BUFFER_SIZE = 1024
connected=False
q = Queue.Queue()

def broadcast( ):
  b = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  b.bind(('', 0))
  b.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
  print 'CNC Broadcasting '
  while 1:
      if not connected:
          data = 'Cnc 2.0'
          b.sendto(data, ('<broadcast>', BROADCAST_PORT))
          sys.stdout.write('.')
          sys.stdout.flush()
      time.sleep(2)

def receive( c ):
  global cmdCount
  cmdCount = 0
  remainder = ''

  while connected:
    try:
      data = remainder + c.recv(BUFFER_SIZE)
      remainder = ''
    except IOError:
      print 'IO Error from host.'
      break
    if not data:
      print '\n\nWARNING: No data from host.'
      break

    # If we received a command sliced between two frames (new line is missing at the end)
    if (data[-1]!='\n'):
      # Find the last complete command
      last = data.rfind('\n')
      if( last > -1 ):
        # Save the incomplete command
        remainder = data[last+1:]
        # Only keep complete commands in the data buffer
        data = data[0:last]

    for line in string.split(data, '\n'):
      if not line:
        # Got empty line from host. Just ignore.
        break
      cmdCount = cmdCount + 1
      q.put(line+'\n')

    # Insert a request to send agregated acknowledge from ATMega back to PC
    q.put('A')

  # This will close the connection
  q.put('C')

print '\n------------------------------------------------------'
print 'CNC Server started'
try:
# This is for Arduino Yu
  t=serial.Serial('/dev/ttyATH0',500000,timeout=10)
  print 'HW is Arduino Yu'
  t.flushOutput()
  t.flushInput()
except IOError:
  try:
    # This is for Raspbery Pi 3 Model B
    t=serial.Serial('/dev/ttyS0',500000,timeout=10)
    print 'HW is Raspberry Pi 3'
    t.flushOutput()
    t.flushInput()
  except IOError:
    print 'WARNING: Serial port not found.'
    t=None

thread.start_new_thread( broadcast, ( ))

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind(('', DATA_PORT))

while 1:
  ackCount = 0 
  s.listen(1)
  conn, addr = s.accept()
  print '\nConnection from:', addr
  connected=True
  q.queue.clear()
  thread.start_new_thread( receive, (conn,))

  rsp=''

  t.flushOutput()
  t.flushInput()
  while 1:
    cmd = q.get()

    if( cmd=='C' ):
      # The termination request comes from receiving thread
      break
    if( cmd=='A' ):
      # This a request to send agreagated response from ATMega to the host
      try:
        conn.send(rsp)
	print 'Sent to host :', rsp
        rsp=''
      except socket.error, msg:
        print 'Socket error sending to host.'
		# This will cause the socket to be closed
        break
    else:
      # Everything else gets sent down to the ATMega expecting a response
      retry = 0
      c = 'C'
      while( c == 'C' and retry < 3 ):
        t.write(cmd)
        c = t.read(1)
        retry = retry + 1
      if not c:
        print 'Response timeout.'
        break
      if( c == 'C' ):
        # CRC error, retries failed
        print 'CRC error.'
        break
      if( c == 'O' or c == 'E' ):
        # Command OKAYed or ERROR
        rsp=rsp+c
        ackCount = ackCount + 1
      else:
        # String response. Wait for line return.
        while 1:
          if not c:
            print 'Response timeout (string).'
            break
          rsp=rsp+c
          if c == '\n':
            break
          c=t.read(1)
        if c: ackCount = ackCount + 1

    sys.stdout.write("%06d-%06d %d\r" % (cmdCount,ackCount,cmdCount-ackCount))
    sys.stdout.flush()

  while not q.empty(): q.get()
  conn.close()
  conn=None
  connected=False
  print 'Connection closed.'
