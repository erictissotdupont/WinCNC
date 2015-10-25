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
BUFFER_SIZE = 2000
connected=False
q = Queue.Queue()

def broadcast( ):
  b = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  b.bind(('', 0))
  b.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
  print 'Broadcasting...'
  while 1:
      if not connected:
          data = 'Cnc 1.0'
          b.sendto(data, ('<broadcast>', BROADCAST_PORT))
      time.sleep(2)

thread.start_new_thread( broadcast, ( ))

def receive( c ):
  global cmdCount
  cmdCount = 0
  while connected:
    try:
      data = c.recv(BUFFER_SIZE)
    except IOError:
      print 'IO Error from host.'
      break
    if not data:
      print 'No data from host.'
      break

    for line in string.split(data, '\n'):
      if not line:
        # Got empty line from host. Just ignore.
        break
      cmdCount = cmdCount + 1
      q.put(line+'\n')
    q.put('A')

  # This will close the connection
  q.put('C')

try:
  t=serial.Serial('/dev/ttyATH0',500000,timeout=10)
  t.flushOutput()
  t.flushInput()
except IOError:
  print 'WARNING: Serial port not found. Debug mode enabled.'
  t=None

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind(('', DATA_PORT))

while 1:
  ackCount = 0 
  s.listen(1)
  conn, addr = s.accept()
  print 'Connection from:', addr
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
        rsp=''
      except socket.error, msg:
        print 'Socket error sending to host.'
		# This will cause the socket to be closed
        break
    else:
      # Everything else gets sent down to the ATMega expecting a response
      retry = 0
      t.write(cmd)
      c=t.read(1)
      if not c:
        print 'Response timeout.'
        break
        
      if( c == 'O' or c == 'E' ):
        # Command OKAYed or ERROR
        rsp=rsp+c
        ackCount = ackCount + 1
      else:
        # String response. Wait for line return.
        while 1:
          if not c:
            print 'Response timeout.'
            break
          rsp=rsp+c
          if c == '\n':
            break
          c=t.read(1)
        if c: ackCount = ackCount + 1

    sys.stdout.write("%06d-%06d %d \r" % (cmdCount,ackCount,cmdCount-ackCount))
    sys.stdout.flush()

  while not q.empty(): q.get()
  conn.close()
  conn=None
  connected=False
  print '\nConnection closed.'
