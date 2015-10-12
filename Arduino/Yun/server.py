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
  while 1:
    try:
      data = c.recv(BUFFER_SIZE)
    except IOError:
      q.put('C')
      break
    
    if not data:
      q.put('C')
      break
    for line in string.split(data, '\n'):
      if not line: break
      #print " Command:",line
      cmdCount = cmdCount + 1
      q.put(line+'\n')
    q.put('A')

try:
  t=serial.Serial('/dev/ttyATH0',500000,timeout=60)
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
  thread.start_new_thread( receive, (conn,))

  rsp=''

  if not t:
    while 1:
      cmd = q.get()
      if( cmd=='C' ): break
      if( cmd=='A' ):
        try:
          conn.send(rsp)
          rsp=''
        except socket.error, msg:
          break
      else:
        sys.stdout.write("%03d %s     \r" % (q.qsize(),cmd.rstrip('\n')))
        sys.stdout.flush()
        if( cmd=='RST\n' ) :
          rsp = 'HLO'
        else : 
          rsp=rsp+'O'
          time.sleep(0.01)

  else:
    t.flushOutput()
    t.flushInput()
    while 1:
      cmd = q.get()
      if( cmd=='C' ):break
      if( cmd=='A' ):
        try:
          #print " Ack:",rsp
          conn.send(rsp)
          rsp=''
        except socket.error, msg:
          break
      else:
        # print ' Command:',cmd
        retry = 0
        while 1:
          t.write(cmd)        
          c=t.read(1)
          if( c != 'E' ): break
          time.sleep(0.1)
          t.flushInput()
          print 'Command Retry:',cmd
          retry=retry+1
          if( retry >= 3 ): break

        if( c == 'E' ):
          print 'Retry failed.'
          break
        
        if( c == 'O' ): 
          rsp=rsp+c
          ackCount = ackCount + 1
        else: 
          while(c != '\n'):
            #print 'Got:'+c
            rsp=rsp+c
            c=t.read(1)
          ackCount = ackCount + 1

      # sys.stdout.write("%06d-%06d %d \r" % (cmdCount,ackCount,cmdCount-ackCount))
      # sys.stdout.flush()

  while not q.empty(): q.get()
  conn.close()
  conn=None
  connected=False
  print '\nConnection closed.'
