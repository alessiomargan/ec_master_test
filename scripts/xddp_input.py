#! /usr/bin/env python3
import time
import sys

try :
    p=open('/proc/xenomai/registry/rtipc/xddp/EC_board_input','w')
except IOError as e :
    p=open('/tmp/EC_board_input','w')

while True :
    c = input('%>>')
    p.write(c)
    p.flush()
    if c == 'Q' :
        break

p.close()
