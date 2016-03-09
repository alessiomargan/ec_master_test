#! /usr/bin/env python
import time
p=open('/proc/xenomai/registry/rtipc/xddp/terminal','a',0)
p.write("1")
time.sleep(0.5)
p.close()
