#!/usr/bin/env python

import getpass
import sys
import telnetlib
import signal
from datetime import datetime
from time import strftime

HOST = "localhost"
PORT = "19021"

filename = "log.txt"
method = 'aw'

if(len(sys.argv) > 1):
	filename = sys.argv[1]

if(len(sys.argv) > 2):
	method = sys.argv[2]

fp = open(filename, method)

#date_time = strftime("%Y-%m-%d %H:%M:%S")
#string = "\n----------------------------------\n" + date_time + "\n----------------------------------\n"
#fp.write(string)
#print string
print "\nConnecting with Jlink/GDB Server....\n"

while True:
	try:
		tn = telnetlib.Telnet(HOST, PORT)

		while True:
			line = tn.read_eager()
			if (len(line)>0 and line!=" "):
				sys.stdout.write(line)
				fp.write(line)
	except KeyboardInterrupt:
		fp.close()
		tn.close()
		sys.exit(0)
	except:
		continue

