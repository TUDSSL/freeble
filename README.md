# Freeble
This repository contains the source code used to enable the expiriments of the paper that is published here:
- Link

## Compiler
In order to compile the source you need the gcc-arm embedded compiler. We use the 4.9-2015-q1 version.
https://launchpad.net/gcc-arm-embedded/4.9/4.9-2015-q1-update/+download/gcc-arm-none-eabi-4_9-2015q1-20150306-linux.tar.bz2

Goto the download directry and extract all files to /usr/local with the command:
```
tar -xjvf gcc-arm-none-eabi-4_9-2015q1-20150306-linux.tar.b -C /usr/local
```
Use `sudo` if you do not have rights to the /usr/local folder.

## Debug and firmware upload requirements
To upload the binary to the nRF51822 device we use a J-LINK device from SEGGER and we also use the J-LINK software on a Linux Ubuntu OS. Download and install the the Software and documentation pack from:
https://www.segger.com/jlink-software.html

To upload binaries to the nRF51822 the user must have acces to the serialport or USB port. First check which group is owner of the port. It should be dailout.
```
ls -l /dev/tty*
```
then 
```
sudo adduser <username> <group>
```
if the group does not exists add the group to the system
```
sudo groupadd dialout
```

## Build and upload
To ensure make kan open a new gnome terminal window and switch back to this window you need to install wmctrl:
`sudo apt-get -y install wmctrl`

Now you can compile and upload the program. You can compile for four different  
targets:
 - Mobile Node
 - Anchor Node
 - WPA Node
 - Sniffer

You first need to modify the make file to the right target and then you can compile.

First clean your build:
`make clean`

Then compile:
`make all`

Then make sure your J-LINK device connected correctly and type:
`make upload` 

Now your device is programed with your new program. If something goes wrong you can erease the device with:
`make erase-all`

For GDB debuging you can use:
`make gdb`
Which creates a GDB server and you can perform step and trace trough the program.

Finally also the RTT of SEGGER is implemented and you can use the printf command in your program. You can see the debug output on the RTT vieuwer SEGGER or use our python script that also saves the logs to a script. You can view the debug output with the command:
`./rtt.py`

It makes a telnet connection to the RTT interface of the J-LINK device and saves all data in log.txt

## Redo expiriments
For every experiment the code is changed a bit to get the correct results. To redo the **wiploc_basic OR deadzone experiment** the following has to be done:
- Change the name of the following files and compile and upload them to the devices:
```
mobile_node\main_mobile_node.c to mobile_node\main_mobile_node.c.bak
mobile_node\main_mobile_node.c.wiploc_basic to mobile_node\main_mobile_node.c

sniffer\sniffer.c to sniffer\sniffer.c.bak
sniffer\sniffer.c.wiploc_basic to sniffer\sniffer.c
```

To redo the **wiploc_plus** experiment the following has to be done:
- Change the name of the following files and compile and upload them to the devices
```
mobile_node\main_mobile_node.c to mobile_node\main_mobile_node.c.bak
mobile_node\main_mobile_node.c.wiploc_plus to mobile_node\main_mobile_node.c

sniffer\sniffer.c to sniffer\sniffer.c.bak
sniffer\sniffer.c.wiploc_plus to sniffer\sniffer.c
```	

To redo the **wiploc_plus_twowave** experiment the following has to be done:
- Change the name of the following files and compile and upload them to the devices
```
mobile_node\main_mobile_node.c to mobile_node\main_mobile_node.c.bak
mobile_node\main_mobile_node.c.wiploc_plus_twowave to mobile_node\main_mobile_node.c

sniffer\sniffer.c to sniffer\sniffer.c.bak
sniffer\sniffer.c.wiploc_plus_twowave to sniffer\sniffer.c
```

To redo the **wpt_rssi** experiment the following has to be done:
- Add the following target to the Makefile:
```
SRC_DIR = wpa_node
```

Comment out the rest of the targets and compile the code. Use the same hardware as for a Mobile Node. Upload the code to the hardware node.

This node now becomes a data logger that directly prints data via the Segger RTT.
Every mesurement you have 5 seconds to move the position of the Mobile Node.
The receiving node can just be a normal powered Anchor Node.