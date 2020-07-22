#### Toolchain commands
### ----------------------------------------------------------------
GCC_INSTALL_DIR  := /usr/local
GCC_INSTALL_ROOT := $(GCC_INSTALL_DIR)/gcc-arm-none-eabi-4_9-2015q1
GCC_VERSION      := 4.9.3
GCC_PREFIX       := arm-none-eabi

CC      := $(GCC_INSTALL_ROOT)/bin/$(GCC_PREFIX)-gcc
AS      := $(GCC_INSTALL_ROOT)/bin/$(GCC_PREFIX)-as
AR      := $(GCC_INSTALL_ROOT)/bin/$(GCC_PREFIX)-ar -r
LD      := $(GCC_INSTALL_ROOT)/bin/$(GCC_PREFIX)-ld
NM      := $(GCC_INSTALL_ROOT)/bin/$(GCC_PREFIX)-nm
OBJDUMP := $(GCC_INSTALL_ROOT)/bin/$(GCC_PREFIX)-objdump
OBJCOPY := $(GCC_INSTALL_ROOT)/bin/$(GCC_PREFIX)-objcopy
GDB     := $(GCC_INSTALL_ROOT)/bin/$(GCC_PREFIX)-gdb
#CGDB    := "/usr/bin/cgdb"

echo = echo
PWD = `pwd`

MK	:= mkdir -p
RM	:= rm -rf

### General Variables
OUTPUT_NAME = main
OUTPUT_DIR  = build
OBJ_DIR     = obj
LINK_DIR    = link
ASM_DIR    = asm


### Switch here between targets
### ----------------------------------------------------------------
#SRC_DIR = mobile_node
SRC_DIR = wpa_node
#SRC_DIR = anchor_node
#SRC_DIR = sniffer

## Set the Acnhor ID of your target
BEACON_NUMBER   = 3


### Device related stuff
### ----------------------------------------------------------------
BOARD        := BOARD_PCA20006
CPU          := cortex-m0
DEVICE       := NRF51
DEVICESERIES := nrf51

BLE          := BLE_STACK_SUPPORT_REQD


### Programmer
### ----------------------------------------------------------------
### Make sure you install Jlink from the segger website!
JLINK_DIR       = /opt/SEGGER/Jlink
JLINK           = $(JLINK_DIR)/JLinkExe
JLINKGDBSERVER	= $(JLINK_DIR)/JLinkGDBServer
JLINKRTT        = $(JLINK_DIR)/JLinkRTTClient
GDB_PORT_NUMBER = 3333
JLINK_DEVICE_NAME = NRF51822
JLINK_SPEED     = 1000
JLINK_INTERFACE = SWD

# We dont use this, but it can work.
SOFTDEVICE      = softdevice/s110_nrf51822_6.0.0_softdevice.hex

# Include directories
INCLUDEDIRS	 = $(shell find nrf51_sdk/old_sdk -type d)


### Source files
### ----------------------------------------------------------------
C_SRC  = $(shell find $(SRC_DIR) -name *.c | awk -F/ '{print $$NF}')

### Assembly source files
ASSEMBLY_SRC = gcc_startup_$(DEVICESERIES).s

### Compiler related stuff
### ----------------------------------------------------------------
CFLAGS  = -ggdb	#info for the debugger
CFLAGS	+= -O0	#debugging friendly
CFLAGS	+= -mcpu=$(CPU)
CFLAGS	+= -mthumb
CFLAGS	+= -mabi=aapcs
CFLAGS	+= --std=gnu99
CFLAGS	+= -Wall -fdiagnostics-color=always
CFLAGS	+= -D$(DEVICE)
CFLAGS	+= -D$(BOARD)
CFLAGS	+= -D$(BLE)
CFLAGS	+= -DDEBUG   ## This enables the standaard RTT debug
#CFLAGS	+= -DUART_DEBUG  ## This enables also the UART debug.
CFLAGS	+= -DBEACON=$(BEACON_NUMBER)
CFLAGS	+= $(patsubst %,-I%, $(INCLUDEDIRS))

### Linker related stuff
### ---------------------------------------------------------
LDDIRS 	 = $(GCC_INSTALL_ROOT)/$(GCC_PREFIX)/lib/armv6-m
LDDIRS 	+= $(GCC_INSTALL_ROOT)/lib/gcc/$(GCC_PREFIX)/$(GCC_VERSION)/armv6-m
LDDIRS	+= $(LINK_DIR)

## Use other linker script if you also want to add the softdevice!
LD_SCRIPT = $(LINK_DIR)/gcc_nrf51_blank_xxaa.ld
#LD_SCRIPT = $(LINK_DIR)/gcc_nrf51_s110_xxaa.ld
#LD_SCRIPT = $(LINK_DIR)/gcc_nrf51_s110_8.0.0_xxaa.ld

LDFLAGS = -Xlinker
LDFLAGS += -Map=$(OUTPUT_DIR)/$(OUTPUT_NAME).map
LDFLAGS += -mcpu=$(CPU)
LDFLAGS += -mthumb
LDFLAGS += -mabi=aapcs
LDFLAGS += -fdiagnostics-color=always
LDFLAGS += -T$(LD_SCRIPT)
LDFLAGS	+= -D$(DEVICE)
LDFLAGS	+= -D$(BOARD)
LDFLAGS += --specs=nano.specs
LDFLAGS	+= $(patsubst %,-L%, $(LDDIRS))

# Sorting removes duplicates
BUILD_DIRS := $(sort $(OBJ_DIR) $(OUTPUT_DIR) $(ASM_DIR) )

# Make a list of source paths
C_SRC_DIRS = $(shell find $(SRC_DIR) -type d)
ASSEMBLY_SRC_DIRS = $(shell find $(SRC_DIR) -type d)

# Object files
C_OBJ        = $(addprefix $(OBJ_DIR)/, $(C_SRC:.c=.o))
ASSEMBLY_OBJ = $(addprefix $(OBJ_DIR)/, $(ASSEMBLY_SRC:.s=.o))

# Set source lookup paths
vpath %.c $(C_SRC_DIRS)
vpath %.s $(ASSEMBLY_SRC_DIRS)

# Include automatically previously generated dependencies
-include $(addprefix $(OBJ_DIR)/, $(C_OBJ:.o=.d))

### Rules
###
# Default build target
.PHONY : all
all : release ##rename-and-zip

clean:
	@echo "Removing build and object files, folders and scripts!"
	@$(RM) $(OUTPUT_DIR)/*
	@$(RM) $(OBJ_DIR)/*
	@- $(RM) JLink.log
	@- $(RM) gdb_init_script.gdb

.PHONY: release

#release :  CFLAGS += -DNDEBUG -O3
release: $(OUTPUT_DIR)/$(OUTPUT_NAME).bin $(OUTPUT_DIR)/$(OUTPUT_NAME).hex   ##rm.jlink upload.jlink

mobile_node: $(BUILD_DIRS)
	@echo "Make mobile-node device!"
	@- $(MK) $(OUTPUT_DIR)/$@
	@- $(MK) $(OBJ_DIR)/$@

beacon:


flash: release upload gdb

# Create Output dirs
$(BUILD_DIRS) :
	@echo "Creating directories"
	- $(MK) $@

# Create objects from C source files
$(OBJ_DIR)/%.o : %.c
	@#@echo "Build header dependencies for file: " $<
	@ $(CC) $(CFLAGS) -M $< -MF "$(@:.o=.d)" -MT $@
	@echo "Compiling: " $<
	@ $(CC) $(CFLAGS) -c -o $@ $<

## Assemble .s files
$(OBJ_DIR)/%.o : %.s
	@echo
	@echo "Compiling: " $<
	@ $(CC) $(patsubst %,-I%, $(INCLUDEDIRS)) -c -o $@ $<


## Link C and assembler objects to an .elf file
$(OUTPUT_DIR)/$(OUTPUT_NAME).elf : $(BUILD_DIRS) $(C_OBJ) $(ASSEMBLY_OBJ)
	@echo
	@echo "Linking object files: "
	@echo $(C_OBJ) $(ASSEMBLY_OBJ)
	@ $(CC) $(LDFLAGS) $(C_OBJ) $(ASSEMBLY_OBJ) -lm -o $(OUTPUT_DIR)/$(OUTPUT_NAME).elf

## Create binary .bin file from the .elf file
$(OUTPUT_DIR)/$(OUTPUT_NAME).bin : $(OUTPUT_DIR)/$(OUTPUT_NAME).elf
	@echo
	@echo "Create binary(.bin) file from: " $<
	@ $(OBJCOPY) -O binary $(OUTPUT_DIR)/$(OUTPUT_NAME).elf $(OUTPUT_DIR)/$(OUTPUT_NAME).bin

## Create binary .hex file from the .elf file
$(OUTPUT_DIR)/$(OUTPUT_NAME).hex : $(OUTPUT_DIR)/$(OUTPUT_NAME).elf
	@echo "Create binary(.hex) file from: " $<
	@ $(OBJCOPY) -O ihex $(OUTPUT_DIR)/$(OUTPUT_NAME).elf $(OUTPUT_DIR)/$(OUTPUT_NAME).hex


## UPLOADING STUFF

## Program device
upload: stopgdb rm.jlink upload.jlink
	@echo "--------------------------------------------------------------------"
	@echo "Starting uploading application..."
	@echo "--------------------------------------------------------------------"
	$(JLINK) -If $(JLINK_INTERFACE) -Speed $(JLINK_SPEED) -Device $(JLINK_DEVICE_NAME) -CommanderScript $(OUTPUT_DIR)/upload.jlink

## Program device
upload-dongleV2: stopgdb rm.jlink upload-specific.jlink
	@echo "--------------------------------------------------------------------"
	@echo "Starting uploading application to Blue Dongle..."
	@echo "--------------------------------------------------------------------"
	$(JLINK) -SelectEmuBySN 680718389 -If $(JLINK_INTERFACE) -Speed $(JLINK_SPEED) -Device $(JLINK_DEVICE_NAME) -CommanderScript $(OUTPUT_DIR)/upload-specific.jlink

rm.jlink:
	@echo "Remove *.jlink files..."
	-rm -rf $(OUTPUT_DIR)/*.jlink

## Ask which programmer to uzse fro programming by "selemu"
# Set w4 4001e504(NVMC config register) to 1, to enable writing the flash memory.
upload.jlink:
	@$(echo) "selemu\n\
	r\n\
	h\n\
	w4 4001e504 1\n\
	loadbin $(PWD)/$(OUTPUT_DIR)/$(OUTPUT_NAME).bin $(shell $(OBJDUMP) -h $(PWD)/$(OUTPUT_DIR)/$(OUTPUT_NAME).elf -j .text | grep .text | awk '{print $$4}')\n\
	r\n\
	g\n\
	q\n\
	" > $(OUTPUT_DIR)/$@

upload-specific.jlink:
	@$(echo) "r\n\
	h\n\
	w4 4001e504 1\n\
	loadbin $(PWD)/$(OUTPUT_DIR)/$(OUTPUT_NAME).bin $(shell $(OBJDUMP) -h $(PWD)/$(OUTPUT_DIR)/$(OUTPUT_NAME).elf -j .text | grep .text | awk '{print $$4}')\n\
	r\n\
	g\n\
	q\n\
	" > $(OUTPUT_DIR)/$@

upload-softdevice: upload-softdevice.jlink stopgdb
	@echo "Convert from hex to binary. Split original hex in two to avoid huge (>250 MB) binary file with just 0s. "
	$(OBJCOPY) -Iihex -Obinary --remove-section .sec3 $(SOFTDEVICE) $(OUTPUT_DIR)/_mainpart.bin
	$(OBJCOPY) -Iihex -Obinary --remove-section .sec1 --remove-section .sec2 $(SOFTDEVICE) $(OUTPUT_DIR)/_uicr.bin
	$(JLINK) $(OUTPUT_DIR)/upload-softdevice.jlink

upload-softdevice.jlink:
	@echo "--------------------------------------------------------------------"
	@echo "Upload the softdevice! Enable writing, load mainpart bin, load uicr bin. Reset."
	@echo "--------------------------------------------------------------------"
	@$(echo) "w4 4001e504 1\n\
	loadbin \"$(OUTPUT_DIR)/_mainpart.bin\" 0\n\
	loadbin \"$(OUTPUT_DIR)/_uicr.bin\" 0x10001000\n\
	r\n\
	g\n\
	qc\n\
	" > $(OUTPUT_DIR)/upload-softdevice.jlink

## Erase-all stuff

erase-all: erase-all.jlink
	$(JLINK) -If $(JLINK_INTERFACE) -Speed $(JLINK_SPEED) -Device $(JLINK_DEVICE_NAME) -CommanderScript $(OUTPUT_DIR)/erase-all.jlink

erase-all.jlink:
	@echo "--------------------------------------------------------------------"
	@echo "Erase the complete device."
	@echo "--------------------------------------------------------------------"
	@$(echo) "selemu\n\
	w4 4001e504 2\n\
	w4 4001e50c 1\n\
	w4 4001e514 1\n\
	r\n\
	qc\n\
	" > $(OUTPUT_DIR)/$@

## GDB STUFF

gdb: stopgdb gdb_init_script.gdb
	@echo "Open a seperate window for the JlinkGDBServer otherwise the CRTL+C command also kills te server."
	@wmctrl -r :ACTIVE: -N "Make GDB client"
	@gnome-terminal --command="$(JLINKGDBSERVER) -if $(JLINK_INTERFACE) -speed $(JLINK_SPEED) -device $(JLINK_DEVICE_NAME) -port $(GDB_PORT_NUMBER)" --title="JlinkGDBServer"
	@sleep 1
	@echo ""
	@wmctrl -a "Make GDB client"
	@ $(GDB) -x gdb_init_script.gdb $(OUTPUT_DIR)/$(OUTPUT_NAME).elf

gdb-dongle: stopgdb gdb_init_script.gdb
	@echo "Open a seperate window for the JlinkGDBServer otherwise the CRTL+C command also kills te server."
	@wmctrl -r :ACTIVE: -N "Make GDB client"
	@gnome-terminal --command="$(JLINKGDBSERVER) -if $(JLINK_INTERFACE) -speed $(JLINK_SPEED) -device $(JLINK_DEVICE_NAME) -select usb=480107367 -port $(GDB_PORT_NUMBER)" --title="$(JLINKGDBSERVER)"
	@sleep 1
	@echo ""
	@wmctrl -a "Make GDB client"
	@ $(GDB) -x gdb_init_script.gdb $(OUTPUT_DIR)/$(OUTPUT_NAME).elf

gdb-dongleV2: stopgdb gdb_init_script.gdb
	@echo "Open a seperate window for the JlinkGDBServer otherwise the CRTL+C command also kills te server."
	@wmctrl -r :ACTIVE: -N "Make GDB client"
	@gnome-terminal --command="$(JLINKGDBSERVER) -if $(JLINK_INTERFACE) -speed $(JLINK_SPEED) -device $(JLINK_DEVICE_NAME) -select usb=680718389 -port $(GDB_PORT_NUMBER)" --title="$(JLINKGDBSERVER)"
	@sleep 1
	@echo ""
	@wmctrl -a "Make GDB client"
	@ $(GDB) -x gdb_init_script.gdb $(OUTPUT_DIR)/$(OUTPUT_NAME).elf

start-gdb-server: stopgdb gdb_init_script.gdb
	@echo "Use this window for the JlinkGDBServer."
	$(JLINKGDBSERVER) -if $(JLINK_INTERFACE) -speed $(JLINK_SPEED) -device $(JLINK_DEVICE_NAME) -port $(GDB_PORT_NUMBER)

start-gdb: stopgdb gdb_init_script.gdb
	@echo "GDB Client..."
	@ $(GDB) -x gdb_init_script.gdb $(OUTPUT_DIR)/$(OUTPUT_NAME).elf

stopgdb:
	-killall $(JLINKGDBSERVER)

gdb_init_script.gdb:
	@$(echo) "target remote localhost:$(GDB_PORT_NUMBER)\n\
	monitor reset\n\
	" > $@
	@$(echo) "define rst\nmonitor reset\nend\n" >> $@


## TOOLCHAIN STUFF

install-all64: install-clibs-32 install-gcc-arm install-jlink64 install-wmctrl
	@echo "Installing complete toolchain..."

install-all32: install-gcc-arm install-jlink32 install-wmctrl
	@echo "Installing complete toolchain..."

install-gcc-arm: 
	@echo "Dowloading gcc arm none eabi 4.9.3 2015 q1"
	wget -O ~/Downloads/gcc-arm-none-eabi-4_9-2015q1-20150306-linux.tar.bz2 "https://launchpadlibrarian.net/200701245/gcc-arm-none-eabi-4_9-2015q1-20150306-linux.tar.bz2"
	cd $(GCC_INSTALL_DIR)/ && sudo tar -xjf ~/Downloads/gcc-arm-none-eabi-4_9-2015q1-20150306-linux.tar.bz2

install-clibs-32:
	apt-get -y install lib32z1 lib32ncurses5 lib32bz2-1.0

install-jlink64:
	@echo "Extracting  jlink to /opt/SEGGER"
	mkdir /opt/SEGGER
	tar -xvzf toolchain/JLink_Linux_V502f_x86_64.tgz -C /opt/SEGGER/
	mv /opt/SEGGER/JLink_Linux_V502f_x86_64 $(JLINK_DIR)

install-jlink32:
	@echo "Extracting  jlink to /opt/SEGGER"
	mkdir /opt/SEGGER
	tar -xvzf toolchain/JLink_Linux_V502f_i386.tgz -C /opt/SEGGER/
	mv /opt/SEGGER/JLink_Linux_V502f_i386 $(JLINK_DIR)

install-wmctrl:
	apt-get -y install wmctrl


## DEBUG STOFF RTT
rtt:
	@gnome-terminal --command="$(JLINKRTT)" --title="JLinkRTTClient"


## OVER The air firmware update
rename-and-zip:
	@cp build/main.hex build/application.hex
	@python crc_calc.py
	@zip -j build/application.zip build/application.hex build/application.dat
	@thunderbird -compose "to='', subject='firmware', body='frimware', attachment='build/appilcation.zip' " &


.PHONY: upload upload-softdevice erase-all gdb stopgdb rtt uart
