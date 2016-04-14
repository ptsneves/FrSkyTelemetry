frksky_telemetry_mkfile_dir:=$(dir $(lastword $(MAKEFILE_LIST)))
SHELL = /bin/bash
PROJECT_DIRECTORY:=$(shell git rev-parse --show-toplevel)/Firmware/Source
port:=/dev/ttyUSB0
upload_speed:=57600
upload_protocol:=arduino
mcu:=atmega328p
cpu_frequency:=16000000
board_variant:=eightanaloginputs

#location of the arduino installation, set for default in debian.
#Arduino libraries are not considered for this core make file
#This makefile generates a library called libarduinocore.a
arduino_dir:=/usr/share/arduino/
arduino_core_directory:=$(arduino_dir)hardware/arduino/cores/arduino/
arduino_variants_directory:=$(arduino_dir)hardware/arduino/variants/

avr_core_src_c:=wiring.c \
	wiring_analog.c wiring_digital.c \
	wiring_pulse.c \
	wiring_shift.c WInterrupts.c
	

avr_core_src_cpp:=HardwareSerial.cpp WMath.cpp Print.cpp CDC.cpp

avr_core_obj_dir:=$(frksky_telemetry_mkfile_dir)/core
avr_core_cpp_obj:=$(patsubst %.cpp, $(avr_core_obj_dir)/%.o, $(avr_core_src_cpp))
avr_core_c_obj:=$(patsubst %.c, $(avr_core_obj_dir)/%.o, $(avr_core_src_c))

avr_core_inc:=-I$(arduino_core_directory) -I$(arduino_variants_directory)$(board_variant)

arduino_static_library_name:=arduinocore
arduino_static_library:=$(avr_core_obj_dir)/lib$(arduino_static_library_name).a

#See arduino/hardware/arduino/avr/platform.txt for compilation flags
AVR_CPPFLAGS:=-c -O2 -g -mmcu=$(mcu) -DF_CPU=$(cpu_frequency) -DARDUINO=105 -D__PROG_TYPES_COMPAT__ \
	-fno-exceptions -ffunction-sections -fdata-sections \
	-funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -fno-threadsafe-statics \
	-std=c++11 -Wall
AVR_LFLAGS:= -mmcu=$(mcu) -Os  -Wl,--gc-sections
AVR_INC:=$(avr_core_inc)
AVR_CC:=avr-g++
AVR_OBJCOPY:=avr-objcopy
AVR_OBJDUMP:=avr-objdump
AVR_AR:=avr-ar
AVR_SIZE:=avr-size
AVR_NM:=avr-nm

AVRDUDE:=avrdude
AVRDUDE_MCU:=ATmega328p
AVRDUDE_WRITE_FLASH = -U flash:w:
AVRDUDE_FLAGS = -q -D -F \
    -p $(AVRDUDE_MCU) -P $(port) -c $(upload_protocol) \
    -b $(upload_speed) -C $(arduino_dir)hardware/tools/avrdude.conf


$(avr_core_obj_dir)/%.o:
	@echo "CC $(filter $*.%, $(avr_core_src_cpp) $(avr_core_src_c)) \
	------> $*.o"
	@$(AVR_CC) $(AVR_CPPFLAGS) -MMD \
	$(avr_core_inc) $(arduino_core_directory)$(filter $*.%, $(avr_core_src_cpp) $(avr_core_src_c)) \
	-o $@ 


$(arduino_static_library): $(avr_core_c_obj) $(avr_core_cpp_obj)
	@echo "Creating arduino library"
	$(AVR_AR) rcs $(arduino_static_library) $(avr_core_c_obj) $(avr_core_cpp_obj)


frksky_telemetry_src:=$(wildcard $(PROJECT_DIRECTORY)/*.cpp) \
	$(wildcard $(PROJECT_DIRECTORY)/GCS_Mavlink/*.cpp) \
	$(wildcard $(PROJECT_DIRECTORY)/SoftwareSerial/*.cpp) \

frksky_telemetry_obj:=$(patsubst %.cpp, %.o, $(frksky_telemetry_src))

%.o: %.cpp
	@echo "CC $^ ------> $@"
	
	$(CC) $(CPPFLAGS) $(ACFLAGS) $(INC_PARAMS) $^ -o $@ 

clean:
	find $(PROJECT_DIRECTORY) -type f -name "*.d" -o -name "*.o" -o -name "*.a"  \
		-o -name "*.hex" -o -name "*.zip" | xargs rm -f

/tmp/frsky_telemetry.elf: CC:=$(AVR_CC)
/tmp/frsky_telemetry.elf: CPPFLAGS:=$(AVR_CPPFLAGS)
/tmp/frsky_telemetry.elf: AC_FLAGS:=$(AVR_CPPFLAGS)
/tmp/frsky_telemetry.elf: INC_PARAMS+=$(AVR_INC)
/tmp/frsky_telemetry.elf: $(arduino_static_library) $(frksky_telemetry_obj)
	$(CC) $(frksky_telemetry_obj) \
		$(frksky_telemetry_softwareserial_obj) \
		$(AVR_LFLAGS) -L$(avr_core_obj_dir) -l$(arduino_static_library_name) \
		-o $@ 

#	@$(PRETTY_PRINT)

frsky_telemetry.hex: /tmp/frsky_telemetry.elf
	$(AVR_OBJCOPY) -O ihex -R .eeprom $< $@
	
upload_frsky_telemetry: frsky_telemetry.hex
	$(AVRDUDE) $(AVRDUDE_FLAGS) $(AVRDUDE_WRITE_FLASH)$<:i

manual:
	pushd Manual/2_1_x/; pdflatex Manual.tex; popd
	mv Manual/2_1_x/Manual.pdf "Quick Start Guide.pdf"

frsky_telemetry.zip: frsky_telemetry.hex manual
	#
	pushd $(PROJECT_DIRECTORY); git archive -o frsky_telemetry.zip -9 HEAD; popd
	zip $(PROJECT_DIRECTORY)/frsky_telemetry.zip -d Firmware/Source/Manual*
	pushd $(PROJECT_DIRECTORY); zip -g frsky_telemetry.zip Firmware/frsky_telemetry.hex; popd

all: upload_frsky_telemetry
