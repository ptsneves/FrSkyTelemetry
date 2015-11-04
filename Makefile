#Skimmed from russellbarnes
#https://raw.githubusercontent.com/russellbarnes/Teensy-3.1-C-Example/master/Makefile
#Will generate some warning due to CXX flags used in .c files. Not nice but can be ignored as the compiler does... 

frksky_telemetry_mkfile_dir:=$(dir $(lastword $(MAKEFILE_LIST)))

PROJECT_DIRECTORY:=$(shell git rev-parse --show-toplevel)

frksky_telemetry_src:=$(wildcard $(PROJECT_DIRECTORY)/*.cpp) \
	$(wildcard $(PROJECT_DIRECTORY)/GCS_Mavlink/*.cpp)

frksky_telemetry_obj:=$(patsubst %.cpp, %.o, $(frksky_telemetry_src))

teensy_dir:=/home/pneves/Downloads/arduino-1.6.5
teensy_core_directory:=$(teensy_dir)/hardware/teensy/avr/cores/teensy3
teensy_core_src:=$(notdir $(wildcard $(teensy_core_directory)/*.cpp)) \
	$(notdir $(wildcard $(teensy_core_directory)/*.c))
	
teensy_core_obj_dir:=$(PROJECT_DIRECTORY)/core
teensy_core_obj:=$(patsubst %.cpp, $(teensy_core_obj_dir)/%.o, $(filter %.cpp, $(teensy_core_src))) \
	$(patsubst %.c, $(teensy_core_obj_dir)/%.o, $(filter %.c, $(teensy_core_src)))
teensy_core_inc:=-I$(teensy_core_directory)

teensy_static_library_name:=teensycore
teensy_static_library:=$(teensy_core_obj_dir)/lib$(teensy_static_library_name).a

$(teensy_core_obj_dir)/%.o:
	#matches the current source file being compiled
	@echo "CC CORE $(filter $*.%, $(teensy_core_src)) \
	------> $*.o"
	$(CC) -c $(CPPFLAGS) $(CXXFLAGS) -MMD \
		$(teensy_core_inc) $(teensy_core_directory)/$(filter $*.%, $(teensy_core_src)) -o $@


$(teensy_static_library): $(teensy_core_obj)
	@echo "Creating arduino library"
	$(AR) rcs $(teensy_static_library) $(teensy_core_obj)

%.o: %.cpp
	@echo "CC $^ ------> $@"
	@$(CC) -c $(CPPFLAGS) $(CXXFLAGS) $(teensy_core_inc) $^ -o $@ 

clean:
	find $(PROJECT_DIRECTORY) -type f -name "*.d" -o -name "*.o" -o -name "*.a"  \
		-o -name "*.hex" -o -name "*.zip" | xargs rm -f

/tmp/frsky_telemetry.elf: AR:=arm-none-eabi-ar
/tmp/frsky_telemetry.elf: CC:=arm-none-eabi-gcc
/tmp/frsky_telemetry.elf: CPPFLAGS:=-Os -mthumb -mcpu=cortex-m4 -fsingle-precision-constant \
	-D__MK20DX256__ -DUSING_MAKEFILE -DARDUINO=10600 -DTEENSYDUINO=125 -DF_CPU=96000000 -DUSB_SERIAL -DLAYOUT_US_ENGLISH \
	-g -Wall -ffunction-sections -fdata-sections -MMD -nostdlib
/tmp/frsky_telemetry.elf: CXXFLAGS:=-std=gnu++0x -felide-constructors -fno-exceptions -fno-rtti
/tmp/frsky_telemetry.elf: LDFLAGS:=-Os -Wl,--gc-sections,--relax -mcpu=cortex-m4 -mthumb \
	-T$(teensy_core_directory)/mk20dx256.ld
/tmp/frsky_telemetry.elf: LIBS:=-lm -L -l$(teensy_static_library_name)
/tmp/frsky_telemetry.elf: $(teensy_static_library) $(frksky_telemetry_obj)
	$(CC) $(LDFLAGS) $(frksky_telemetry_obj) -o $@ -L$(teensy_core_directory) $(LIBS)

#	@$(PRETTY_PRINT)
frsky_telemetry.hex: OBJCOPY:=arm-none-eabi-objcopy
frsky_telemetry.hex: SIZE:=arm-none-eabi-size
frsky_telemetry.hex: /tmp/frsky_telemetry.elf
	$(SIZE) $<
	$(OBJCOPY) -O ihex -R .eeprom $< $@
	
upload_frsky_telemetry: frsky_telemetry.hex

frsky_telemetry.zip: frsky_telemetry.hex
	git archive -o frsky_telemetry.zip -9 HEAD
	zip -g frsky_telemetry.zip frsky_telemetry.hex

all: upload_frsky_telemetry