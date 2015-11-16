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
teensy_core_src_cpp:=$(wildcard $(teensy_core_directory)/*.cpp)
teensy_core_src_c:=$(wildcard $(teensy_core_directory)/*.c)

teensy_core_obj_dir:=$(PROJECT_DIRECTORY)/core

teensy_core_cpp_obj:=$(patsubst %.cpp, %.o, $(filter %.cpp, $(teensy_core_src_cpp)))
teensy_core_c_obj:=$(patsubst %.c, %.o, $(filter %.c, $(teensy_core_src_c)))
teensy_core_inc:=-I$(teensy_core_directory)

teensy_static_library_name:=teensycore
teensy_static_library:=$(teensy_core_obj_dir)/lib$(teensy_static_library_name).a

#$(teensy_core_obj_dir)/%.o: %.c
#	#matches the current source file being compiled
#	@echo "CC CORE $(filter $*.%, $(teensy_core_src_cpp) $(teensy_core_src_c)) \
#	------> $*.o"
#	$(CC) -c $(CPPFLAGS) -MMD \
#		$(teensy_core_inc) $(teensy_core_directory)/$(filter $*.%, $(teensy_core_src_c)) -o $@

#$(teensy_core_obj_dir)/%.o: %.cpp
#	#matches the current source file being compiled
#	@echo "CC CORE $(filter $*.%, $(teensy_core_src_cpp) $(teensy_core_src_c)) \
#	------> $*.o"
#	$(CXX) -c $(CPPFLAGS) $(CXXFLAGS) -MMD \
#		$(teensy_core_inc) $(teensy_core_directory)/$(filter $*.%, $(teensy_core_src_cpp)) -o $@

$(teensy_core_cpp_obj):
	#matches the current source file being compiled
	@echo "CXX CORE $(filter $*.%, $(teensy_core_src_cpp)) \
	------> $(notdir $@)"
	$(CXX) -c $(CPPFLAGS) $(CXXFLAGS) -MMD \
		$(teensy_core_inc) $(filter $*.%, $(teensy_core_src_cpp)) -o $(teensy_core_obj_dir)/$(notdir $@)
		
$(teensy_core_c_obj):
	#matches the current source file being compiled
	@echo "CC CORE $(filter $*.%, $(teensy_core_src_c)) \
	------> $(notdir $@)"
	$(CC) -c $(CPPFLAGS) -MMD \
		$(teensy_core_inc) $(filter $*.%, $(teensy_core_src_c)) -o $(teensy_core_obj_dir)/$(notdir $@)

$(teensy_static_library): $(teensy_core_cpp_obj) $(teensy_core_c_obj)
	@echo "Creating arduino library"
	#$(AR) rcs $(teensy_static_library) $(addprefix $(teensy_core_obj_dir)/, $(notdir $(teensy_core_cpp_obj))) $(addprefix $(teensy_core_obj_dir)/, $(notdir $(teensy_core_c_obj)))

$(frksky_telemetry_obj):
	@echo "CXX $(filter $*.%, $(teensy_core_src_cpp)) \
	------> $(notdir $@)"
	$(CXX) -c -I$(teensy_core_directory)/ $(CPPFLAGS) $(CXXFLAGS) -MMD  \
		$(filter $*.%, $(frksky_telemetry_src)i) -o $(frksky_telemetry_mkfile_dir)/$(notdir $@)

%.o: %.cpp
	@echo "CC $^ ------> $@"
	@$(CC) -c $(CPPFLAGS) $(CXXFLAGS) $(teensy_core_inc) $^ -o $@ 

clean:
	find $(PROJECT_DIRECTORY) -type f -name "*.d" -o -name "*.o" -o -name "*.a"  \
		-o -name "*.hex" -o -name "*.zip" | xargs rm -f

frsky_telemetry.elf: AR:=$(teensy_dir)/hardware/tools/arm/bin/arm-none-eabi-ar
frsky_telemetry.elf: CXX:=$(teensy_dir)/hardware/tools/arm/bin/arm-none-eabi-g++
frsky_telemetry.elf: CC:=$(teensy_dir)/hardware/tools/arm/bin/arm-none-eabi-gcc

#frsky_telemetry.elf: CPPFLAGS:=-g -Wall -ffunction-sections -fdata-sections -MMD -nostdlib -Os \
#	-mthumb -mcpu=cortex-m4 -fsingle-precision-constant \
#	-D__MK20DX256__ -DTEENSYDUINO=125 \
#	-DF_CPU=48000000 -DARDUINO_ARCH_AVR -DUSB_SERIAL \
#	-DLAYOUT_US_ENGLISH -DUSING_MAKEFILE
frsky_telemetry.elf: OPTIONS:=-DF_CPU=48000000 -DUSB_SERIAL -DLAYOUT_US_ENGLISH -DUSING_MAKEFILE \
	-D__MK20DX256__ -DARDUINO=10600 -DTEENSYDUINO=121
	
frsky_telemetry.elf: CPPFLAGS:=-Wall -g -O -mcpu=cortex-m4 -mthumb -nostdlib -MMD $(OPTIONS) -I.
frsky_telemetry.elf: CXXFLAGS:= -std=gnu++0x -felide-constructors -fno-exceptions -fno-rtti

frsky_telemetry.elf: LDFLAGS:=-O -Wl,--gc-sections,--defsym=__rtc_localtime=0 --specs=nano.specs -mcpu=cortex-m4 -mthumb -Tmk20dx256.ld
frsky_telemetry.elf: LIBS:=-L -l$(teensy_static_library_name) -lm 
frsky_telemetry.elf: $(teensy_core_cpp_obj) $(teensy_core_c_obj) $(frksky_telemetry_obj) $(teensy_core_directory)/mk20dx256.ld
	$(CC) $(LDFLAGS) $(frksky_telemetry_obj) -o $@ -L$(teensy_core_directory) $(LIBS)

#	@$(PRETTY_PRINT)
frsky_telemetry.hex: OBJCOPY:=/home/pneves/Downloads/arduino-1.6.5/hardware/tools/arm/bin/arm-none-eabi-objcopy 
frsky_telemetry.hex: SIZE:=/home/pneves/Downloads/arduino-1.6.5/hardware/tools/arm/bin/arm-none-eabi-size
frsky_telemetry.hex: frsky_telemetry.elf
	$(SIZE) $<
	$(OBJCOPY)  -O ihex -R .eeprom $< $@
	
	
upload_frsky_telemetry: frsky_telemetry.hex
	/home/pneves/Downloads/arduino-1.6.5/hardware/tools/teensy_post_compile -file=frsky_telemetry -path=$(pwd) -tools=/home/pneves/Downloads/arduino-1.6.5/hardware/tools/
	
frsky_telemetry.zip: frsky_telemetry.hex
	git archive -o frsky_telemetry.zip -9 HEAD
	zip -g frsky_telemetry.zip frsky_telemetry.hex

all: upload_frsky_telemetry