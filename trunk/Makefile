DATE = \"`date`\"
CXX = g++
CXXFLAGS = -DDISASM -Wall -O3 -DDATE="$(DATE)" -DCXX="\"$(CXX)\"" -DDEBUG
RM = rm -f
STRIP = strip
all:
	$(CXX) $(CXXFLAGS) -c wiemu.cc -o build/wiemu.o -g
	$(CXX) $(CXXFLAGS) -c firmware.cc -o build/firmware.o -g
	$(CXX) $(CXXFLAGS) -c pin.cc -o build/pin.o -g
	$(CXX) $(CXXFLAGS) -c device.cc -o build/device.o -g
	#$(CXX) $(CXXFLAGS) -c arch/arm/regs.cc -o build/arm/regs.o -g 
	#$(CXX) $(CXXFLAGS) -c arch/arm/system.cc -o build/arm/system.o -g
	$(CXX) $(CXXFLAGS) -c arch/avr/avr.cc -o build/avr/avr.o -g
	$(CXX) $(CXXFLAGS) -c arch/avr/regs.cc -o build/avr/regs.o -g
	$(CXX) $(CXXFLAGS) -c arch/avr/pins.cc -o build/avr/pins.o -g
	$(CXX) $(CXXFLAGS) -c mote/mica2/mica2.cc -o build/mote/mica2/mica2.o -g
	$(CXX) $(CXXFLAGS) -c mote/mica2/leds.cc -o build/mote/mica2/leds.o -g
	$(CXX) $(CXXFLAGS) -c mote/mica2/cc1000.cc -o build/mote/mica2/cc1000.o -g
	$(CXX) build/wiemu.o \
		build/firmware.o \
		build/pin.o \
		build/device.o \
		build/avr/regs.o \
		build/avr/avr.o \
		build/avr/pins.o \
		build/mote/mica2/mica2.o \
		build/mote/mica2/leds.o \
		build/mote/mica2/cc1000.o \
		-o build/wiemu -g
	#$(STRIP) wiemu
	@echo "WiEmu built Successfully ..."
clean:
	$(RM) build/avr/* build/arm/* build/i386/* build/mote/mica2/*
	$(RM) build/*.o build/wiemu

