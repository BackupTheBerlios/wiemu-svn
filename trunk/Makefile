DATE = \"`date`\"
CXX = g++
CXXFLAGS = -DDISASM -Wall -O3 -DDATE="$(DATE)" -DCXX="\"$(CXX)\"" -DDEBUG
LDFLAGS = -lpthread -g
RM = rm -f
STRIP = strip
all:
	$(CXX) $(CXXFLAGS) -c wiemu.cc -o build/wiemu.o -g
	$(CXX) $(CXXFLAGS) -c thread.cc -o build/thread.o -g
	$(CXX) $(CXXFLAGS) -c node.cc -o build/node.o -g
	$(CXX) $(CXXFLAGS) -c firmware.cc -o build/firmware.o -g
	$(CXX) $(CXXFLAGS) -c pin.cc -o build/pin.o -g
	$(CXX) $(CXXFLAGS) -c clock.cc -o build/clock.o -g
	#$(CXX) $(CXXFLAGS) -c device.cc -o build/device.o -g
	#$(CXX) $(CXXFLAGS) -c arch/arm/regs.cc -o build/arm/regs.o -g 
	#$(CXX) $(CXXFLAGS) -c arch/arm/system.cc -o build/arm/system.o -g
	$(CXX) $(CXXFLAGS) -c arch/avr/avr.cc -o build/avr/avr.o -g
	$(CXX) $(CXXFLAGS) -c arch/avr/regs.cc -o build/avr/regs.o -g
	$(CXX) $(CXXFLAGS) -c arch/avr/mem.cc -o build/avr/mem.o -g
	$(CXX) $(CXXFLAGS) -c arch/avr/pins.cc -o build/avr/pins.o -g
	$(CXX) $(CXXFLAGS) -c arch/avr/timer.cc -o build/avr/timer.o -g
	$(CXX) $(CXXFLAGS) -c arch/avr/spi.cc -o build/avr/spi.o -g
	$(CXX) $(CXXFLAGS) -c arch/avr/adc.cc -o build/avr/adc.o -g
	$(CXX) $(CXXFLAGS) -c mote/mica2/mica2.cc -o build/mote/mica2/mica2.o -g
	$(CXX) $(CXXFLAGS) -c mote/mica2/leds.cc -o build/mote/mica2/leds.o -g
	$(CXX) $(CXXFLAGS) -c mote/mica2/cc1000.cc -o build/mote/mica2/cc1000.o -g
	$(CXX) build/wiemu.o \
		build/thread.o \
		build/node.o \
		build/firmware.o \
		build/pin.o \
		build/clock.o \
		build/avr/mem.o \
		build/avr/regs.o \
		build/avr/avr.o \
		build/avr/pins.o \
		build/avr/timer.o \
		build/avr/spi.o \
		build/avr/adc.o \
		build/mote/mica2/mica2.o \
		build/mote/mica2/leds.o \
		build/mote/mica2/cc1000.o \
		-o build/wiemu $(LDFLAGS)
	#$(STRIP) wiemu
	@echo "WiEmu built Successfully ..."
clean:
	$(RM) build/avr/* build/arm/* build/i386/* build/mote/mica2/*
	$(RM) build/*.o build/wiemu

