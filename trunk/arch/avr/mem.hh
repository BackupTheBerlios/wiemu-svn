/**
    mem.hh
    Copyright (C) 2011  Mohamed Aslan <maslan@maslan.info>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
**/

#ifndef MEM_HH
#define MEM_HH

#include <stdint.h>
#include <map>
#include "../../include/internaldevice.hh"

// General purpose registers
#define	AVR_REG_START	0x0		// 32 Bytes Regs
#define	AVR_REG_END	0x1F
// IO registers
#define	AVR_IO_START	0x20		// 224 Bytes IO
#define	AVR_IO_END	0xff		// 224 Bytes IO
// Internal SRAM
#define	AVR_SRAM_START	0x100		// 4 KB SRAM
#define	AVR_SRAM_END	0x10FF

class Avr;

class Mem{
public:
	Mem();
	Mem(Avr *avr);
	Mem(Avr *avr, unsigned int);
	~Mem();
	uint8_t& operator[](const int);
	void addIOListener(uint8_t, InternalDevice *);
	uint8_t read(const uint16_t);
	void write(const uint16_t, const uint8_t);
	void clearBit(uint16_t, uint8_t);
	void setBit(uint16_t, uint8_t);
	bool getBit(uint16_t, uint8_t);
private:
	uint8_t *mem;
	uint8_t *oldmem;		// stores previous values of mem REGS + IO only
	unsigned int memsize;
	unsigned int oldmemsize;
	std::map<uint8_t, InternalDevice *> iomap;
	Avr *avr;
};

#endif

