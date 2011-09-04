/**
    mem.cc
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

#include <stdint.h>
#include "mem.hh"
#include "avr.hh"
#include <iostream>

Mem::Mem(){}

Mem::Mem(Avr *avr){
	this->avr = avr;
	memsize = AVR_SRAM_END + 1;
	mem = new uint8_t[memsize];
}

Mem::Mem(Avr *avr, unsigned int size){
	this->avr = avr;
	memsize = size;
	mem = new uint8_t[memsize];
}

Mem::~Mem(){
	delete[] mem;
	memsize = 0;
}

uint8_t&
Mem::operator[](const int i){
	//if((unsigned int)i>=0 && (unsigned int)i<memsize){
		return mem[i];
	//}else{
	//	std::cerr << "MEM ILLEGAL ADDRESS: 0x" << std::hex << i << std::dec << std::endl;
	//	return mem[memsize-1];	// TODO: fix this out
	//}

}

uint8_t
Mem::read(const uint16_t idx){
	if(idx>=AVR_IO_START && idx<=AVR_IO_END){
		std::map<uint8_t, InternalDevice *>::iterator it = iomap.find(idx);
		if(it != iomap.end()){
			it->second->probeIO(idx - AVR_IO_START, DEV_IO_READ);
		}
	}
	//std::cout << "READ idx 0x" << std::hex << (int)idx << " = 0x" << (int)mem[idx] << std::dec << std::endl;
	return mem[idx];	// TODO: check for idx
}

void
Mem::write(const uint16_t idx, const uint8_t val){
	//std::cout << "WRITE idx 0x" << std::hex << (int)idx << " = 0x" << (int)val << std::dec << std::endl;
	mem[idx] = val;	// TODO: check for idx
	if(idx>=AVR_IO_START && idx<=AVR_IO_END){
		std::map<uint8_t, InternalDevice *>::iterator it = iomap.find(idx);
		if(it != iomap.end()){
			it->second->probeIO(idx - AVR_IO_START, DEV_IO_WRITE);
		}
	}
}

void
Mem::clearBit(uint16_t addr, uint8_t bit){
	// mem[addr] &= ~(1 << bit);
	write(addr, (read(addr) & ~(1 << bit)));
}

void
Mem::setBit(uint16_t addr, uint8_t bit){
	// mem[addr] |= (1 << bit);
	write(addr, (read(addr) | (1 << bit)));
}

bool
Mem::getBit(uint16_t addr, uint8_t bit){
	if(mem[addr] & (1 << bit))
		return true;
	else
		return false;
}

void
Mem::addIOListener(uint8_t ioaddr, InternalDevice *dev){
	iomap.insert(std::make_pair(ioaddr + AVR_IO_START, dev));
}

