/**
    timer.cc
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
#include "timer.hh"
#include <iostream>

Timer::Timer(){}

Timer::~Timer(){}

void
Timer::setMem(uint8_t *mem){
	this->mem = mem;
}

void
Timer::setFlash(uint16_t *flash){
	this->flash = flash;
}

bool
Timer::getBit(uint8_t byte, unsigned int bit){
	if(byte & (1 << bit))
		return true;
	else
		return false;
}

bool
Timer::getBit(uint16_t byte, unsigned int bit){
	if(byte & (1 << bit))
		return 1;
	else
		return 0;
}

void
Timer::setBit(uint8_t &byte, unsigned int bit){
	byte |= (1 << bit);
}

void
Timer::clearBit(uint8_t &byte, unsigned int bit){
	byte &= ~(1 << bit);	
}

void
Timer::setTCCR0(){
	tccr0 = mem[TCCR0+0x20];
	// clock select (CS)
	std::cout << "TCCR0: " << (int)(tccr0) << std::endl;
	uint8_t cs = tccr0 & 0x7;
	switch(cs){
			case T0_CS_NCLK:
				break;
			case T0_CS_PRE8:
				break;
			case T0_CS_PRE64:
				break;
			case T0_CS_PRE256:
				break;
			case T0_CS_PRE1024:
				break;
			default:
				break;
	}
}
