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

Timer::Timer(){
	t0_run = false;
	tcnt0 = 0;
	t1_run = false;
	tcnt1 = 0;
}

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
Timer::inp(uint8_t port){
	if(port == TCNT0)
		mem[TCNT0+0x20] = tcnt0;
	else if(port == TCNT1L)
		mem[TCNT1L+0x20] = (uint8_t)(tcnt1);
	else if(port == TCNT1H)
		mem[TCNT1H+0x20] = (uint8_t)(tcnt1 >> 8);
}

void
Timer::outp(uint8_t port, uint64_t clk){
	if(port == TCCR0)
		setTCCR0(clk);
	else if(port == TCCR1B)
		setTCCR1B(clk);
}

void
Timer::update(uint64_t clk){
	updateTimer0(clk);
	updateTimer1(clk);
}

void
Timer::setTCCR0(uint64_t clk){
	tccr0 = mem[TCCR0+0x20];
	t0_clk_start = clk;
	// clock select (CS)
	std::cout << "TCCR0: " << (int)(tccr0) << std::endl;
	uint8_t cs = tccr0 & 0x7;
	switch(cs){
		case T0_CS_NCLK:
			t0_cs = 0;
			t0_run = false;
			break;
		case T0_CS_PRE1:
			t0_cs = 1;
			t0_run = true;
			break;
		case T0_CS_PRE8:
			t0_cs = 8;
			t0_run = true;
			break;
		case T0_CS_PRE32:
			t0_cs = 32;
			t0_run = true;
			break;
		case T0_CS_PRE64:
			t0_cs = 64;
			t0_run = true;
			break;
		case T0_CS_PRE128:
			t0_cs = 128;
			t0_run = true;
			break;
		case T0_CS_PRE256:
			t0_cs = 256;
			t0_run = true;
			break;
		case T0_CS_PRE1024:
			t0_cs = 1024;
			t0_run = true;
			break;
		default:
			break;
	}
}

void
Timer::updateTimer0(uint64_t clk){
	uint8_t tmp = tcnt0;
	if(t0_run){
		tcnt0 = (clk - t0_clk_start) / t0_cs;
	}
	if(tmp != tcnt0)
		std::cout << "CLK=" << (int)(clk) << " TCNT0: " << (int)(tcnt0) << std::endl;
}

void
Timer::setTCCR1B(uint64_t clk){
	tccr1b = mem[TCCR1B+0x20];
	t1_clk_start = clk;
	// clock select (CS)
	std::cout << "TCCR1B: " << (int)(tccr1b) << std::endl;
	uint8_t cs = tccr1b & 0x7;
	switch(cs){
		case T1_CS_NCLK:
			t1_cs = 0;
			t1_run = false;
			break;
		case T1_CS_PRE1:
			t1_cs = 1;
			t1_run = true;
			break;
		case T1_CS_PRE8:
			t1_cs = 8;
			t1_run = true;
			break;
		case T1_CS_PRE64:
			t1_cs = 64;
			t1_run = true;
			break;
		case T1_CS_PRE256:
			t1_cs = 256;
			t1_run = true;
			break;
		case T1_CS_PRE1024:
			t1_cs = 1024;
			t1_run = true;
			break;
		default:
			break;
	}
}

void
Timer::updateTimer1(uint64_t clk){
	uint16_t tmp = tcnt1;
	if(t1_run){
		tcnt1 = (clk - t1_clk_start) / t1_cs;
	}
	if(tmp != tcnt1)
		std::cout << "CLK=" << (int)(clk) << " TCNT1: " << (int)(tcnt1) << std::endl;
}
