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
	// Timer0
	ticks0 = 0;
	t0_run = false;
	tcnt0 = 0;
	assr = 0;
	async0 = false;
	// Timer1
	ticks1 = 0;
	t1_run = false;
	tcnt1 = 0;
	temp1 = 0;
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
	if(port == TCNT0){
		mem[TCNT0 + AVR_IO_BASE] = tcnt0;
	}
	else if(port == TCCR0){
		mem[TCCR0 + AVR_IO_BASE] = tccr0;
	}
	else if(port == TCNT1L){
		mem[TCNT1L + AVR_IO_BASE] = (uint8_t)(tcnt1);
		temp1 = (uint8_t)(tcnt1 >> 8);			// Copy TCNTL1H to TEMP Reg
	}
	else if(port == TCNT1H){
		mem[TCNT1H + AVR_IO_BASE] = temp1;
	}
	else if(port == TCCR1B){
		mem[TCCR1B + AVR_IO_BASE] = tccr1b;
	}
	else if(port == ASSR){
		mem[ASSR + AVR_IO_BASE] = assr;
	}
	else{
		std::cerr << "INP ELSE: " << (int)port << std::endl;
	}
}

void
Timer::outp(uint8_t port, uint64_t clk){
	if(port == TCCR0)
		setTCCR0();
	else if(port == TCCR1B)
		setTCCR1B();
	else if(port == TCNT1L)
		setTCNT1L();
	else if(port == TCNT1H)
		setTCNT1H();
	else if(port == ASSR)
		setASSR();
	else
		std::cerr << clk << ": OUTP ELSE: " << (int)port << "<=" << (int)mem[port+0x20] << std::endl;
}

void
Timer::update(uint64_t c){
	// Increment the overall ticks by the elapsed
	// clock cycles in the last instruction
	if(t0_run){
		ticks0 += c;
		updateTimer0();
	}
	if(t1_run){
		ticks1 += c;
		updateTimer1();
	}
}

void
Timer::setTCCR0(){
	tccr0 = mem[TCCR0 + AVR_IO_BASE];
	// clock select (CS)
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
Timer::setASSR(){
	assr = mem[ASSR + AVR_IO_BASE];
	if(assr & (1 << AS0)){
		async0 = true;
	}else{
		async0 = false;
	}
}

void
Timer::updateTimer0(){
	if(!async0){
		tcnt0 = ticks0 / t0_cs;
	}else{
		tcnt0 = (ticks0 / ASYNC_DIV) / t0_cs;
	}
}

void
Timer::setTCCR1B(){
	tccr1b = mem[TCCR1B + AVR_IO_BASE];
	// clock select (CS)
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
Timer::setTCNT1L(){
	tcnt1 = (tcnt1 & 0xff00) | temp1;
}

void
Timer::setTCNT1H(){
	tcnt1 = (tcnt1 & 0x00ff) | (mem[TCNT1H + AVR_IO_BASE] << 8);
	temp1 = mem[TCNT1L + AVR_IO_BASE];
}

void
Timer::updateTimer1(){
	tcnt1 = ticks1 / t1_cs;
}
