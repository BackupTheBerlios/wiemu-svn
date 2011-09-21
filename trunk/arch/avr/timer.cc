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
#include "../../include/clock.hh"
#include "timer.hh"
#include "avr.hh"
#include <iostream>

Timer::Timer(){
	this->avr = avr;
	overflow = false;
	assr = 0;
	tifr = 0;
	timsk = 0;
	sleep = false;
	// Timer0
	ticks0 = 0;
	t0_run = false;
	t0_initialized = false;
	tcnt0 = 0;
	ocr0 = 0;
	async0 = false;
	// Timer1
	ticks1 = 0;
	t1_run = false;
	t1_initialized = false;
	tcnt1 = 0;
	temp1 = 0;
	ocr1a = 0;
	ocr1b = 0;
	async1 = false;
}

Timer::~Timer(){}

void
Timer::setMCU(Mcu *mcu){
	avr = (Avr*)mcu;
	avr->addIOListener(TCNT0, this);
	avr->addIOListener(TCCR0, this);
	avr->addIOListener(OCR0, this);
	avr->addIOListener(ASSR, this);
	avr->addIOListener(TCNT1L, this);
	avr->addIOListener(TCNT1H, this);
	avr->addIOListener(TCCR1B, this);
	//avr->addIOListener(OCR1A, this);
	//avr->addIOListener(OCR1B, this);
	avr->addIOListener(TIMSK, this);
	avr->addIOListener(TIFR, this);
}

void
Timer::probeIO(uint8_t port, uint8_t verb){
	switch(verb){
		case DEV_IO_READ:
			if(port == TCNT0){
				(*(avr->sram))[TCNT0 + AVR_IO_BASE] = tcnt0;
			}
			else if(port == TCCR0){
				(*(avr->sram))[TCCR0 + AVR_IO_BASE] = tccr0;
			}
			else if(port == OCR0){
				(*(avr->sram))[OCR0 + AVR_IO_BASE] = ocr0;
			}
			else if(port == ASSR){
				(*(avr->sram))[ASSR + AVR_IO_BASE] = assr;
			}
			else if(port == TCNT1L){
				(*(avr->sram))[TCNT1L + AVR_IO_BASE] = (uint8_t)(tcnt1);
				temp1 = (uint8_t)(tcnt1 >> 8);			// Copy TCNTL1H to TEMP Reg
			}
			else if(port == TCNT1H){
				(*(avr->sram))[TCNT1H + AVR_IO_BASE] = temp1;
			}
			else if(port == TCCR1B){
				(*(avr->sram))[TCCR1B + AVR_IO_BASE] = tccr1b;
			}
			//else if(port == OCR1A){
			//	(*(avr->sram))[OCR1A + AVR_IO_BASE] = ocr1a;
			//}
			//else if(port == OCR1B){
			//	(*(avr->sram))[OCR1B + AVR_IO_BASE] = ocr1b;
			//}
			else if(port == TIMSK){
				(*(avr->sram))[TIMSK + AVR_IO_BASE] = timsk;
			}
			else if(port == TIFR){
				(*(avr->sram))[TIFR + AVR_IO_BASE] = tifr;
			}
			break;
		case DEV_IO_WRITE:
			if(port == TCNT0)
				setTCNT0();
			else if(port == TCCR0)
				setTCCR0();
			else if(port == ASSR)
				setASSR();
			else if(port == TCCR1B)
				setTCCR1B();
			else if(port == OCR0)
				setOCR0();
			else if(port == TCNT1L)
				setTCNT1L();
			else if(port == TCNT1H)
				setTCNT1H();
			//else if(port == OCR1A)
			//	setOCR1A();
			//else if(port == OCR1B)
			//	setOCR1B();
			else if(port == TIMSK)
				setTIMSK();
			else if(port == TIFR)
				setTIFR();
			break;
	}
}

void
Timer::update(Clock c){
	// Increment the overall ticks by the elapsed
	// clock cycles in the last instruction
	if(t0_run){
		for(uint64_t i=0 ; i<c.getCycles() ; i++){
			ticks0++;
			if(t0_initialized && t0_cs)
				updateTimer0();
		}
	}
	if(t1_run){
		for(uint64_t i=0 ; i<c.getCycles() ; i++){
			ticks1++;
			if(t1_initialized && t1_cs)
				updateTimer1();
		}
	}
}

void
Timer::setASSR(){
	assr = (*(avr->sram))[ASSR + AVR_IO_BASE];
	if(assr & (1 << AS0)){
		async0 = true;
	}else{
		async0 = false;
	}
}

void
Timer::setTIMSK(){
	timsk = (*(avr->sram))[TIMSK + AVR_IO_BASE];
}

void
Timer::setTIFR(){
	tifr = (*(avr->sram))[TIFR + AVR_IO_BASE];
	// Some flags are cleared by writting one into them
	if(tifr & (1 << OCF0))
		tifr &= ~(1 << OCF0);		// Clear OCF0
	if(tifr & (1 << TOV0))
		tifr &= ~(1 << TOV0);		// Clear TOV0
}

void
Timer::setTCCR0(){
	tccr0 = (*(avr->sram))[TCCR0 + AVR_IO_BASE];
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
	ticks0--;						// Skip next cycle
	// For compatability with AVR Studio
	// Timer start ticking when any value is written in TCNT
	// even if TCCRx is not set yet!
	t0_initialized = true;
}

void
Timer::setTCNT0(){
	tcnt0 = (*(avr->sram))[TCNT0 + AVR_IO_BASE];
	ticks0 = 0;						 // Reset Timer0
	t0_run = true;
}

void
Timer::setOCR0(){
	ocr0 = (*(avr->sram))[OCR0 + AVR_IO_BASE];
}

void
Timer::updateTimer0(){
	uint8_t tcnt0_prev = tcnt0;
	if(!async0){
		if(ticks0 % t0_cs == 0 && ticks0 != 0)
			tcnt0++;
	}else{
		if(ticks0 % (ASYNC_DIV * t0_cs) == 0 && ticks0 != 0)
			tcnt0++;
	}
	if(tcnt0_prev != tcnt0){
		// Check for Overflow
		if(tcnt0 == 0){
			tifr |= (1 << TOV0);
		}
		// Compare Match
		if(tcnt0_prev == ocr0){
			tifr |= (1 << OCF0);
		}
		/**
			Interrupts
		**/	
		// Compare Match Interrupt
		if((timsk & (1 << OCIE0)) && (tifr & (1 << OCF0))){
			// Clear OCF0
			tifr &= ~(1 << OCF0);
			std::cerr << "Timer0 COMP......" << std::endl;
			// Interrupt
			avr->fireInterrupt(15);
		}
		// Overflow Interrupt
		else if((timsk & (1 << TOIE0)) && (tifr & (1 << TOV0))){
			// Clear TOV0
			tifr &= ~(1 << TOV0);
			// Interrupt
			avr->fireInterrupt(16);
		}
	}
}

void
Timer::setTCCR1B(){
	tccr1b = (*(avr->sram))[TCCR1B + AVR_IO_BASE];
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
	ticks1--;						// Skip next cycle
	// For compatability with AVR Studio
	// Timer start ticking when any value is written in TCNT
	// even if TCCRx is not set yet!
	t1_initialized = true;
}

void
Timer::setTCNT1L(){
	tcnt1 = (tcnt1 & 0xff00) | temp1;
}

void
Timer::setTCNT1H(){
	tcnt1 = (tcnt1 & 0x00ff) | ((*(avr->sram))[TCNT1H + AVR_IO_BASE] << 8);
	temp1 = (*(avr->sram))[TCNT1L + AVR_IO_BASE];
	ticks1 = 0;						 // Reset Timer1
	t1_run = true;
}

void
Timer::updateTimer1(){
	uint16_t tcnt1_prev = tcnt1;
	if(!async1){
		if(ticks1 % t1_cs == 0 && ticks1 != 0)
			tcnt1++;
	}else{
		if(ticks1 % (ASYNC_DIV * t1_cs) == 0 && ticks1 != 0)
			tcnt1++;
	}
	if(tcnt1_prev != tcnt1){
		// Check for Overflow
		if(tcnt1 == 0){
			tifr |= (1 << TOV1);
		}
		// Compare Match
		if(tcnt1_prev == ocr1a){
			tifr |= (1 << OCF1A);
		}
		if(tcnt1_prev == ocr1b){
			tifr |= (1 << OCF1B);
		}
		// Compare Match Interrupt
		if((timsk & (1 << OCIE1A)) && (tifr & (1 << OCF1A))){
			// Clear OCF1A
			tifr &= ~(1 << OCF1A);
			// Interrupt
			avr->fireInterrupt(12); 
		}
		if((timsk & (1 << OCIE1B)) && (tifr & (1 << OCF1B))){
			// Clear OCF1B
			tifr &= ~(1 << OCF1B);
			// Interrupt
			avr->fireInterrupt(13);
		}
		// Overflow Interrupt
		else if((timsk & (1 << TOIE1)) && (tifr & (1 << TOV1))){
			// Clear TOV1
			tifr &= ~(1 << TOV1);
			// Interrupt
			avr->fireInterrupt(14);
		}
	}
}

int Timer::getTCNT0(){
	return (int)tcnt0;
}
