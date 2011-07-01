/**
    timer.hh
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

#ifndef TIMER_HH
#define TIMER_HH

#define		AVR_IO_BASE	0x20

/**
 * Common for all timers
 **/
// TIFR: Flag Register
#define		TIFR		0x36
// TIFR: Bit Masks
#define		OCF2		7
#define		TOV2		6
#define		ICF1		5
#define		OCF1A		4
#define		OCF1B		3
#define		TOV1		2
#define		OCF0		1				// Timer0 Output Compare Flag
#define		TOV0		0				// Timer0 Overflow Flag

// TIMSK:	Interrupt Mask Register
#define		TIMSK		0x37
// TIMSK:	Bit Masks
#define		OCIE2		7
#define		TOIE2		6
#define		TICIE1		5
#define		OCIE1A		4
#define		OCIE1B		3
#define		TOIE1		2
#define		OCIE0		1
#define		TOIE0		0				// Timer0 Overflow Interrupt

/**
 * Timer 0
 **/
// TCCR0: Control Register
#define		TCCR0		0x33
// TCCR0: Bit Masks
#define		FOC0		7
#define		WGM00		6
#define		COM01		5
#define		COM00		4
#define		WGM01		3
#define		CS02		2
#define		CS01		1
#define		CS00		0
// Timer0 CS
#define		T0_CS_NCLK		0			// No clock (stop)
#define		T0_CS_PRE1		1			// No prescaling
#define		T0_CS_PRE8		2			// Prescaling with /8
#define		T0_CS_PRE32		3			// Prescaling with /32
#define		T0_CS_PRE64		4			// Prescaling with /64
#define		T0_CS_PRE128	5			// Prescaling with /128
#define		T0_CS_PRE256	6			// Prescaling with /256
#define		T0_CS_PRE1024	7			// Prescaling with /1024
// Timer0 Counter Register
#define		TCNT0		0x32
// Timer0 Asynchronous Control & Status Register
#define		ASSR		0x30
#define		AS0		3
#define		ASYNC_DIV	225				// Ext.CLK = 32.768kHz for SYSCLK = 8MHz
// Timer0 Output Compare Register
#define		OCR0		0x31

/**
 * Timer 1
 **/
// TCCR1: Control Registers
#define		TCCR1B		0x2e
// Timer1 CS
#define		T1_CS_NCLK		0			// No clock (stop)
#define		T1_CS_PRE1		1			// No prescaling
#define		T1_CS_PRE8		2			// Prescaling with /8
#define		T1_CS_PRE64		3			// Prescaling with /64
#define		T1_CS_PRE256	4			// Prescaling with /256
#define		T1_CS_PRE1024	5			// Prescaling with /1024
// Timer0 Counter Register
#define		TCNT1L		0x2c
#define		TCNT1H		0x2d


class Avr;

class Timer{
public:
	Timer();
	Timer(Avr *);
	~Timer();
	void setMem(uint8_t *);
	void setFlash(uint16_t *);
	void inp(uint8_t);
	void outp(uint8_t);
	void update(uint64_t);
	bool overflow;
	int getTCNT0();
private:
	Avr *avr;

	uint8_t *mem;
	uint16_t *flash;

	uint8_t timsk;						// Timer Interrupt Mask
	uint8_t tifr;						// Timer Interrupt Flags
	
	// Timer0
	uint64_t ticks0;					// Overall system clock cycles elapsed by timer0
	uint8_t tccr0;						// Timer0 Control Register
	uint8_t tcnt0;						// Timer0 Counter Register
	uint8_t ocr0;						// Timer0 Output Compare Register
	uint8_t assr;						// Asynchronous
	unsigned int t0_cs;					// Timer0 CLK Select
	bool t0_run;						// is Timer0 running?
	bool t0_initialized;					// is TCCR0 initialized?
	bool async0;						// is Asynchronous mode enabled?
	bool toie0;						// is Timer0 interrupt on Overflow enabled?

	// Timer1
	uint64_t ticks1;					// Overall system clock cycles elapsed by timer1
	uint8_t tccr1a, tccr1b, tccr1c;
	uint16_t tcnt1;
	uint16_t ocr1a, ocr1b;
	unsigned int t1_cs;
	bool t1_run;
	bool t1_initialized;
	uint64_t t1_clk_start;
	uint8_t temp1;
	bool async1;						// is Asynchronous mode enabled?

	bool getBit(uint8_t byte, unsigned int bit);
	bool getBit(uint16_t byte, unsigned int bit);
	void setBit(uint8_t &byte, unsigned int bit);
	void clearBit(uint8_t &byte, unsigned int bit);
	void setASSR();
	void setTIMSK();
	void setTIFR();
	void setTCCR0();
	void setTCNT0();
	void setOCR0();
	void updateTimer0();
	void setTCCR1B();
	void setTCNT1L();
	void setTCNT1H();
	void updateTimer1();
};

#endif

