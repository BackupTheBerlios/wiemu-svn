/**
    spi.hh
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

#ifndef ADC_HH
#define ADC_HH

#include "../../include/internaldevice.hh"
#include "../../include/clock.hh"

/*
 * Analog to Digital Converter (ADC)
 */
#define	AVR_IOREG_ADCSRA	0x06	// ADC Control and Status Register A
#define	AVR_IOREG_ADMUX		0x07	// ADC Multiplexer Selection Register
#define	AVR_IOREG_ADCL		0x04	// The ADC Data Register Low
#define	AVR_IOREG_ADCH		0x05	// The ADC Data Register High

#define	AVR_IOREG_SFIOR		0x20	// Special Function IO Register

// ADC Control and Status Register A (ADCSRA)
#define	AVR_ADCSRA_ADEN         7
#define	AVR_ADCSRA_ADSC         6
#define	AVR_ADCSRA_ADFR         5
#define	AVR_ADCSRA_ADIF         4
#define	AVR_ADCSRA_ADIE         3
#define	AVR_ADCSRA_ADPS2        2
#define	AVR_ADCSRA_ADPS1        1
#define	AVR_ADCSRA_ADPS0        0

// ADC Multiplexer select (ADMUX)
#define	AVR_ADMUX_REFS1		7
#define	AVR_ADMUX_REFS0		6
#define	AVR_ADMUX_ADLAR		5
#define	AVR_ADMUX_MUX4		4
#define	AVR_ADMUX_MUX3		3
#define	AVR_ADMUX_MUX2		2
#define	AVR_ADMUX_MUX1		1
#define	AVR_ADMUX_MUX0		0

#define	AVR_ADPS_MSK		0x7

// PINS
#define	ADC0		1		// PINF1

#define	LOW		false
#define	HIGH		true

#include <fstream>

class Avr;

class Adc: InternalDevice{
	class ClkEvent: public Event{
	private:
		Adc *adc;
	public:
		ClkEvent();
		void setCycles(uint64_t);
		void setDevice(void *);
		void fired();
	};
private:
	Avr *avr;
	bool enabled;
	bool interrupt_enabled;
	bool conversion_complete, conversion_start;
	uint8_t adcsra, adch, adcl;
	uint8_t prescale;
	ClkEvent clk_event;
	unsigned int cycles;
	double Vin, Vref;
	void updateADCSRA();
	void updateADMUX();
	void tick();
	void convert();
	std::ofstream log;
public:
	Adc();
	void setMCU(Mcu *);
	void probeIO(uint8_t, uint8_t);
};

#endif

