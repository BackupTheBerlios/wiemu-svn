/**
    adc.cc
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
#include "adc.hh"
#include "avr.hh"
#include "regs.hh"
#include "../../include/clock.hh"
#include <iostream>
#include <fstream> // <--- temp

Adc::Adc(){
	enabled = false;
	interrupt_enabled = false;
	conversion_complete = false;
	conversion_start = false;
	cycles = 0;
	Vref = 2.56;		// TODO: switch between AVcc & Aref
	clk_event.setDevice((void *)this);
	log.open("adc.log");
}

void
Adc::setMCU(Mcu *mcu){
	avr = (Avr*)mcu;
	avr->addIOListener(AVR_IOREG_ADCSRA, this);
	//avr->addIOListener(AVR_IOREG_PINF, this);
	avr->addIOListener(AVR_IOREG_ADMUX, this);
	avr->addIOListener(AVR_IOREG_ADCL, this);
	avr->addIOListener(AVR_IOREG_ADCH, this);
}

void
Adc::probeIO(uint8_t port, uint8_t verb){
	switch(verb){
		case DEV_IO_READ:
			if(port == AVR_IOREG_ADCSRA){
				(*(avr->sram))[AVR_IOREG_ADCSRA + AVR_IO_BASE] = adcsra;
				log << avr->getCycles() << " " << std::hex << avr->regs->pc*2 << std::dec << " ";
				log << "ADC READ ADCSRA = 0x" << std::hex << (int)adcsra << std::dec << std::endl;
			}
			else if(port == AVR_IOREG_PINF){
				//log << avr->getCycles() << " " << std::hex << avr->regs->pc*2 << std::dec << " ";
				//log << "ADC READ PINF = 0x" << std::hex << (int)(*(avr->sram))[AVR_IOREG_PINF + AVR_IO_BASE] << std::dec << std::endl;
			}
			else if(port == AVR_IOREG_ADMUX){
				log << avr->getCycles() << " " << std::hex << avr->regs->pc*2 << std::dec << " ";
				log << "ADC READ ADMUX = 0x" << std::hex << (int)(*(avr->sram))[AVR_IOREG_ADMUX + AVR_IO_BASE] << std::dec << std::endl;
			}
			else if(port == AVR_IOREG_ADCL){
				(*(avr->sram))[AVR_IOREG_ADCL + AVR_IO_BASE] = adcl;
				log << avr->getCycles() << " " << std::hex << avr->regs->pc*2 << std::dec << " ";
				log << "ADC READ ADCL = 0x" << std::hex << (int)adcl << std::dec << std::endl;
			}
			else if(port == AVR_IOREG_ADCH){
				(*(avr->sram))[AVR_IOREG_ADCH + AVR_IO_BASE] = adch;
				log << avr->getCycles() << " " << std::hex << avr->regs->pc*2 << std::dec << " ";
				log << "ADC READ ADCH = 0x" << std::hex << (int)adch << std::dec << std::endl;
			}
			break;
		case DEV_IO_WRITE:
			if(port == AVR_IOREG_ADCSRA){
				adcsra = (*(avr->sram))[AVR_IOREG_ADCSRA + AVR_IO_BASE];
				log << avr->getCycles() << " " << std::hex << avr->regs->pc*2 << std::dec << " ";
				log << "ADC WRITE ADCSRA = 0x" << std::hex << (int)adcsra << std::dec << std::endl;
				updateADCSRA();
			}
			else if(port == AVR_IOREG_PINF){
				//log << avr->getCycles() << " " << std::hex << avr->regs->pc*2 << std::dec << " ";
				//log << "ADC WRITE PINF = 0x" << std::hex << (int)(*(avr->sram))[AVR_IOREG_PINF + AVR_IO_BASE] << std::dec << std::endl;
			}
			else if(port == AVR_IOREG_ADMUX){
				log << avr->getCycles() << " " << std::hex << avr->regs->pc*2 << std::dec << " ";
				log << "ADC WRITE ADMUX = 0x" << std::hex << (int)(*(avr->sram))[AVR_IOREG_ADMUX + AVR_IO_BASE] << std::dec << std::endl;
			}
			else if(port == AVR_IOREG_ADCL){
				log << avr->getCycles() << " " << std::hex << avr->regs->pc*2 << std::dec << " ";
				log << "ADC WRITE ADCL = 0x" << std::hex << (int)(*(avr->sram))[AVR_IOREG_ADCL + AVR_IO_BASE] << std::dec << std::endl;
			}
			else if(port == AVR_IOREG_ADCH){
				log << avr->getCycles() << " " << std::hex << avr->regs->pc*2 << std::dec << " ";
				log << "ADC WRITE ADCH = 0x" << std::hex << (int)(*(avr->sram))[AVR_IOREG_ADCH+ AVR_IO_BASE] << std::dec << std::endl;
			}
			break;
	}
	log.flush();
}

void
Adc::updateADCSRA()
{
	enabled = adcsra & (1 << AVR_ADCSRA_ADEN);
	interrupt_enabled = adcsra & (1 << AVR_ADCSRA_ADIE);
	// Datasheet '2467V–AVR–02/11' page 244
	// ADIF is cleared when logical one is written to it
	if(adcsra & (1 << AVR_ADCSRA_ADIF)){
		conversion_complete = false;
		adcsra &= ~(1 << AVR_ADCSRA_ADIF);
	}
	// Prescaler Select Bits
	// These bits determine the division factor between the XTAL frequency and the input clock to the ADC
	switch(adcsra & AVR_ADPS_MSK){
		case 0:
		case 1:
			prescale = 2;
			break;
		case 2:
			prescale = 4;
			break;
		case 3:
			prescale = 8;
			break;
		case 4:
			prescale = 16;
			break;
		case 5:
			prescale = 32;
			break;
		case 6:
			prescale = 64;
			break;
		case 7:
			prescale = 128;
			break;
	}
	// Conversion Started ???
	conversion_start = adcsra & (1 << AVR_ADCSRA_ADSC);

	clk_event.setCycles(prescale);
	avr->addClockEvent(&clk_event);
}

void
Adc::updateADMUX()
{
	// TODO: TinyOS only update ADMUX with 0x0
}

void
Adc::tick()
{
	if(!enabled)
		return;
	if(conversion_complete && interrupt_enabled){
		conversion_complete = false;			// Clear conversion_complete
		adcsra &= ~(1 << AVR_ADCSRA_ADIF);		// Clear ADIF
		log << avr->getCycles() << " " << std::hex << avr->regs->pc*2 << std::dec << " ";
		log << "ADC Firing an Interrupt 0x21" << std::endl;
		avr->fireInterrupt(21);				// Fire ADC conversion complete interrupt
	}
	else if(conversion_start){
		if(++cycles == 13){		// TODO: 1st conversion takes 25 cycles instead
			convert();
			adcsra |= (1 << AVR_ADCSRA_ADIF);
			adcsra &= ~(1 << AVR_ADCSRA_ADSC);
			conversion_start = false;
			conversion_complete = true;
			cycles = 0;
		}
	}
	avr->addClockEvent(&clk_event);
}

void
Adc::convert()
{
	Vin = avr->getPins()[AVR_PIN_PF0].getAnalogValue();
	log << "ADC Vin = " << Vin << std::endl;
	int val = int((Vin * 1024) / Vref);
	val = 0x3ff;
	// TODO: implement ADLAR bit
	adcl = (uint8_t)val;
	adch = (uint8_t)((val >> 8) & 0x3);
}

/*
	CLK Ticker
*/
Adc::ClkEvent::ClkEvent()
{
	cycles = 2;
}

void
Adc::ClkEvent::setCycles(uint64_t cycles)
{
	this->cycles = cycles;
}

void
Adc::ClkEvent::setDevice(void *dev)
{
	adc = (Adc *)dev;
}

void
Adc::ClkEvent::fired()
{
	adc->tick();
}

