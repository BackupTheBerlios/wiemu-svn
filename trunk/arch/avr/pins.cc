/**
    pins.cc
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

#include "avr.hh"
#include "pins.hh"
#include <iostream>

void
Avr::initPins()
{
	// Port A
	pins[AVR_PIN_PA0] = new Pin("PA0", true);
	pins[AVR_PIN_PA1] = new Pin("PA1", true);
	pins[AVR_PIN_PA2] = new Pin("PA2", true);
	pins[AVR_PIN_PA3] = new Pin("PA3", false);
	pins[AVR_PIN_PA4] = new Pin("PA4", false);
	pins[AVR_PIN_PA5] = new Pin("PA5", false);
	pins[AVR_PIN_PA6] = new Pin("PA6", false);
	pins[AVR_PIN_PA7] = new Pin("PA7", false);
	// Port B
	pins[AVR_PIN_PB0] = new Pin("PB0");
	pins[AVR_PIN_PB1] = new Pin("PB1");
	pins[AVR_PIN_PB2] = new Pin("PB2");
	pins[AVR_PIN_PB3] = new Pin("PB3");
	pins[AVR_PIN_PB4] = new Pin("PB4");
	pins[AVR_PIN_PB5] = new Pin("PB5");
	pins[AVR_PIN_PB6] = new Pin("PB6");
	pins[AVR_PIN_PB7] = new Pin("PB7");
	// Port C
	pins[AVR_PIN_PC0] = new Pin("PC0");
	pins[AVR_PIN_PC1] = new Pin("PC1");
	pins[AVR_PIN_PC2] = new Pin("PC2");
	pins[AVR_PIN_PC3] = new Pin("PC3");
	pins[AVR_PIN_PC4] = new Pin("PC4");
	pins[AVR_PIN_PC5] = new Pin("PC5");
	pins[AVR_PIN_PC6] = new Pin("PC6");
	pins[AVR_PIN_PC7] = new Pin("PC7");
	// Port D
	pins[AVR_PIN_PD0] = new Pin("PD0");
	pins[AVR_PIN_PD1] = new Pin("PD1");
	pins[AVR_PIN_PD2] = new Pin("PD2");
	pins[AVR_PIN_PD3] = new Pin("PD3");
	pins[AVR_PIN_PD4] = new Pin("PD4");
	pins[AVR_PIN_PD5] = new Pin("PD5");
	pins[AVR_PIN_PD6] = new Pin("PD6");
	pins[AVR_PIN_PD7] = new Pin("PD7");
	// Port E
	pins[AVR_PIN_PE0] = new Pin("PE0");
	pins[AVR_PIN_PE1] = new Pin("PE1");
	pins[AVR_PIN_PE2] = new Pin("PE2");
	pins[AVR_PIN_PE3] = new Pin("PE3");
	pins[AVR_PIN_PE4] = new Pin("PE4");
	pins[AVR_PIN_PE5] = new Pin("PE5");
	pins[AVR_PIN_PE6] = new Pin("PE6");
	pins[AVR_PIN_PE7] = new Pin("PE7");
	// Port F
	pins[AVR_PIN_PF0] = new Pin("PF0");
	pins[AVR_PIN_PF1] = new Pin("PF1");
	pins[AVR_PIN_PF2] = new Pin("PF2");
	pins[AVR_PIN_PF3] = new Pin("PF3");
	pins[AVR_PIN_PF4] = new Pin("PF4");
	pins[AVR_PIN_PF5] = new Pin("PF5");
	pins[AVR_PIN_PF6] = new Pin("PF6");
	pins[AVR_PIN_PF7] = new Pin("PF7");
	// Port G
	pins[AVR_PIN_PG0] = new Pin("PG0");
	pins[AVR_PIN_PG1] = new Pin("PG1");
	pins[AVR_PIN_PG2] = new Pin("PG2");
	pins[AVR_PIN_PG3] = new Pin("PG3");
	pins[AVR_PIN_PG4] = new Pin("PG4");
}

void
Avr::writePins()
{
	// Port A
	writePin(AVR_IOREG_DDRA, AVR_IOREG_PINA, AVR_IOREG_PORTA, AVR_PIN_PA0, 0);
	writePin(AVR_IOREG_DDRA, AVR_IOREG_PINA, AVR_IOREG_PORTA, AVR_PIN_PA1, 1);
	writePin(AVR_IOREG_DDRA, AVR_IOREG_PINA, AVR_IOREG_PORTA, AVR_PIN_PA2, 2);
	writePin(AVR_IOREG_DDRA, AVR_IOREG_PINA, AVR_IOREG_PORTA, AVR_PIN_PA3, 3);
	writePin(AVR_IOREG_DDRA, AVR_IOREG_PINA, AVR_IOREG_PORTA, AVR_PIN_PA4, 4);
	writePin(AVR_IOREG_DDRA, AVR_IOREG_PINA, AVR_IOREG_PORTA, AVR_PIN_PA5, 5);
	writePin(AVR_IOREG_DDRA, AVR_IOREG_PINA, AVR_IOREG_PORTA, AVR_PIN_PA6, 6);
	writePin(AVR_IOREG_DDRA, AVR_IOREG_PINA, AVR_IOREG_PORTA, AVR_PIN_PA7, 7);
	// Port B
	writePin(AVR_IOREG_DDRB, AVR_IOREG_PINB, AVR_IOREG_PORTB, AVR_PIN_PB0, 0);
	writePin(AVR_IOREG_DDRB, AVR_IOREG_PINB, AVR_IOREG_PORTB, AVR_PIN_PB1, 1);
	writePin(AVR_IOREG_DDRB, AVR_IOREG_PINB, AVR_IOREG_PORTB, AVR_PIN_PB2, 2);
	writePin(AVR_IOREG_DDRB, AVR_IOREG_PINB, AVR_IOREG_PORTB, AVR_PIN_PB3, 3);
	writePin(AVR_IOREG_DDRB, AVR_IOREG_PINB, AVR_IOREG_PORTB, AVR_PIN_PB4, 4);
	writePin(AVR_IOREG_DDRB, AVR_IOREG_PINB, AVR_IOREG_PORTB, AVR_PIN_PB5, 5);
	writePin(AVR_IOREG_DDRB, AVR_IOREG_PINB, AVR_IOREG_PORTB, AVR_PIN_PB6, 6);
	writePin(AVR_IOREG_DDRB, AVR_IOREG_PINB, AVR_IOREG_PORTB, AVR_PIN_PB7, 7);
	// Port C
	writePin(AVR_IOREG_DDRC, AVR_IOREG_PINC, AVR_IOREG_PORTC, AVR_PIN_PC0, 0);
	writePin(AVR_IOREG_DDRC, AVR_IOREG_PINC, AVR_IOREG_PORTC, AVR_PIN_PC1, 1);
	writePin(AVR_IOREG_DDRC, AVR_IOREG_PINC, AVR_IOREG_PORTC, AVR_PIN_PC2, 2);
	writePin(AVR_IOREG_DDRC, AVR_IOREG_PINC, AVR_IOREG_PORTC, AVR_PIN_PC3, 3);
	writePin(AVR_IOREG_DDRC, AVR_IOREG_PINC, AVR_IOREG_PORTC, AVR_PIN_PC4, 4);
	writePin(AVR_IOREG_DDRC, AVR_IOREG_PINC, AVR_IOREG_PORTC, AVR_PIN_PC5, 5);
	writePin(AVR_IOREG_DDRC, AVR_IOREG_PINC, AVR_IOREG_PORTC, AVR_PIN_PC6, 6);
	writePin(AVR_IOREG_DDRC, AVR_IOREG_PINC, AVR_IOREG_PORTC, AVR_PIN_PC7, 7);
	// Port D
	writePin(AVR_IOREG_DDRD, AVR_IOREG_PIND, AVR_IOREG_PORTD, AVR_PIN_PD0, 0);
	writePin(AVR_IOREG_DDRD, AVR_IOREG_PIND, AVR_IOREG_PORTD, AVR_PIN_PD1, 1);
	writePin(AVR_IOREG_DDRD, AVR_IOREG_PIND, AVR_IOREG_PORTD, AVR_PIN_PD2, 2);
	writePin(AVR_IOREG_DDRD, AVR_IOREG_PIND, AVR_IOREG_PORTD, AVR_PIN_PD3, 3);
	writePin(AVR_IOREG_DDRD, AVR_IOREG_PIND, AVR_IOREG_PORTD, AVR_PIN_PD4, 4);
	writePin(AVR_IOREG_DDRD, AVR_IOREG_PIND, AVR_IOREG_PORTD, AVR_PIN_PD5, 5);
	writePin(AVR_IOREG_DDRD, AVR_IOREG_PIND, AVR_IOREG_PORTD, AVR_PIN_PD6, 6);
	writePin(AVR_IOREG_DDRD, AVR_IOREG_PIND, AVR_IOREG_PORTD, AVR_PIN_PD7, 7);
	// Port E
	writePin(AVR_IOREG_DDRE, AVR_IOREG_PINE, AVR_IOREG_PORTE, AVR_PIN_PE0, 0);
	writePin(AVR_IOREG_DDRE, AVR_IOREG_PINE, AVR_IOREG_PORTE, AVR_PIN_PE1, 1);
	writePin(AVR_IOREG_DDRE, AVR_IOREG_PINE, AVR_IOREG_PORTE, AVR_PIN_PE2, 2);
	writePin(AVR_IOREG_DDRE, AVR_IOREG_PINE, AVR_IOREG_PORTE, AVR_PIN_PE3, 3);
	writePin(AVR_IOREG_DDRE, AVR_IOREG_PINE, AVR_IOREG_PORTE, AVR_PIN_PE4, 4);
	writePin(AVR_IOREG_DDRE, AVR_IOREG_PINE, AVR_IOREG_PORTE, AVR_PIN_PE5, 5);
	writePin(AVR_IOREG_DDRE, AVR_IOREG_PINE, AVR_IOREG_PORTE, AVR_PIN_PE6, 6);
	writePin(AVR_IOREG_DDRE, AVR_IOREG_PINE, AVR_IOREG_PORTE, AVR_PIN_PE7, 7);
	// Port F
	writePin(AVR_IOREG_DDRF, AVR_IOREG_PINF, AVR_IOREG_PORTF, AVR_PIN_PF0, 0);
	writePin(AVR_IOREG_DDRF, AVR_IOREG_PINF, AVR_IOREG_PORTF, AVR_PIN_PF1, 1);
	writePin(AVR_IOREG_DDRF, AVR_IOREG_PINF, AVR_IOREG_PORTF, AVR_PIN_PF2, 2);
	writePin(AVR_IOREG_DDRF, AVR_IOREG_PINF, AVR_IOREG_PORTF, AVR_PIN_PF3, 3);
	writePin(AVR_IOREG_DDRF, AVR_IOREG_PINF, AVR_IOREG_PORTF, AVR_PIN_PF4, 4);
	writePin(AVR_IOREG_DDRF, AVR_IOREG_PINF, AVR_IOREG_PORTF, AVR_PIN_PF5, 5);
	writePin(AVR_IOREG_DDRF, AVR_IOREG_PINF, AVR_IOREG_PORTF, AVR_PIN_PF6, 6);
	writePin(AVR_IOREG_DDRF, AVR_IOREG_PINF, AVR_IOREG_PORTF, AVR_PIN_PF7, 7);
	// Port G
	writePin(AVR_IOREG_DDRG, AVR_IOREG_PING, AVR_IOREG_PORTG, AVR_PIN_PG0, 0);
	writePin(AVR_IOREG_DDRG, AVR_IOREG_PING, AVR_IOREG_PORTG, AVR_PIN_PG1, 1);
	writePin(AVR_IOREG_DDRG, AVR_IOREG_PING, AVR_IOREG_PORTG, AVR_PIN_PG2, 2);
	writePin(AVR_IOREG_DDRG, AVR_IOREG_PING, AVR_IOREG_PORTG, AVR_PIN_PG3, 3);
}

void
Avr::writePin(uint16_t DDR, uint16_t IN, uint16_t OUT, uint16_t PIN, uint8_t b)
{
	if(getBit(this->sram[AVR_IO_START+DDR], b)){	// Is Output?
		if(getBit(this->sram[AVR_IO_START+OUT], b))
			pins[PIN].set();
		else
			pins[PIN].clear();
	}
}

void
Avr::readPins()
{
	// Port A
	readPin(AVR_IOREG_DDRA, AVR_IOREG_PINA, AVR_IOREG_PORTA, AVR_PIN_PA0, 0);
	readPin(AVR_IOREG_DDRA, AVR_IOREG_PINA, AVR_IOREG_PORTA, AVR_PIN_PA1, 1);
	readPin(AVR_IOREG_DDRA, AVR_IOREG_PINA, AVR_IOREG_PORTA, AVR_PIN_PA2, 2);
	readPin(AVR_IOREG_DDRA, AVR_IOREG_PINA, AVR_IOREG_PORTA, AVR_PIN_PA3, 3);
	readPin(AVR_IOREG_DDRA, AVR_IOREG_PINA, AVR_IOREG_PORTA, AVR_PIN_PA4, 4);
	readPin(AVR_IOREG_DDRA, AVR_IOREG_PINA, AVR_IOREG_PORTA, AVR_PIN_PA5, 5);
	readPin(AVR_IOREG_DDRA, AVR_IOREG_PINA, AVR_IOREG_PORTA, AVR_PIN_PA6, 6);
	readPin(AVR_IOREG_DDRA, AVR_IOREG_PINA, AVR_IOREG_PORTA, AVR_PIN_PA7, 7);
	// Port B
	readPin(AVR_IOREG_DDRB, AVR_IOREG_PINB, AVR_IOREG_PORTB, AVR_PIN_PB0, 0);
	readPin(AVR_IOREG_DDRB, AVR_IOREG_PINB, AVR_IOREG_PORTB, AVR_PIN_PB1, 1);
	readPin(AVR_IOREG_DDRB, AVR_IOREG_PINB, AVR_IOREG_PORTB, AVR_PIN_PB2, 2);
	readPin(AVR_IOREG_DDRB, AVR_IOREG_PINB, AVR_IOREG_PORTB, AVR_PIN_PB3, 3);
	readPin(AVR_IOREG_DDRB, AVR_IOREG_PINB, AVR_IOREG_PORTB, AVR_PIN_PB4, 4);
	readPin(AVR_IOREG_DDRB, AVR_IOREG_PINB, AVR_IOREG_PORTB, AVR_PIN_PB5, 5);
	readPin(AVR_IOREG_DDRB, AVR_IOREG_PINB, AVR_IOREG_PORTB, AVR_PIN_PB6, 6);
	readPin(AVR_IOREG_DDRB, AVR_IOREG_PINB, AVR_IOREG_PORTB, AVR_PIN_PB7, 7);
	// Port C
	readPin(AVR_IOREG_DDRC, AVR_IOREG_PINC, AVR_IOREG_PORTC, AVR_PIN_PC0, 0);
	readPin(AVR_IOREG_DDRC, AVR_IOREG_PINC, AVR_IOREG_PORTC, AVR_PIN_PC1, 1);
	readPin(AVR_IOREG_DDRC, AVR_IOREG_PINC, AVR_IOREG_PORTC, AVR_PIN_PC2, 2);
	readPin(AVR_IOREG_DDRC, AVR_IOREG_PINC, AVR_IOREG_PORTC, AVR_PIN_PC3, 3);
	readPin(AVR_IOREG_DDRC, AVR_IOREG_PINC, AVR_IOREG_PORTC, AVR_PIN_PC4, 4);
	readPin(AVR_IOREG_DDRC, AVR_IOREG_PINC, AVR_IOREG_PORTC, AVR_PIN_PC5, 5);
	readPin(AVR_IOREG_DDRC, AVR_IOREG_PINC, AVR_IOREG_PORTC, AVR_PIN_PC6, 6);
	readPin(AVR_IOREG_DDRC, AVR_IOREG_PINC, AVR_IOREG_PORTC, AVR_PIN_PC7, 7);
	// Port D
	readPin(AVR_IOREG_DDRD, AVR_IOREG_PIND, AVR_IOREG_PORTD, AVR_PIN_PD0, 0);
	readPin(AVR_IOREG_DDRD, AVR_IOREG_PIND, AVR_IOREG_PORTD, AVR_PIN_PD1, 1);
	readPin(AVR_IOREG_DDRD, AVR_IOREG_PIND, AVR_IOREG_PORTD, AVR_PIN_PD2, 2);
	readPin(AVR_IOREG_DDRD, AVR_IOREG_PIND, AVR_IOREG_PORTD, AVR_PIN_PD3, 3);
	readPin(AVR_IOREG_DDRD, AVR_IOREG_PIND, AVR_IOREG_PORTD, AVR_PIN_PD4, 4);
	readPin(AVR_IOREG_DDRD, AVR_IOREG_PIND, AVR_IOREG_PORTD, AVR_PIN_PD5, 5);
	readPin(AVR_IOREG_DDRD, AVR_IOREG_PIND, AVR_IOREG_PORTD, AVR_PIN_PD6, 6);
	readPin(AVR_IOREG_DDRD, AVR_IOREG_PIND, AVR_IOREG_PORTD, AVR_PIN_PD7, 7);
}

void
Avr::readPin(uint16_t DDR, uint16_t IN, uint16_t OUT, uint16_t PIN, uint8_t b)
{
	if(!getBit(this->sram[AVR_IO_START+DDR], b)){	// Is Input?
		if(pins[PIN].get())
			setBit(this->sram[AVR_IO_START+IN], b);
		else
			clearBit(this->sram[AVR_IO_START+IN], b);
	}
}

void
Avr::printPins()
{
	std::cout << "(DDRA)0x1a=" << (int)this->sram[AVR_IOREG_DDRA + AVR_IO_START] << std::endl;
	std::cout << "(PORTA)0x1b=" << (int)this->sram[AVR_IOREG_PORTA + AVR_IO_START] << std::endl;
	std::cout << "(PINA)0x19=" << (int)this->sram[AVR_IOREG_PINA + AVR_IO_START] << std::endl;
	std::cout << pins[AVR_PIN_PA0].getName() + "=" << pins[AVR_PIN_PA0].get() << std::endl;
	std::cout << "PA1=" << (pins[AVR_PIN_PA1].get()) << std::endl;
	std::cout << "PA2=" << (pins[AVR_PIN_PA2].get()) << std::endl;
	std::cout << "PA3=" << (pins[AVR_PIN_PA3].get()) << std::endl;
	std::cout << "PA4=" << (pins[AVR_PIN_PA4].get()) << std::endl;
	std::cout << "PA5=" << (pins[AVR_PIN_PA5].get()) << std::endl;
	std::cout << "PA6=" << (pins[AVR_PIN_PA6].get()) << std::endl;
	std::cout << "PA7=" << (pins[AVR_PIN_PA7].get()) << std::endl;	
}
