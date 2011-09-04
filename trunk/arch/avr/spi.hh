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

#ifndef SPI_HH
#define SPI_HH

#include "../../include/internaldevice.hh"

/*
 * Serial Peripheral Interface (SPI)
 */
#define	AVR_IOREG_SPCR	0x0d		// SPI Control Register
#define	AVR_IOREG_SPSR	0x0e		// SPI Status Register
#define	AVR_IOREG_SPDR	0x0f		// SPI Data Register
// SPI Control Register (SPCR)
#define	AVR_SPCR_SPIE	7		// SPI Interrupt Enable
#define	AVR_SPCR_SPE	6		// SPI Enable
#define	AVR_SPCR_DORD	5		// Data Order
#define	AVR_SPCR_MSTR	4		// Master/Slave Select
#define	AVR_SPCR_CPOL	3		// Clock Polarity
#define	AVR_SPCR_CPHA	2		// Clock Phase
#define	AVR_SPCR_SPR1	1		// SPI Clock Rate Select 1
#define	AVR_SPCR_SPR0	0		// SPI Clock Rate Select 0
// SPI Status Register (SPSR)
#define	AVR_SPSR_SPIF	7		// SPI Interrupt Flag
#define	AVR_SPSR_WCOL	6		// Write COLlision flag
#define	AVR_SPSR_SPI2X	0		// Double SPI Speed Bit

#define	SLAVE		0
#define	MASTER		1

#include <fstream>

class Avr;

class Spi: InternalDevice{
private:
	Avr *avr;
	bool idle;
	bool operating;
	void printSPCR();
	void printSPSR();
	void printSPDR();
	std::ofstream log;
public:
	Spi();
	void setMCU(Mcu *);
	void probeIO(uint8_t, uint8_t);
};

#endif

