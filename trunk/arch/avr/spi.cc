/**
    spi.cc
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
#include "spi.hh"
#include "avr.hh"
#include "regs.hh"
#include <iostream>
#include <fstream> // <--- temp

Spi::Spi(){
	log.open("spi.log");
}

void
Spi::setMCU(Mcu *mcu){
	avr = (Avr*)mcu;
	avr->addIOListener(AVR_IOREG_SPCR, this);
	avr->addIOListener(AVR_IOREG_SPSR, this);
	avr->addIOListener(AVR_IOREG_SPDR, this);
}

void
Spi::probeIO(uint8_t port, uint8_t verb){
	switch(verb){
		case DEV_IO_READ:
			if(port == AVR_IOREG_SPCR){
				log << avr->getCycles() << " " << std::hex << avr->regs->pc*2 << std::dec << " ";
				log << "SPI READ SPCR = 0x" << std::hex << (int)(*(avr->sram))[AVR_IOREG_SPCR + AVR_IO_BASE] << std::dec << std::endl;
			}
			else if(port == AVR_IOREG_SPSR){
				log << avr->getCycles() << " " << std::hex << avr->regs->pc*2 << std::dec << " ";
				log << "SPI READ SPSR = 0x" << std::hex << (int)(*(avr->sram))[AVR_IOREG_SPSR + AVR_IO_BASE] << std::dec << std::endl;
			}
			else if(port == AVR_IOREG_SPDR){
				log << avr->getCycles() << " " << std::hex << avr->regs->pc*2 << std::dec << " ";
				log << "SPI READ SPDR = 0x" << std::hex << (int)(*(avr->sram))[AVR_IOREG_SPDR + AVR_IO_BASE] << std::dec << std::endl;
				//(*(avr->sram))[AVR_IOREG_SPSR + AVR_IO_BASE] |= (1 << AVR_SPSR_SPIF);
			}
			break;
		case DEV_IO_WRITE:
			if(port == AVR_IOREG_SPCR){
				log << avr->getCycles() << " " << std::hex << avr->regs->pc*2 << std::dec << " ";
				log << "SPI WRITE SPCR = 0x" << std::hex << (int)(*(avr->sram))[AVR_IOREG_SPCR + AVR_IO_BASE] << std::dec << std::endl;
				//printSPCR();
				log << "Interrupt = 18" << std::endl;
				avr->fireInterrupt(17);
			}
			else if(port == AVR_IOREG_SPSR){
				log << avr->getCycles() << " " << std::hex << avr->regs->pc*2 << std::dec << " ";
				log << "SPI WRITE SPSR = 0x" << std::hex << (int)(*(avr->sram))[AVR_IOREG_SPSR + AVR_IO_BASE] << std::dec << std::endl;
			}
			else if(port == AVR_IOREG_SPDR){
				log << avr->getCycles() << " " << std::hex << avr->regs->pc*2 << std::dec << " ";
				log << "SPI WRITE SPDR = 0x" << std::hex << (int)(*(avr->sram))[AVR_IOREG_SPDR + AVR_IO_BASE] << std::dec << std::endl;
				//(*(avr->sram))[AVR_IOREG_SPSR + AVR_IO_BASE] |= (1 << AVR_SPSR_SPIF);
			}
			break;
	}
	uint8_t spcr = (*(avr->sram))[AVR_IOREG_SPCR + AVR_IO_BASE];
	uint8_t spsr = (*(avr->sram))[AVR_IOREG_SPSR + AVR_IO_BASE];
	// is SPI interrupt enabled?
	if((spcr & (1 << AVR_SPCR_SPIE)) && (spsr & (1 << AVR_SPSR_SPIF))){
		(*(avr->sram))[AVR_IOREG_SPSR + AVR_IO_BASE] &= ~(1 << AVR_SPSR_SPIF);
		log << avr->getCycles() << " " << std::hex << avr->regs->pc*2 << std::dec << " ";
		log << "Interrupt = 18" << std::endl;
		avr->fireInterrupt(17);
	}
	log.flush();
}

void
Spi::printSPCR(){
	uint8_t spcr = (*(avr->sram))[AVR_IOREG_SPCR + AVR_IO_BASE];
	// SPI Enable
	if(spcr & (1 << AVR_SPCR_SPE))
		std::cerr << "SPI: ON" << std::endl;
	else
		std::cerr << "SPI: OFF" << std::endl;
	// SPI Interrupt Enable
	if(spcr & (1 << AVR_SPCR_SPIE))
		std::cerr << "SPI Interrupt: Enabled" << std::endl;
	else
		std::cerr << "SPI Interrupt: Disabled" << std::endl;
	// Data Order
	if(spcr & (1 << AVR_SPCR_DORD))
		std::cerr << "SPI Data Order: LSB -> MSB" << std::endl;
	else
		std::cerr << "SPI Data Order: MSB -> LSB" << std::endl;
	// Master/Slave Select
	if(spcr & (1 << AVR_SPCR_MSTR)){
		std::cerr << "SPI Operating: Master" << std::endl;
		operating = MASTER;
	}
	else{
		std::cerr << "SPI Operating: Slave" << std::endl;
		operating = SLAVE;
	}
	// Clock Polarity
	if(spcr & (1 << AVR_SPCR_CPOL))
		std::cerr << "SPI IDLE CLK Polarity: HIGH" << std::endl;
	else
		std::cerr << "SPI IDLE CLK Polarity: LOW" << std::endl;
	// Clock Phase
	if(spcr & (1 << AVR_SPCR_CPHA))
		std::cerr << "SPI Sampling Edge: Trailing" << std::endl;
	else
		std::cerr << "SPI Sampling Edge: Leading" << std::endl;
	// SPI Clock Rate Select (Master only)
	if(operating == MASTER){
		uint8_t spr = (spcr & (1 << AVR_SPCR_SPR1)) | (spcr & (1 << AVR_SPCR_SPR0));
		// Frequency divider
		uint8_t div = 0;
		switch(spr){
			case 0:
				div = 4;
				break;
			case 1:
				div = 16;
				break;
			case 2:
				div = 64;
				break;
			case 3:
				div = 128;
				break;
		}
		// The SPI2X(Double SPI Speed Bit) flag is in the SPSR not SPCR!
		uint8_t spsr = (*(avr->sram))[AVR_IOREG_SPSR + AVR_IO_BASE];
		if(spsr & (1 << AVR_SPSR_SPI2X))
			div /= 2;
		std::cerr << "SPI Frequency: /" << (int)div << std::endl;
	}
}

void
Spi::printSPSR(){
	uint8_t spsr = (*(avr->sram))[AVR_IOREG_SPSR + AVR_IO_BASE];
	/*
		SPI Interrupt Flag
		When a serial transfer is complete, the SPIF flag is set. An interrupt is generated if SPIE in
		SPCR is set and global interrupts are enabled. If SS is an input and is driven low when the SPI is
		in Master mode, this will also set the SPIF flag. SPIF is cleared by hardware when executing the
		corresponding interrupt handling vector. Alternatively, the SPIF bit is cleared by first reading the
		SPI Status Register with SPIF set, then accessing the SPI Data Register (SPDR).
	*/
	if(spsr & (1 << AVR_SPSR_SPIF))
		std::cerr << "SPI SPIF Set" << std::endl;
	/*
		Write COLlision flag
		The WCOL bit is set if the SPI Data Register (SPDR) is written during a data transfer. The
		WCOL bit (and the SPIF bit) are cleared by first reading the SPI Status Register with WCOL set,
		and then accessing the SPI Data Register.
	*/
	if(spsr & (1 << AVR_SPSR_WCOL))
		std::cerr << "SPI Data Collision Occured" << std::endl;
}

void
Spi::printSPDR(){
	uint8_t spdr = (*(avr->sram))[AVR_IOREG_SPDR + AVR_IO_BASE];
	std::cerr << "SPI SPDR = " << std::hex << (int)spdr << std::dec << std::endl;
}


