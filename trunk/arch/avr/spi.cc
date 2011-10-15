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
	enabled = false;
	ss = HIGH;
	ss_prev = HIGH;
	sck = LOW;
	sck_prev = LOW;
	clk_state = LEADING;
	bits = 0;
	spdr = 0;
	spdr_buf = 0;
	txf.open("tx.dat");
	rxf.open("rx.dat");
}

void
Spi::setMCU(Mcu *mcu){
	avr = (Avr*)mcu;
	avr->addIOListener(AVR_IOREG_SPCR, this);
	avr->addIOListener(AVR_IOREG_SPSR, this);
	avr->addIOListener(AVR_IOREG_SPDR, this);
	avr->addIOListener(AVR_IOREG_PINB, this);
}

void
Spi::probeIO(uint8_t port, uint8_t verb){
	switch(verb){
		case DEV_IO_READ:
			if(port == AVR_IOREG_SPCR){
				(*(avr->sram))[AVR_IOREG_SPCR + AVR_IO_BASE] = spcr;
			}
			else if(port == AVR_IOREG_SPSR){
				(*(avr->sram))[AVR_IOREG_SPSR + AVR_IO_BASE] = spsr;
			}
			else if(port == AVR_IOREG_SPDR){
				spdr = spdr_buf;
				(*(avr->sram))[AVR_IOREG_SPDR + AVR_IO_BASE] = spdr;
				rxf << avr->getCycles() << "\t" << std::hex << int(spdr) << std::dec << std::endl;
			}
			else if(port == AVR_IOREG_PINB){
			}
			break;
		case DEV_IO_WRITE:
			if(port == AVR_IOREG_SPCR){
				spcr = (*(avr->sram))[AVR_IOREG_SPCR + AVR_IO_BASE];
				if(spcr & (1 << AVR_SPCR_SPE))
					enabled = true;
				else
					enabled = false;
				if(spcr & (1 << AVR_SPCR_CPHA))
					cpha = TRAILING;
				else
					cpha = LEADING;
				if(spcr & (1 << AVR_SPCR_CPOL))
					cpol = LOW;
				else
					cpol = HIGH;
				reset();
				//printSPCR();
			}
			else if(port == AVR_IOREG_SPSR){
				spsr = (*(avr->sram))[AVR_IOREG_SPSR + AVR_IO_BASE];
			}
			else if(port == AVR_IOREG_SPDR){
				spdr = (*(avr->sram))[AVR_IOREG_SPDR + AVR_IO_BASE];
				reset();
				txf << avr->getCycles() << "\t" << std::hex << int(spdr) << std::dec << std::endl;
			}
			else if(port == AVR_IOREG_PINB){
				if(!enabled)					// SPI disabled
					return;
				// SS
				ss_prev = ss;
				ss = avr->getPins()[AVR_PIN_PB0].get();		// read SS from PB0 pin
				// SCK
				sck_prev = sck;
				sck = avr->getPins()[AVR_PIN_PB1].get();	// read SCK from PB1 pin
				if(sck_prev != sck){ 				// tick
					if(operating == MASTER)
						master();
					else
						slave();
					clk_state = !clk_state;
				}
			}
			break;
	}
	rxf.flush();
	txf.flush();
}

void
Spi::reset(){
	bits = 8;
	(*(avr->sram))[AVR_IOREG_SPSR + AVR_IO_BASE] &= ~(1 << AVR_SPSR_SPIF);
}

void
Spi::master(){
	// TODO
}

void
Spi::slave(){
	// Working as SLAVE
	if(ss != LOW)
		return;
	if((clk_state != cpha) || (sck != cpol))
		return;
	if(bits){
		if(spdr & MSB)				// write MISO to PB3 pin
			avr->getPins()[AVR_PIN_PB3].set();
		else
			avr->getPins()[AVR_PIN_PB3].clear();
		if(avr->getPins()[AVR_PIN_PB2].get())	// read MOSI from PB2 pin
			spdr_buf |= 0x1;		// set the LSB
		else
			spdr_buf &= 0xfe;		// clear the LSB
		--bits;
		if(!bits){
			(*(avr->sram))[AVR_IOREG_SPSR + AVR_IO_BASE] |= (1 << AVR_SPSR_SPIF);
			if(spcr & (1 << AVR_SPCR_SPIE)){		// Interrupts are enabled?
				(*(avr->sram))[AVR_IOREG_SPSR + AVR_IO_BASE] &= ~(1 << AVR_SPSR_SPIF); //<---
				avr->fireInterrupt(17);
			}
		}
		else{
			spdr <<= 1;
			spdr_buf <<= 1;
		}
	}
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


