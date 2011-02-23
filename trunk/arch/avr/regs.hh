/**
    regs.hh
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

#ifndef REGS_HH
#define REGS_HH

#include <stdint.h>
#include <string>

#define REG_R0		0x0
#define REG_R1		0x1
#define REG_R2		0x2
#define REG_R3		0x3
#define REG_R4		0x4
#define REG_R5		0x5
#define REG_R6		0x6
#define REG_R7		0x7
#define REG_R8		0x8
#define REG_R9		0x9
#define REG_R10		0xa
#define REG_R11		0xb
#define REG_R12		0xc
#define REG_R13		0xd
#define REG_R14		0xe
#define REG_R15		0xf
#define REG_R16		0x10
#define REG_R17		0x11
#define REG_R18		0x12
#define REG_R19		0x13
#define REG_R20		0x14
#define REG_R21		0x15
#define REG_R22		0x16
#define REG_R23		0x17
#define REG_R24		0x18
#define REG_R25		0x19
#define REG_R26		0x1a
#define REG_R27		0x1b
#define REG_R28		0x1c
#define REG_R29		0x1d
#define REG_R30		0x1e
#define REG_R31		0x1f
#define REG_SPL		0x5d
#define REG_SPH		0x5e
#define REG_SREG	0x5f
#define REG_RAMPZ	0x5b

#define	AVR_IO_START	0x20
#define	AVR_IOREG_DDRA	0x1a
#define	AVR_IOREG_PORTA	0x1b
#define	AVR_IOREG_PINA	0x19
#define	AVR_IOREG_DDRB	0x17
#define	AVR_IOREG_PORTB	0x18
#define	AVR_IOREG_PINB	0x16
#define	AVR_IOREG_DDRC	0x14
#define	AVR_IOREG_PORTC	0x15
#define	AVR_IOREG_PINC	0x13
#define	AVR_IOREG_DDRD	0x11
#define	AVR_IOREG_PORTD	0x12
#define	AVR_IOREG_PIND	0x10
#define	AVR_IOREG_DDRE	0x02
#define	AVR_IOREG_PORTE	0x03
#define	AVR_IOREG_PINE	0x01
#define	AVR_IOREG_DDRF	0x61
#define	AVR_IOREG_PORTF	0x62
#define	AVR_IOREG_PINF	0x00
#define	AVR_IOREG_DDRG	0x64
#define	AVR_IOREG_PORTG	0x65
#define	AVR_IOREG_PING	0x63

class Regs{
public:
	uint8_t *r;
	uint16_t pc;
	Regs();
	~Regs();
	void setMem(uint8_t *);
	void init();
	void dump(void);
	void setI();
	void clearI();
	bool isI();
	void setT();
	void clearT();
	bool isT();
	void setH();
	void clearH();
	bool isH();
	void setS();
	void clearS();
	bool isS();
	void setV();
	void clearV();
	bool isV();
	void setN();
	void clearN();
	bool isN();
	void setZ();
	void clearZ();
	bool isZ();
	void setC();
	void clearC();
	bool isC();
	uint16_t getX();
	void setX(uint16_t);
	uint16_t getY();
	void setY(uint16_t);
	uint16_t getZ();
	void setZ(uint16_t);
	uint16_t getSP();
	void setSP(uint16_t);
	std::string getName(unsigned int);
private:
	static const std::string reg_names[];
};

#endif

