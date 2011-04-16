/**
    regs.cc
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

#include "regs.hh"
#include <iostream>
#include <iomanip>
#include <sstream>			// To be removed

const std::string
Regs::reg_names[] = {
	"r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7", "r8", "r9", "r10",
	"r11", "r12", "r13", "r14", "r15", "r16", "r17", "r18", "r19",
	"r20", "r21", "r22", "r23", "r24", "r25", "r26", "r27", "r28",
	"r29", "r30", "r31"
};

Regs::Regs(){}

Regs::~Regs(){}

void
Regs::setMem(uint8_t *mem){
	this->r = mem;
}

void
Regs::init(){
	for(int i=0 ; i<32; i++)
		this->r[i] = 0;
	this->pc = 0;
	this->r[REG_SREG] = 0;
	this->r[REG_SPL] = 0;
	this->r[REG_SPH] = 0;
}

void
Regs::dump(){
	for(int i=0 ; i<32 ; i++){
		std::cout << "R" << std::setfill('0') << std::setw(2) << std::dec << i << std::hex;
		std::cout << "=" << std::setfill('0') << std::setw(2) << (int)this->r[i] << "   ";
		if(!((i+1)%8))
			std::cout << std::endl;
	}
	std::cout << "X=";
	std::cout << std::setw(2);
	std::cout << (int)this->r[REG_R27];
	std::cout << std::setw(2);
	std::cout << (int)this->r[REG_R26];
	std::cout << "   ";
	std::cout << "Y=";
	std::cout << std::setw(2);
	std::cout << (int)this->r[REG_R29];
	std::cout << std::setw(2);
	std::cout << (int)this->r[REG_R28];
	std::cout << "   ";
	std::cout << "Z=";
	std::cout << std::setw(2);
	std::cout << (int)this->r[REG_R31];
	std::cout << std::setw(2);
	std::cout << (int)this->r[REG_R30];
	std::cout << "   ";
	// Mapped in the I/O
	std::cout << "SP=";
	std::cout << std::setw(2);
	std::cout << (int)this->r[REG_SPH];		// SPH
	std::cout << std::setw(2);
	std::cout << (int)this->r[REG_SPL];		// SPL
	std::cout << "   ";
	// Mapped in the I/O
	std::cout << "SREG=";
	std::cout << std::setw(2) << (int)this->r[REG_SREG];
	std::cout << "   ";
	std::cout << "PC=";
	std::cout << std::setw(4) << this->pc;
	std::cout << std::dec << std::endl;
}

std::string
Regs::dump2(){
	std::stringstream ss;
	for(int i=0 ; i<32 ; i++){
		ss << "R" << std::setfill('0') << std::setw(2) << std::dec << i << std::hex;
		ss << "=" << std::setfill('0') << std::setw(2) << (int)this->r[i] << "   ";
		if(!((i+1)%8))
			ss << std::endl;
	}
	ss << "X=";
	ss << std::setw(2);
	ss << (int)this->r[REG_R27];
	ss << std::setw(2);
	ss << (int)this->r[REG_R26];
	ss << "   ";
	ss << "Y=";
	ss << std::setw(2);
	ss << (int)this->r[REG_R29];
	ss << std::setw(2);
	ss << (int)this->r[REG_R28];
	ss << "   ";
	ss << "Z=";
	ss << std::setw(2);
	ss << (int)this->r[REG_R31];
	ss << std::setw(2);
	ss << (int)this->r[REG_R30];
	ss << "   ";
	// Mapped in the I/O
	ss << "SP=";
	ss << std::setw(2);
	ss << (int)this->r[REG_SPH];		// SPH
	ss << std::setw(2);
	ss << (int)this->r[REG_SPL];		// SPL
	ss << "   ";
	// Mapped in the I/O
	ss << "SREG=";
	ss << std::setw(2) << (int)this->r[REG_SREG];
	ss << "   ";
	ss << "PC=";
	ss << std::setw(4) << this->pc;
	ss << std::dec << std::endl;
	return ss.str();
}

std::string
Regs::dump3(uint16_t pc, uint64_t clk){
	std::stringstream ss;
	for(int i=0 ; i<32 ; i++){
		if(!((i+1)%8)){
			ss << "R" << i;
			ss << "=" << (int)this->r[i];
			ss << std::endl;
		}else{
			ss << "R" << i;
			ss << "=" << (int)this->r[i] << " ";
		}
	}
	uint16_t sp = (this->r[REG_SPH] << 8) | this->r[REG_SPL];
	ss << "SP=";
	ss << (int)sp;
	ss << " ";
	ss << "SREG=";
	ss << (int)this->r[REG_SREG];
	ss << " ";
	ss << "PC=";
	ss << (int)(pc*2);
	ss << " ";
	ss << "SCLK=";
	ss << clk;
	ss << std::endl;
	return ss.str();
}

void
Regs::setI(){
	this->r[REG_SREG] |= 0x80;
}

void
Regs::clearI(){
	this->r[REG_SREG] &= 0x7f;
}

bool
Regs::isI(){
	return this->r[REG_SREG] & 0x80;
}

void
Regs::setT(){
	this->r[REG_SREG] |= 0x40;
}

void
Regs::clearT(){
	this->r[REG_SREG] &= 0xbf;
}

bool
Regs::isT(){
	return this->r[REG_SREG] & 0x40;
}

void
Regs::setH(){
	this->r[REG_SREG] |= 0x20;
}

void
Regs::clearH(){
	this->r[REG_SREG] &= 0xdf;
}

bool
Regs::isH(){
	return this->r[REG_SREG] & 0x20;
}

void
Regs::setS(){
	this->r[REG_SREG] |= 0x10;
}

void
Regs::clearS(){
	this->r[REG_SREG] &= 0xef;
}

bool
Regs::isS(){
	return this->r[REG_SREG] & 0x10;
}

void
Regs::setV(){
	this->r[REG_SREG] |= 0x8;
}

void
Regs::clearV(){
	this->r[REG_SREG] &= 0xf7;
}

bool
Regs::isV(){
	return this->r[REG_SREG] & 0x8;
}

void
Regs::setN(){
	this->r[REG_SREG] |= 0x4;
}

void
Regs::clearN(){
	this->r[REG_SREG] &= 0xfb;
}

bool
Regs::isN(){
	return this->r[REG_SREG] & 0x4;
}

void
Regs::setZ(){
	this->r[REG_SREG] |= 0x2;
}

void
Regs::clearZ(){
	this->r[REG_SREG] &= 0xfd;
}

bool
Regs::isZ(){
	return this->r[REG_SREG] & 0x2;
}

void
Regs::setC(){
	this->r[REG_SREG] |= 0x1;
}

void
Regs::clearC(){
	this->r[REG_SREG] &= 0xfe;
}

bool
Regs::isC(){
	return this->r[REG_SREG] & 0x1;
}

uint16_t
Regs::getX(){
	return ((this->r[REG_R27] << 8) | (this->r[REG_R26]));
}

void
Regs::setX(uint16_t X){
	this->r[REG_R27] = (uint8_t)(X >> 8);
	this->r[REG_R26] = (uint8_t)(X);
}

uint16_t
Regs::getY(){
	return ((this->r[REG_R29] << 8) | (this->r[REG_R28]));
}

void
Regs::setY(uint16_t Y){
	this->r[REG_R29] = (uint8_t)(Y >> 8);
	this->r[REG_R28] = (uint8_t)(Y);
}

uint16_t
Regs::getZ(){
	return ((this->r[REG_R31] << 8) | (this->r[REG_R30]));
}

void
Regs::setZ(uint16_t Z){
	this->r[REG_R31] = (uint8_t)(Z >> 8);
	this->r[REG_R30] = (uint8_t)(Z);
}

uint16_t
Regs::getSP(){
	return ((this->r[REG_SPH] << 8) | (this->r[REG_SPL]));
}

void
Regs::setSP(uint16_t SP){
	this->r[REG_SPH] = (uint8_t)(SP >> 8);
	this->r[REG_SPL] = (uint8_t)(SP);
}

std::string
Regs::getName(unsigned int r){
	return reg_names[r];
}
