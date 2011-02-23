/**
    avr.cc
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

#include <iostream>
#include <iomanip>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include "avr.hh"

Avr::Avr(){
#ifdef DEBUG
	std::cout << "Avr()" << std::endl;
#endif
	initPins();
	initSRAM(0x1000);			// 4 KB SRAM
	initFLASH(0x10000);			// 128 KB Flash
	initEEPROM(0x1000);			// 4 KB EEPROM
	regs.setMem(this->sram);
	regs.init();
	this->cycles = 0;
	this->instructions = 0;
	this->watchdog_timer = 0;
	this->stopped = false;
	this->sleeping = false;
	
#ifdef DEBUG
	regs.dump();
#endif
	std::cout << "SRAM Size = " << (this->msize/1024) << " KB" << std::endl;
	std::cout << "FLASH Size = " << (this->fsize/1024) << " KB" << std::endl;
	std::cout << "EEPROM Size = " << (this->esize/1024) << " KB" << std::endl;
}

Avr::~Avr(){
#ifdef DEBUG
	std::cout << "~Avr()" << std::endl;
#endif
}

void
Avr::initSRAM(unsigned int size){
		this->msize = size * sizeof(uint8_t);
		// + 0x100 = 32 Regs + 64 I/O + 160 Additional I/O
		this->sram = (uint8_t *)malloc(this->msize + 0x100);
}

void
Avr::initFLASH(unsigned int size){
		this->fsize = size * sizeof(uint16_t);
		this->flash = (uint16_t *)malloc(this->fsize);
}

void
Avr::initEEPROM(unsigned int size){
		this->esize = size * sizeof(uint8_t);
		this->eeprom = (uint8_t *)malloc(this->esize);
}

void
Avr::loadImage(std::string image){
	fw.setFileName(image);
	if(!fw.loadImage()){
		std::cerr << "Error: Loading Firmware Image" << std::endl;
		exit(0);
	}
	if(fw.getSize()>1024)
		std::cout << "Firmware Size = " << (fw.getSize()/1024) << " KB" << std::endl;
	else
		std::cout << "Firmware Size = " << fw.getSize() << " Bytes" << std::endl;
	const char *m = fw.getFirm();
	bcopy(m, this->flash, fw.getSize());
}

unsigned int
Avr::getCycles(void){
		return this->cycles;
}

Pin*
Avr::getPins(void){
		return this->pins;
}

void toBitArray(uint16_t word, uint8_t* bits){
	unsigned int bitmask = 1;
	for(int i=0 ; i<16 ; i++){
		if(word & bitmask)
			bits[i] = 1;
		else
			bits[i] = 0;
		bitmask <<= 1;
	}
}

unsigned int
Avr::getBit(uint8_t byte, unsigned int bit){
	if(byte & (1 << bit))
		return 1;
	else
		return 0;
}

unsigned int
Avr::getBit(uint16_t byte, unsigned int bit){
	if(byte & (1 << bit))
		return 1;
	else
		return 0;
}

void
Avr::setBit(uint8_t &byte, unsigned int bit){
	byte |= (1 << bit);
}

void
Avr::clearBit(uint8_t &byte, unsigned int bit){
	byte &= ~(1 << bit);	
}

void
Avr::reverseEndian(uint16_t &word){
	word = ((word << 8) | (word >> 8));
}

void
Avr::step(void){
	if(this->regs.pc*2 >= fw.getSize()){
		std::cout << "FLASH: ILLEGAL ADDRESS: ";
		std::cout << std::hex << this->regs.pc*2 << std::dec << std::endl;
		this->stopped = true;
		return;
	}
	opcode = this->flash[this->regs.pc];
#ifdef DEBUG
	std::cout << "PC=" << std::hex << std::setw(4) << this->regs.pc << ", opcode=" << std::setw(4) << (int)opcode << ", ";
	std::cout << std::dec;
#endif
	uint64_t cycles_prev = this->cycles;
	readPins();

	if((opcode & MSK_ADC) == OP_ADC)
		_adc();
	else if((opcode & MSK_ADD) == OP_ADD)
		_add();
	else if((opcode & MSK_ADIW) == OP_ADIW)
		_adiw();
	else if((opcode & MSK_AND) == OP_AND)
		_and();
	else if((opcode & MSK_ANDI) == OP_ANDI)
		_andi();
	else if((opcode & MSK_ASR) == OP_ASR)
		_asr();
	else if((opcode & MSK_BCLR) == OP_BCLR)
		_bclr();
	else if((opcode & MSK_BLD) == OP_BLD)
		_bld();
	else if((opcode & MSK_BRBC) == OP_BRBC)
		_brbc();
	else if((opcode & MSK_BRBS) == OP_BRBS)
		_brbs();
	else if((opcode & MSK_BREAK) == OP_BREAK)
		_break();
	else if((opcode & MSK_BSET) == OP_BSET)
		_bset();
	else if((opcode & MSK_BST) == OP_BST)
		_bst();
	else if((opcode & MSK_CALL) == OP_CALL)
		_call();
	else if((opcode & MSK_CBI) == OP_CBI)
		_cbi();
	else if((opcode & MSK_COM) == OP_COM)
		_com();
	else if((opcode & MSK_CP) == OP_CP)
		_cp();
	else if((opcode & MSK_CPC) == OP_CPC)
		_cpc();
	else if((opcode & MSK_CPI) == OP_CPI)
		_cpi();
	else if((opcode & MSK_CPSE) == OP_CPSE)
		_cpse();
	else if((opcode & MSK_DEC) == OP_DEC)
		_dec();
	else if((opcode & MSK_ELPM1) == OP_ELPM1)
		_elpm();
	else if((opcode & MSK_ELPM2) == OP_ELPM2)
		_elpm();
	else if((opcode & MSK_ELPM3) == OP_ELPM3)
		_elpm();
	else if((opcode & MSK_EOR) == OP_EOR)
		_eor();
	else if((opcode & MSK_FMUL) == OP_FMUL)
		_fmul();
	else if((opcode & MSK_FMULS) == OP_FMULS)
		_fmuls();
	else if((opcode & MSK_FMULSU) == OP_FMULSU)
		_fmulsu();
	else if((opcode & MSK_ICALL) == OP_ICALL)
		_icall();
	else if((opcode & MSK_IJMP) == OP_IJMP)
		_ijmp();
	else if((opcode & MSK_IN) == OP_IN)
		_in();
	else if((opcode & MSK_INC) == OP_INC)
		_inc();
	else if((opcode & MSK_JMP) == OP_JMP)
		_jmp();
	else if((opcode & MSK_LDI) == OP_LDI)
		_ldi();
	// Conflicts with STY4 (st Y+q)
	/**
	else if((opcode & MSK_LDS) == OP_LDS)
		_lds();
	**/
	else if((opcode & MSK_LDS32) == OP_LDS32)
		_lds32();
	else if((opcode & MSK_LDX1) == OP_LDX1)
		_ldx();
	else if((opcode & MSK_LDX2) == OP_LDX2)
		_ldx();
	else if((opcode & MSK_LDX3) == OP_LDX3)
		_ldx();
	/**
	else if((opcode & MSK_LDY1) == OP_LDY1)
		_ldy();
	**/
	else if((opcode & MSK_LDY2) == OP_LDY2)
		_ldy();
	else if((opcode & MSK_LDY3) == OP_LDY3)
		_ldy();
	else if((opcode & MSK_LDY4) == OP_LDY4)
		_ldy();
	/**
	else if((opcode & MSK_LDZ1) == OP_LDZ1)
		_ldz();
	**/
	else if((opcode & MSK_LDZ2) == OP_LDZ2)
		_ldz();
	else if((opcode & MSK_LDZ3) == OP_LDZ3)
		_ldz();
	else if((opcode & MSK_LDZ4) == OP_LDZ4)
		_ldz();
	else if((opcode & MSK_LPM1) == OP_LPM1)
		_lpm();
	else if((opcode & MSK_LPM2) == OP_LPM2)
		_lpm();
	else if((opcode & MSK_LPM3) == OP_LPM3)
		_lpm();
	else if((opcode & MSK_LSR) == OP_LSR)
		_lsr();
	else if((opcode & MSK_MOV) == OP_MOV)
		_mov();
	else if((opcode & MSK_MOVW) == OP_MOVW)
		_movw();
	else if((opcode & MSK_MUL) == OP_MUL)
		_mul();
	else if((opcode & MSK_MULS) == OP_MULS)
		_muls();
	else if((opcode & MSK_MULSU) == OP_MULSU)
		_mulsu();
	else if((opcode & MSK_NEG) == OP_NEG)
		_neg();
	else if((opcode & MSK_NOP) == OP_NOP)
		_nop();
	else if((opcode & MSK_OR) == OP_OR)
		_or();
	else if((opcode & MSK_ORI) == OP_ORI)
		_ori();
	else if((opcode & MSK_OUT) == OP_OUT)
		_out();
	else if((opcode & MSK_POP) == OP_POP)
		_pop();
	else if((opcode & MSK_PUSH) == OP_PUSH)
		_push();
	else if((opcode & MSK_RCALL) == OP_RCALL)
		_rcall();
	else if((opcode & MSK_RET) == OP_RET)
		_ret();
	else if((opcode & MSK_RETI) == OP_RETI)
		_reti();
	else if((opcode & MSK_RJMP) == OP_RJMP)
		_rjmp();
	else if((opcode & MSK_ROR) == OP_ROR)
		_ror();
	else if((opcode & MSK_SBC) == OP_SBC)
		_sbc();
	else if((opcode & MSK_SBCI) == OP_SBCI)
		_sbci();
	else if((opcode & MSK_SBI) == OP_SBI)
		_sbi();
	else if((opcode & MSK_SBIC) == OP_SBIC)
		_sbic();
	else if((opcode & MSK_SBIS) == OP_SBIS)
		_sbis();
	else if((opcode & MSK_SBIW) == OP_SBIW)
		_sbiw();
	else if((opcode & MSK_SBRC) == OP_SBRC)
		_sbrc();
	else if((opcode & MSK_SBRS) == OP_SBRS)
		_sbrs();
	else if((opcode & MSK_SLEEP) == OP_SLEEP)
		_sleep();
	else if((opcode & MSK_STX1) == OP_STX1)
		_stx();
	else if((opcode & MSK_STX1) == OP_STX2)
		_stx();
	else if((opcode & MSK_STX1) == OP_STX3)
		_stx();
	//else if((opcode & MSK_STY1) == OP_STY1)
	//	_sty();
	else if((opcode & MSK_STY2) == OP_STY2)
		_sty();
	else if((opcode & MSK_STY3) == OP_STY3)
		_sty();
	else if((opcode & MSK_STY4) == OP_STY4)
		_sty();
	//else if((opcode & MSK_STZ1) == OP_STZ1)
	//	_stz();
	else if((opcode & MSK_STZ2) == OP_STZ2)
		_stz();
	else if((opcode & MSK_STZ3) == OP_STZ3)
		_stz();
	else if((opcode & MSK_STZ4) == OP_STZ4)
		_stz();
	// Conflicts with STZ4 (st Y+q)
	/**
	else if((opcode & MSK_STS) == OP_STS)
		_sts();
	**/
	else if((opcode & MSK_STS32) == OP_STS32)
		_sts32();
	else if((opcode & MSK_SUB) == OP_SUB)
		_sub();
	else if((opcode & MSK_SUBI) == OP_SUBI)
		_subi();
	else if((opcode & MSK_SWAP) == OP_SWAP)
		_swap();
	else if((opcode & MSK_WDR) == OP_WDR)
		_wdr();
	else
		illegal();
	this->regs.dump();
	writePins();
	for(unsigned int i=0 ; i<this->devices.size() ; i++)
		this->devices[i]->probe(cycles_prev);
	instructions++;
}

void
Avr::run(){
		while(true){
			if(this->stopped)
				break;
			if(this->sleeping){
				std::cerr << "SLEEPING" << std::endl;
				break;
				//continue;
			}
			step();
		}
}

void
Avr::illegal(){
		std::cout << "ILLEGAL INSTRUCTION" << std::endl;
		this->regs.dump();
		this->stopped = true;
}

void
Avr::_add(){
	uint8_t Rr = ((opcode >> 5) & 0x10) | (opcode & 0xf);
	uint8_t Rd = ((opcode >> 4) & 0x1f);
	uint8_t R = this->regs.r[Rd] + this->regs.r[Rr];
	int n, v;
	// check for the H flag
	unsigned int Rr3 = getBit(this->regs.r[Rr], 3);
	unsigned int Rd3 = getBit(this->regs.r[Rd], 3);
	unsigned int R3 = getBit(R, 3);
	if((Rd3&Rr3) | (Rr3&(!R3)) | ((!R3)&Rd3)) this->regs.setH();
	else this->regs.clearH();
	// check for the C flag
	unsigned int Rr7 = getBit(this->regs.r[Rr], 7);
	unsigned int Rd7 = getBit(this->regs.r[Rd], 7);
	unsigned int R7 = getBit(R, 7);
	if((Rd7&Rr7) | (Rr7&(!R7)) | ((!R7)&Rd7)) this->regs.setC();
	else this->regs.clearC();
	// check for the N flag
	if(R7) {this->regs.setN(); n=1;}
	else {this->regs.clearN(); n=0;}
	// check for the V flag
	if((Rd7&Rr7&(!R7)) | ((!Rd7)&(!Rr7)&R7)) {this->regs.setV(); v=1;}
	else {this->regs.clearV(); v=0;}
	// check for the S flag
	if(n^v) this->regs.setS();
	else this->regs.clearS();
	// check for the Z flag
	if(!R) this->regs.setZ();
	else this->regs.clearZ();
	// write back the result
	this->regs.r[Rd] = R;
	
	// disassemble
	std::cout << "add " << this->regs.getName(Rd) << ", " << this->regs.getName(Rr) << std::endl;
	
	this->regs.pc += 1;
	this->cycles += 1;
}

void
Avr::_adc(){
	uint8_t Rr = ((opcode >> 5) & 0x10) | (opcode & 0xf);
	//uint8_t Rd = ((opcode >> 4) & 0x10) | ((opcode & 0x1f0) >> 4);
	uint8_t Rd = ((opcode >> 4) & 0x1f);
	uint8_t R = this->regs.r[Rd] + this->regs.r[Rr];
	if(this->regs.isC()) R += 1;
	int n, v;
	// check for the H flag
	unsigned int Rr3 = getBit(this->regs.r[Rr], 3);
	unsigned int Rd3 = getBit(this->regs.r[Rd], 3);
	unsigned int R3 = getBit(R, 3);
	if((Rd3&Rr3) | (Rr3&(!R3)) | ((!R3)&Rd3)) this->regs.setH();
	else this->regs.clearH();
	// check for the C flag
	unsigned int Rr7 = getBit(this->regs.r[Rr], 7);
	unsigned int Rd7 = getBit(this->regs.r[Rd], 7);
	unsigned int R7 = getBit(R, 7);
	if((Rd7&Rr7) | (Rr7&(!R7)) | ((!R7)&Rd7)) this->regs.setC();
	else this->regs.clearC();
	// check for the N flag
	if(R7) {this->regs.setN(); n=1;}
	else {this->regs.clearN(); n=0;}
	// check for the V flag
	if((Rd7&Rr7&(!R7)) | ((!Rd7)&(!Rr7)&R7)) {this->regs.setV(); v=1;}
	else {this->regs.clearV(); v=0;}
	// check for the S flag
	if(n^v) this->regs.setS();
	else this->regs.clearS();
	// check for the Z flag
	if(!R) this->regs.setZ();
	else this->regs.clearZ();
	// write back the result
	this->regs.r[Rd] = R;
	
	// disassemble
	std::cout << "adc " << this->regs.getName(Rd) << ", " << this->regs.getName(Rr) << std::endl;
	
	this->regs.pc += 1;
	this->cycles += 1;
}

void
Avr::_ldi(){
	uint8_t Rd = ((opcode >> 4) & 0xf) + 16;
	uint8_t K = ((opcode >> 4) & 0xf0) | (opcode & 0xf);
	// write back the result
	this->regs.r[Rd] = K;
	
	// disassemble
	std::cout << "ldi " << this->regs.getName(Rd) << ", 0x" << std::hex << (unsigned int)K << std::dec << std::endl;

	this->regs.pc += 1;
	this->cycles += 1;
}

void
Avr::_and(){
	uint8_t Rr = ((opcode >> 5) & 0x10) | (opcode & 0xf);
	uint8_t Rd = ((opcode >> 4) & 0x1f);
	uint8_t R = this->regs.r[Rd] & this->regs.r[Rr];
	unsigned int R7 = getBit(R, 7);
	int n, v = 0;
	// check for the Z flag
	if(!R) this->regs.setZ();
	else this->regs.clearZ();
	// check for the N flag
	if(R7) {this->regs.setN(); n=1;}
	else {this->regs.clearN(); n=0;}
	// clear the V flag
	this->regs.clearV();
	// check for the S flag
	if(n^v) this->regs.setS();
	else this->regs.clearS();
	// write back the result
	this->regs.r[Rd] = R;
	
	// disassemble
	std::cout << "and " << this->regs.getName(Rd) << ", " << this->regs.getName(Rr) << std::endl;
	
	this->regs.pc += 1;
	this->cycles += 1;
}

void
Avr::_or(){
	uint8_t Rr = ((opcode >> 5) & 0x10) | (opcode & 0xf);
	uint8_t Rd = ((opcode >> 4) & 0x1f);
	uint8_t R = this->regs.r[Rd] | this->regs.r[Rr];
	unsigned int R7 = getBit(R, 7);
	int n, v = 0;
	// check for the Z flag
	if(!R) this->regs.setZ();
	else this->regs.clearZ();
	// check for the N flag
	if(R7) {this->regs.setN(); n=1;}
	else {this->regs.clearN(); n=0;}
	// clear the V flag
	this->regs.clearV();
	// check for the S flag
	if(n^v) this->regs.setS();
	else this->regs.clearS();
	// write back the result
	this->regs.r[Rd] = R;
	
	// disassemble
	std::cout << "or " << this->regs.getName(Rd) << ", " << this->regs.getName(Rr) << std::endl;
	
	this->regs.pc += 1;
	this->cycles += 1;
}

void
Avr::_eor(){
	uint8_t Rr = ((opcode >> 5) & 0x10) | (opcode & 0xf);
	uint8_t Rd = ((opcode >> 4) & 0x1f);
	uint8_t R = this->regs.r[Rd] ^ this->regs.r[Rr];
	unsigned int R7 = getBit(R, 7);
	int n, v = 0;
	// check for the Z flag
	if(!R) this->regs.setZ();
	else this->regs.clearZ();
	// check for the N flag
	if(R7) {this->regs.setN(); n=1;}
	else {this->regs.clearN(); n=0;}
	// clear the V flag
	this->regs.clearV();
	// check for the S flag
	if(n^v) this->regs.setS();
	else this->regs.clearS();
	// write back the result
	this->regs.r[Rd] = R;
	
	// disassemble
	std::cout << "eor " << this->regs.getName(Rd) << ", " << this->regs.getName(Rr) << std::endl;

	this->regs.pc += 1;
	this->cycles += 1;
}

void
Avr::_andi(){
	uint8_t Rd = ((opcode >> 4) & 0xf) + 16;
	uint8_t K = ((opcode >> 4) & 0xf0) | (opcode & 0xf);
	uint8_t R = this->regs.r[Rd] & K;
	unsigned int R7 = getBit(R, 7);
	int n, v = 0;
	// check for the Z flag
	if(!R) this->regs.setZ();
	else this->regs.clearZ();
	// check for the N flag
	if(R7) {this->regs.setN(); n=1;}
	else {this->regs.clearN(); n=0;}
	// clear the V flag
	this->regs.clearV();
	// check for the S flag
	if(n^v) this->regs.setS();
	else this->regs.clearS();
	// write back the result
	this->regs.r[Rd] = R;
	
	// disassemble
	std::cout << "andi " << this->regs.getName(Rd) << ", 0x" << std::hex << (unsigned int)K << std::dec << std::endl;

	this->regs.pc += 1;
	this->cycles += 1;
}

void
Avr::_ori(){
	uint8_t Rd = ((opcode >> 4) & 0xf) + 16;
	uint8_t K = ((opcode >> 4) & 0xf0) | (opcode & 0xf);
	uint8_t R = this->regs.r[Rd] | K;
	unsigned int R7 = getBit(R, 7);
	int n, v = 0;
	// check for the Z flag
	if(!R) this->regs.setZ();
	else this->regs.clearZ();
	// check for the N flag
	if(R7) {this->regs.setN(); n=1;}
	else {this->regs.clearN(); n=0;}
	// clear the V flag
	this->regs.clearV();
	// check for the S flag
	if(n^v) this->regs.setS();
	else this->regs.clearS();
	// write back the result
	this->regs.r[Rd] = R;
	
	// disassemble
	std::cout << "ori " << this->regs.getName(Rd) << ", 0x" << std::hex << (unsigned int)K << std::dec << std::endl;

	this->regs.pc += 1;
	this->cycles += 1;
}

void
Avr::_nop(){
	// disassemble
	std::cout << "nop" << std::endl;
	
	this->regs.pc += 1;
	this->cycles += 1;
}

void
Avr::_mov(){
	uint8_t Rr = (((opcode >> 5) & 0x10) | (opcode & 0xf));
	uint8_t Rd = ((opcode >> 4) & 0x1f);
	this->regs.r[Rd] = this->regs.r[Rr];
	
	// disassemble
	std::cout << "mov " << this->regs.getName(Rd) << ", " << this->regs.getName(Rr) << std::endl;

	this->regs.pc += 1;
	this->cycles += 1;
}

void
Avr::_sub(){
	uint8_t Rr = (((opcode >> 5) & 0x10) | (opcode & 0xf));
	uint8_t Rd = ((opcode >> 4) & 0x1f);
	uint8_t R = this->regs.r[Rd] - this->regs.r[Rr];
	int n, v;
	unsigned int Rr3 = getBit(this->regs.r[Rr], 3);
	unsigned int Rd3 = getBit(this->regs.r[Rd], 3);
	unsigned int R3 = getBit(R, 3);
	unsigned int Rr7 = getBit(this->regs.r[Rr], 7);
	unsigned int Rd7 = getBit(this->regs.r[Rd], 7);
	unsigned int R7 = getBit(R, 7);
	// check for the H flag
	if(((!Rd3)&Rr3) | (Rr3&R3) | (R3&(!Rd3))) this->regs.setH();
	else this->regs.clearH();
	// check for the N flag
	if(R7) {this->regs.setN(); n=1;}
	else {this->regs.clearN(); n=0;}
	// check for the V flag
	if((Rd7&(!Rr7)&(!R7)) | ((!Rd7)&Rr7&R7)) {this->regs.setV(); v=1;}
	else {this->regs.clearV(); v=0;}
	// check for the S flag
	if(n^v) this->regs.setS();
	else this->regs.clearS();
	// check for the Z flag
	if(!R) this->regs.setZ();
	else this->regs.clearZ();
	// check for the C flag
	if(((!Rd7)&Rr7) | (Rr7&R7) | (R7&(!Rd7))) this->regs.setC();
	else this->regs.clearC();
	// write back the result
	this->regs.r[Rd] = R;

	this->regs.pc += 1;
	this->cycles += 1;
		
	// disassemble
	std::cout << "sub " << this->regs.getName(Rd) << ", " << this->regs.getName(Rr) << std::endl;
}

void
Avr::_subi(){
	uint8_t Rd = ((opcode >> 4) & 0xf) + 16;
	uint8_t K = (((opcode >> 4) & 0xf0) | (opcode & 0xf));
	uint8_t R = this->regs.r[Rd] - K;
	int n, v;
	unsigned int Rd3 = getBit(this->regs.r[Rd], 3);
	unsigned int R3 = getBit(R, 3);
	unsigned int K3 = getBit(K, 3);
	unsigned int Rd7 = getBit(this->regs.r[Rd], 7);
	unsigned int R7 = getBit(R, 7);
	unsigned int K7 = getBit(K, 7);
	// check for the H flag
	if(((!Rd3)&K3) | (K3&R3) | (R3&(!Rd3))) this->regs.setH();
	else this->regs.clearH();
	// check for the N flag
	if(R7) {this->regs.setN(); n=1;}
	else {this->regs.clearN(); n=0;}
	// check for the V flag
	if((Rd7&(!K7)&(!R7)) | ((!Rd7)&K7&R7)) {this->regs.setV(); v=1;}
	else {this->regs.clearV(); v=0;}
	// check for the S flag
	if(n^v) this->regs.setS();
	else this->regs.clearS();
	// check for the Z flag
	if(!R) this->regs.setZ();
	else this->regs.clearZ();
	// check for the C flag
	if(((!Rd7)&K7) | (K7&R7) | (R7&(!Rd7))) this->regs.setC();
	else this->regs.clearC();
	// write back the result
	this->regs.r[Rd] = R;

	// disassemble
	std::cout << "subi " << this->regs.getName(Rd) << ", 0x" << std::hex << (unsigned int)K << std::dec << std::endl;
		
	this->regs.pc += 1;
	this->cycles += 1;
}

void
Avr::_adiw(){
	uint8_t d = (opcode >> 4) & 0x3;
	uint8_t K = (((opcode >> 2) & 0x30) | (opcode & 0xf));
	uint8_t Rdl= (d << 1) + 24;			// Rd = 24 + Rd*2
	uint8_t Rdh= Rdl + 1;				// Rd = 24 + Rd*2 + 1
	uint16_t R = ((this->regs.r[Rdh] << 8) | (this->regs.r[Rdl])) + K;
	int n, v;
	unsigned int Rdh7 = getBit(this->regs.r[Rdh], 7);
	unsigned int R15 = getBit(R, 15);
	// check for the N flag
	if(R15) {this->regs.setN(); n=1;}
	else {this->regs.clearN(); n=0;}
	// check for the V flag
	if((!Rdh7) & R15) {this->regs.setV(); v=1;}
	else {this->regs.clearV(); v=0;}
	// check for the S flag
	if(n^v) this->regs.setS();
	else this->regs.clearS();
	// check for the Z flag
	if(!R) this->regs.setZ();
	else this->regs.clearZ();
	// check for the C flag
	if((!R15)&Rdh7) this->regs.setC();
	else this->regs.clearC();
	// write back the result
	this->regs.r[Rdl] = (uint8_t)(R);
	this->regs.r[Rdh] = (uint8_t)(R >> 8);

	// disassemble
	std::cout << "adiw " << this->regs.getName(Rdl) << ", 0x" << std::hex << (unsigned int)K << std::dec << std::endl;

	this->regs.pc += 1;
	this->cycles += 2;
}

void
Avr::_sbiw(){
	uint8_t d = (opcode >> 4) & 0x3;
	uint8_t K = (((opcode >> 2) & 0x30) | (opcode & 0xf));
	uint8_t Rdl = (d << 1) + 24;			// Rd = 24 + Rd*2
	uint8_t Rdh = Rdl + 1;					// Rd = 24 + Rd*2 + 1
	uint16_t R = ((this->regs.r[Rdh] << 8) | (this->regs.r[Rdl])) - K;
	int n, v;
	unsigned int Rdh7 = getBit(this->regs.r[Rdh], 7);
	unsigned int R15 = getBit(R, 15);
	// check for the N flag
	if(R15) {this->regs.setN(); n=1;}
	else {this->regs.clearN(); n=0;}
	// check for the V flag
	if(Rdh7 & (!R15)) {this->regs.setV(); v=1;}
	else {this->regs.clearV(); v=0;}
	// check for the S flag
	if(n^v) this->regs.setS();
	else this->regs.clearS();
	// check for the Z flag
	if(!R) this->regs.setZ();
	else this->regs.clearZ();
	// check for the C flag
	if(R15&(!Rdh7)) this->regs.setC();
	else this->regs.clearC();
	// write back the result
	this->regs.r[Rdl] = (uint8_t)(R);
	this->regs.r[Rdh] = (uint8_t)(R >> 8);
	
	// disassemble
	std::cout << "sbiw " << this->regs.getName(Rdl) << ", 0x" << std::hex << (unsigned int)K << std::dec << std::endl;

	this->regs.pc += 1;
	this->cycles += 2;
}

void
Avr::_bclr(){
	uint8_t s = (opcode >> 4) & 0x7;
	switch(s){
			case 0:
				std::cout << "clc" << std::endl;
				this->regs.clearC();
				break;
			case 1:
				std::cout << "clz" << std::endl;
				this->regs.clearZ();
				break;
			case 2:
				std::cout << "cln" << std::endl;
				this->regs.clearN();
				break;
			case 3:
				std::cout << "clv" << std::endl;
				this->regs.clearV();
				break;
			case 4:
				std::cout << "cls" << std::endl;
				this->regs.clearS();
				break;
			case 5:
				std::cout << "clh" << std::endl;
				this->regs.clearH();
				break;
			case 6:
				std::cout << "clt" << std::endl;
				this->regs.clearT();
				break;
			case 7:
				std::cout << "cli" << std::endl;
				this->regs.clearI();
				break;
	}
	
	this->regs.pc += 1;
	this->cycles += 1;
}

void
Avr::_bset(){
	uint8_t s = (opcode >> 4) & 0x7;
	switch(s){
			case 0:
				std::cout << "sec" << std::endl;
				this->regs.setC();
				break;
			case 1:
				std::cout << "sez" << std::endl;
				this->regs.setZ();
				break;
			case 2:
				std::cout << "sen" << std::endl;
				this->regs.setN();
				break;
			case 3:
				std::cout << "sev" << std::endl;
				this->regs.setV();
				break;
			case 4:
				std::cout << "ses" << std::endl;
				this->regs.setS();
				break;
			case 5:
				std::cout << "seh" << std::endl;
				this->regs.setH();
				break;
			case 6:
				std::cout << "set" << std::endl;
				this->regs.setT();
				break;
			case 7:
				std::cout << "sei" << std::endl;
				this->regs.setI();
				break;
	}
	
	this->regs.pc += 1;
	this->cycles += 1;
}

void
Avr::_rjmp(){
	int16_t K = opcode & 0xfff;
	if(K & 0x800) 
			K = -1 * (((~K) + 1) & 0xfff);		// 2th complement

	// disassemble
	if(K<0)
		std::cout << "rjmp .-" << std::hex << (int)(K*-2) << "\t\t; 0x" << (int)((this->regs.pc + K + 1)*2) << std::dec << std::endl;
	else
		std::cout << "rjmp .+" << std::hex << (int)(K*2) << "\t\t; 0x" << (int)((this->regs.pc + K + 1)*2) << std::dec << std::endl;

	this->regs.pc += K + 1;
	this->cycles += 2;
}

void
Avr::_neg(){
	uint8_t Rd = (opcode >> 4) & 0x1f;
	uint8_t R = 0 - this->regs.r[Rd];
	int n, v;
	unsigned int Rd3 = getBit(this->regs.r[Rd], 3);
	unsigned int R3 = getBit(R, 3);
	unsigned int R7 = getBit(R, 7);
	// check for the H flag
	if(R3 | Rd3) this->regs.setH();
	else this->regs.clearH();
	// check for the N flag
	if(R7) {this->regs.setN(); n=1;}
	else {this->regs.clearN(); n=0;}
	// check for the V flag
	if(R == 0x80) {this->regs.setV(); v=1;}
	else {this->regs.clearV(); v=0;}
	// check for the S flag
	if(n^v) this->regs.setS();
	else this->regs.clearS();
	// check for the Z flag
	if(!R) this->regs.setZ();
	else this->regs.clearZ();
	// check for the C flag
	if(R) this->regs.setC();
	else this->regs.clearC();	
	// write back the result	
	this->regs.r[Rd] = R;
	
	// disassemble
	std::cout << "neg " << this->regs.getName(Rd) << std::endl;

	this->regs.pc += 1;
	this->cycles += 1;
}

void
Avr::_inc(){
	uint8_t Rd = (opcode >> 4) & 0x1f;
	uint8_t R = this->regs.r[Rd] + 1;
	int n, v;
	unsigned int R7 = getBit(R, 7);
	// check for the N flag
	if(R7) {this->regs.setN(); n=1;}
	else {this->regs.clearN(); n=0;}
	// check for the V flag
	if(R == 0x80) {this->regs.setV(); v=1;}
	else {this->regs.clearV(); v=0;}
	// check for the S flag
	if(n^v) this->regs.setS();
	else this->regs.clearS();
	// check for the Z flag
	if(!R) this->regs.setZ();
	else this->regs.clearZ();
	// write back the result	
	this->regs.r[Rd] = R;
	
	// disassemble
	std::cout << "inc " << this->regs.getName(Rd) << std::endl;

	this->regs.pc += 1;
	this->cycles += 1;
}

void
Avr::_dec(){
	uint8_t Rd = (opcode >> 4) & 0x1f;
	uint8_t R = this->regs.r[Rd] - 1;
	int n, v;
	unsigned int R7 = getBit(R, 7);
	// check for the N flag
	if(R7) {this->regs.setN(); n=1;}
	else {this->regs.clearN(); n=0;}
	// check for the V flag
	if(R == 0x7f) {this->regs.setV(); v=1;}
	else {this->regs.clearV(); v=0;}
	// check for the S flag
	if(n^v) this->regs.setS();
	else this->regs.clearS();
	// check for the Z flag
	if(!R) this->regs.setZ();
	else this->regs.clearZ();
	// write back the result	
	this->regs.r[Rd] = R;
	
	// disassemble
	std::cout << "dec " << this->regs.getName(Rd) << std::endl;
	
	this->regs.pc += 1;
	this->cycles += 1;
}

void
Avr::_brbc(){
	uint8_t s = opcode & 0x7;
	int16_t K = (opcode >> 3) & 0x7f;
	if(K & 0x40) 
			K = -1 * (((~K) + 1) & 0x7f);		// 2th complement

	// disassemble
	if(K<0)
		std::cout << "brbc " << std::hex << (int)s << ", .-"  << (int)(K*-2) << "\t\t; 0x" << (int)((this->regs.pc + K + 1)*2) << std::dec << std::endl;
	else
		std::cout << "brbc " << std::hex << (int)s << ", .+"  << (int)(K*2) << "\t\t; 0x" << (int)((this->regs.pc + K + 1)*2) << std::dec << std::endl;


	if(!getBit(this->regs.r[REG_SREG], s)){
		this->regs.pc = this->regs.pc + 1 + K;
		this->cycles += 2;
	}
	else{
		this->regs.pc += 1;
		this->cycles += 1;
	}
}

void
Avr::_brbs(){
	uint8_t s = opcode & 0x7;
	int16_t K = (opcode >> 3) & 0x7f;
	if(K & 0x40) 
			K = -1 * (((~K) + 1) & 0x7f);		// 2th complement

	// disassemble
	if(K<0)
		std::cout << "brbs " << std::hex << (int)s << ", .-"  << (int)(K*-2) << "\t\t; 0x" << (int)((this->regs.pc + K + 1)*2) << std::dec << std::endl;
	else
		std::cout << "brbs " << std::hex << (int)s << ", .+"  << (int)(K*2) << "\t\t; 0x" << (int)((this->regs.pc + K + 1)*2) << std::dec << std::endl;

	if(getBit(this->regs.r[REG_SREG], s)){
		this->regs.pc = this->regs.pc + 1 + K;
		this->cycles += 2;
	}
	else{
		this->regs.pc += 1;
		this->cycles += 1;
	}
}

void
Avr::_com(){
	uint8_t Rd = (opcode >> 4) & 0x1f;
	uint8_t R = 0xff - this->regs.r[Rd];
	int n, v;
	unsigned int R7 = getBit(R, 7);
	// check for the N flag
	if(R7) {this->regs.setN(); n=1;}
	else {this->regs.clearN(); n=0;}
	// check for the V flag
	this->regs.clearV();
	v=0;
	// check for the S flag
	if(n^v) this->regs.setS();
	else this->regs.clearS();
	// check for the Z flag
	if(!R) this->regs.setZ();
	else this->regs.clearZ();
	// check for the C flag
	this->regs.setC();
	// write back the result	
	this->regs.r[Rd] = R;
	
	// disassemble
	std::cout << "com " << this->regs.getName(Rd) << std::endl;
	
	this->regs.pc += 1;
	this->cycles += 1;
}

void
Avr::_cp(){
	uint8_t Rr = ((opcode >> 5) & 0x10) | (opcode & 0xf);
	uint8_t Rd = ((opcode >> 4) & 0x1f);
	uint8_t R = this->regs.r[Rd] - this->regs.r[Rr];
	int n, v;
	unsigned int Rd3 = getBit(this->regs.r[Rd], 3);
	unsigned int Rr3 = getBit(this->regs.r[Rr], 3);
	unsigned int R3 = getBit(R, 3);
	unsigned int Rd7 = getBit(this->regs.r[Rd], 7);
	unsigned int Rr7 = getBit(this->regs.r[Rr], 7);
	unsigned int R7 = getBit(R, 7);
	// check for the H flag
	if(((!Rd3)&Rr3) | (Rr3&R3) | (R3&(!Rd3))) this->regs.setH();
	else this->regs.clearH();
	// check for the N flag
	if(R7) {this->regs.setN(); n=1;}
	else {this->regs.clearN(); n=0;}
	// check for the V flag
	if((Rd7&(!Rr7)&(!R7)) | ((!Rd7)&Rr7&R7)){this->regs.setV(); v=1;}
	else {this->regs.clearV(); v=0;}
	// check for the S flag
	if(n^v) this->regs.setS();
	else this->regs.clearS();
	// check for the Z flag
	if(!R) this->regs.setZ();
	else this->regs.clearZ();
	// check for the C flag
	if(((!Rd7)&Rr7) | (Rr7&R7) | (R7&(!Rd7))) this->regs.setC();
	else this->regs.clearC();
	
	// disassemble
	std::cout << "cp " << this->regs.getName(Rd) << ", " << this->regs.getName(Rr) << std::endl;
	
	this->regs.pc += 1;
	this->cycles += 1;
}

void
Avr::_cpc(){
	uint8_t Rr = ((opcode >> 5) & 0x10) | (opcode & 0xf);
	uint8_t Rd = ((opcode >> 4) & 0x1f);
	uint8_t R = this->regs.r[Rd] - this->regs.r[Rr];
	if(this->regs.isC())
		R -= 1;
	int n, v;
	unsigned int Rd3 = getBit(this->regs.r[Rd], 3);
	unsigned int Rr3 = getBit(this->regs.r[Rr], 3);
	unsigned int R3 = getBit(R, 3);
	unsigned int Rd7 = getBit(this->regs.r[Rd], 7);
	unsigned int Rr7 = getBit(this->regs.r[Rr], 7);
	unsigned int R7 = getBit(R, 7);
	// check for the H flag
	if(((!Rd3)&Rr3) | (Rr3&R3) | (R3&(!Rd3))) this->regs.setH();
	else this->regs.clearH();
	// check for the N flag
	if(R7) {this->regs.setN(); n=1;}
	else {this->regs.clearN(); n=0;}
	// check for the V flag
	if((Rd7&(!Rr7)&(!R7)) | ((!Rd7)&Rr7&R7)){this->regs.setV(); v=1;}
	else {this->regs.clearV(); v=0;}
	// check for the S flag
	if(n^v) this->regs.setS();
	else this->regs.clearS();
	// check for the Z flag
	if(!R) this->regs.setZ();
	else this->regs.clearZ();
	// check for the C flag
	if(((!Rd7)&Rr7) | (Rr7&R7) | (R7&(!Rd7))) this->regs.setC();
	else this->regs.clearC();
	
	// disassemble
	std::cout << "cpc " << this->regs.getName(Rd) << ", " << this->regs.getName(Rr) << std::endl;
	
	this->regs.pc += 1;
	this->cycles += 1;
}

void
Avr::_cpi(){
	uint8_t Rd = ((opcode >> 4) & 0xf) + 16;
	uint8_t K = (((opcode & 0xf00) >> 4) | (opcode & 0xf));
	uint8_t R = this->regs.r[Rd] - K;
	int n, v;
	unsigned int Rd3 = getBit(this->regs.r[Rd], 3);
	unsigned int K3 = getBit(K, 3);
	unsigned int R3 = getBit(R, 3);
	unsigned int Rd7 = getBit(this->regs.r[Rd], 7);
	unsigned int K7 = getBit(K, 7);
	unsigned int R7 = getBit(R, 7);
	// check for the H flag
	if(((!Rd3)&K3) | (K3&R3) | (R3&(!Rd3))) this->regs.setH();
	else this->regs.clearH();
	// check for the N flag
	if(R7) {this->regs.setN(); n=1;}
	else {this->regs.clearN(); n=0;}
	// check for the V flag
	if((Rd7&(!K7)&(!R7)) | ((!Rd7)&K7&R7)){this->regs.setV(); v=1;}
	else {this->regs.clearV(); v=0;}
	// check for the S flag
	if(n^v) this->regs.setS();
	else this->regs.clearS();
	// check for the Z flag
	if(!R) this->regs.setZ();
	else this->regs.clearZ();
	// check for the C flag
	if(((!Rd7)&K7) | (K7&R7) | (R7&(!Rd7))) this->regs.setC();
	else this->regs.clearC();
	
	// disassemble
	std::cout << "cpi " << this->regs.getName(Rd) << ", 0x" << std::hex << (unsigned int)K << std::dec << std::endl;
	
	this->regs.pc += 1;
	this->cycles += 1;
}

void
Avr::_lsr(){
	uint8_t Rd = (opcode >> 4) & 0x1f;
	uint8_t R = this->regs.r[Rd] >> 1;
	int n=0, c, v;
	unsigned int Rd0 = getBit(this->regs.r[Rd], 0);
	// check for the N flag
	this->regs.clearN();
	// check for the C flag
	if(Rd0){this->regs.setC(); c=1;}
	else{this->regs.clearC(); c=0;}
	// check for the V flag
	if(n^c){this->regs.setV(); v=1;}
	else{this->regs.clearV(); v=0;}
	// check for the S flag
	if(n^v) this->regs.setS();
	else this->regs.clearS();
	// check for the Z flag
	if(!R) this->regs.setZ();
	else this->regs.clearZ();
	// write back the result	
	this->regs.r[Rd] = R;
	
	// disassemble
	std::cout << "lsr " << this->regs.getName(Rd) << std::endl;
		
	this->regs.pc += 1;
	this->cycles += 1;
}

void
Avr::_swap(){
	uint8_t Rd = (opcode >> 4) & 0x1f;
	uint8_t R = (this->regs.r[Rd] >> 4) | ((this->regs.r[Rd] & 0xf) << 4);
	// write back the result
	this->regs.r[Rd] = R;

	// disassemble
	std::cout << "swap " << this->regs.getName(Rd) << std::endl;
	
	this->regs.pc += 1;
	this->cycles += 1;
}

void
Avr::_push(){
	uint8_t Rr = (opcode >> 4) & 0x1f;
	uint16_t SP = ((this->regs.r[REG_SPH] << 8) | this->regs.r[REG_SPL]);
	
	this->sram[SP] = this->regs.r[Rr];
	
	SP -= 1;
	this->regs.r[REG_SPH] = (uint8_t)(SP >> 8);
	this->regs.r[REG_SPL] = (uint8_t)(SP);
	
	// disassemble
	std::cout << "push " << this->regs.getName(Rr) << std::endl;
		
	this->regs.pc += 1;
	this->cycles += 2;
}

void
Avr::_pop(){
	uint8_t Rr = (opcode >> 4) & 0x1f;
	uint16_t SP = ((this->regs.r[REG_SPH] << 8) | this->regs.r[REG_SPL]);
	
	SP += 1;
	this->regs.r[REG_SPH] = (uint8_t)(SP >> 8);
	this->regs.r[REG_SPL] = (uint8_t)(SP);
	
	this->regs.r[Rr] = this->sram[SP];
	
	// disassemble
	std::cout << "pop " << this->regs.getName(Rr) << std::endl;
		
	this->regs.pc += 1;
	this->cycles += 2;
}

void
Avr::_out(){
	uint8_t Rr = (opcode >> 4) & 0x1f;
	uint8_t A = (((opcode >> 5) & 0x30) | (opcode & 0xf)) + 0x20;
	this->sram[A] = this->regs.r[Rr];

	// disassemble
	std::cout << "out 0x" << std::hex << (unsigned int)(A-0x20) << std::dec << ", " << this->regs.getName(Rr) << std::endl;

	this->regs.pc += 1;
	this->cycles += 1;
}

void
Avr::_in(){
	uint8_t Rd = (opcode >> 4) & 0x1f;
	uint8_t A = (((opcode >> 5) & 0x30) | (opcode & 0xf)) + 0x20;
	this->regs.r[Rd] = this->sram[A];

	// disassemble
	std::cout << "in " << this->regs.getName(Rd) << ", 0x" << std::hex << (unsigned int)(A-0x20) << std::dec << std::endl;

	this->regs.pc += 1;
	this->cycles += 1;
}

void
Avr::_rcall(){
	int16_t K = opcode & 0xfff;
	if(K & 0x800) 
			K = -1 * (((~K) + 1) & 0xfff);		// 2th complement

	this->sram[this->regs.getSP()] = (uint8_t)(this->regs.pc + 1);
	this->sram[this->regs.getSP()-1] = (uint8_t)((this->regs.pc + 1) >> 8);
	
	this->regs.setSP(this->regs.getSP()-2);
	
	// disassemble
	if(K<0)
		std::cout << "rcall .-" << std::hex << (int)(K*-2) << "\t\t; 0x" << (int)((this->regs.pc + K + 1)*2) << std::dec << std::endl;
	else
		std::cout << "rcall .+" << std::hex << (int)(K*2) << "\t\t; 0x" << (int)((this->regs.pc + K + 1)*2) << std::dec << std::endl;
	
	this->regs.pc += K + 1;
	this->cycles += 3;
}

void
Avr::_ret(){
	this->regs.dump();
	this->regs.setSP(this->regs.getSP()+2);

	// disassemble
	std::cout << "ret\t\t" << "; 0x" << ((int)((this->sram[this->regs.getSP()-1] << 8) | (this->sram[this->regs.getSP()])*2)) << std::endl;
	
	this->regs.pc = (this->sram[this->regs.getSP()-1] << 8) | (this->sram[this->regs.getSP()]);
	this->cycles += 4;
}

void
Avr::_ror(){
	uint8_t Rd = (opcode >> 4) & 0x1f;
	uint8_t R = this->regs.r[Rd] >> 1;
	if(this->regs.isC())
		R |= 0x80;
	unsigned int Rd0 = getBit(this->regs.r[Rd], 0);
	unsigned int R7 = getBit(R, 7);
	unsigned int n, c, v;
	// check for the C flag
	if(Rd0) {this->regs.setC(); c=1;}
	else {this->regs.clearC(); c=0;}
	// check for the Z flag
	if(!R) this->regs.setZ();
	else this->regs.clearZ();
	// check for the N flag
	if(R7) {this->regs.setN(); n=1;}
	else {this->regs.clearN(); n=0;}
	// check for the V flag
	if(n^c) {this->regs.setV(); v=1;}
	else {this->regs.clearV(); v=0;}
	// check for the S flag
	if(n^v) this->regs.setS();
	else this->regs.clearS();
	// write back the result
	this->regs.r[Rd] = R;
	
	// disassemble
	std::cout << "ror " << this->regs.getName(Rd) << std::endl;
		
	this->regs.pc += 1;	
	this->cycles += 1;
}

void
Avr::_sts(){
	uint8_t Rr = ((opcode >> 4) & 0xf) + 16;
	uint8_t K = ((opcode >> 4) & 0x7) | (opcode & 0xf);
	this->sram[K] = this->regs.r[Rr];
		
	// disassemble
	std::cout << "sts 0x" << std::hex << (unsigned int)(K) << std::dec << ", " << this->regs.getName(Rr) << std::endl;

	this->regs.pc += 1;	
	this->cycles += 1;
}

void
Avr::_sts32(){
	uint8_t Rr = ((opcode >> 4) & 0x1f);
	uint16_t K = this->flash[this->regs.pc+1];
	if(K >= this->msize)
		std::cout << "SRAM: ILLEGAL ADDRESS" << std::endl;
	this->sram[K] = this->regs.r[Rr];

	// disassemble
	std::cout << "sts 0x" << std::hex << (unsigned int)(K) << std::dec << ", " << this->regs.getName(Rr) << std::endl;
	
	this->regs.pc += 2;	
	this->cycles += 2;
}

void
Avr::_lds(){
	uint8_t Rd = ((opcode >> 4) & 0xf) + 16;
	uint8_t K = ((opcode >> 4) & 0x7) | (opcode & 0xf);
	this->regs.r[Rd] = this->sram[K];

	// disassemble
	std::cout << "lds " << this->regs.getName(Rd) << ", 0x" << std::hex << (unsigned int)(K) << std::dec << std::endl;

		
	this->regs.pc += 1;	
	this->cycles += 1;
}

void
Avr::_lds32(){
	uint8_t Rd = ((opcode >> 4) & 0x1f);
	uint16_t K = this->flash[this->regs.pc+1];
	if(K >= this->msize)
		std::cout << "SRAM: ILLEGAL ADDRESS" << std::endl;
	this->regs.r[Rd] = this->sram[K];

	// disassemble
	std::cout << "lds " << this->regs.getName(Rd) << ", 0x" << std::hex << (unsigned int)(K) << std::dec << std::endl;

	this->regs.pc += 2;	
	this->cycles += 2;
}

void
Avr::_stx(){
	uint8_t Rr = ((opcode >> 4) & 0x1f);
	uint16_t X = this->regs.getX();
	char sign = ' ';			// Only used by disassembler
	if((opcode & MSK_STX2) == OP_STX2){
		this->sram[X] = this->regs.r[Rr];
		++X;
		sign = '+';
	}
	else if((opcode & MSK_STX3) == OP_STX3){
		--X;
		this->sram[X] = this->regs.r[Rr];
		sign = '-';
	}
	else{
		this->sram[X] = this->regs.r[Rr];
	}
	this->regs.setX(X);
	
	// disassemble
	if(sign == '+')
		std::cout << "st x+, " << this->regs.getName(Rr) << std::endl;
	else if(sign == '-')
		std::cout << "st -x, " << this->regs.getName(Rr) << std::endl;
	else
		std::cout << "st x, " << this->regs.getName(Rr) << std::endl;

	this->regs.pc += 1;	
	this->cycles += 2;
}

void
Avr::_sty(){
	uint8_t Rr = ((opcode >> 4) & 0x1f);
	uint16_t Y = this->regs.getY();
	uint16_t q = 0;
	char sign = ' ';			// Only used by disassembler
	if((opcode & MSK_STY2) == OP_STY2){
		this->sram[Y] = this->regs.r[Rr];
		++Y;
		sign = '+';
	}
	else if((opcode & MSK_STY3) == OP_STY3){
		--Y;
		this->sram[Y] = this->regs.r[Rr];
		sign = '-';
	}
	else{
		q = ((opcode >> 8) & 0x20) | ((opcode >> 7) & 0x18) | (opcode & 0x7);
		this->sram[Y+q] = this->regs.r[Rr];
	}
	this->regs.setY(Y);
	
	// disassemble
	if(sign == '+')
		std::cout << "st y+, " << this->regs.getName(Rr) << std::endl;
	else if(sign == '-')
		std::cout << "st -y, " << this->regs.getName(Rr) << std::endl;
	else
		if(q)
			std::cout << "std y+" << (int)(q) << ", " << this->regs.getName(Rr) << std::endl;
		else
			std::cout << "st y, " << this->regs.getName(Rr) << std::endl;

	this->regs.pc += 1;	
	this->cycles += 2;
}

void
Avr::_stz(){
	uint8_t Rr = ((opcode >> 4) & 0x1f);
	uint16_t Z = this->regs.getZ();
	uint16_t q = 0;
	char sign = ' ';			// Only used by disassembler
	if((opcode & MSK_STZ2) == OP_STZ2){
		this->sram[Z] = this->regs.r[Rr];
		++Z;
		sign = '+';
	}
	else if((opcode & MSK_STZ3) == OP_STZ3){
		--Z;
		this->sram[Z] = this->regs.r[Rr];
		sign = '-';
	}
	else{
		q = ((opcode >> 8) & 0x20) | ((opcode >> 7) & 0x18) | (opcode & 0x7);
		this->sram[Z+q] = this->regs.r[Rr];
	}
	this->regs.setZ(Z);

	// disassemble
	if(sign == '+')
		std::cout << "st z+, " << this->regs.getName(Rr) << std::endl;
	else if(sign == '-')
		std::cout << "st -z, " << this->regs.getName(Rr) << std::endl;
	else
		if(q)
			std::cout << "std z+" << (int)(q) << ", " << this->regs.getName(Rr) << std::endl;
		else
			std::cout << "st z, " << this->regs.getName(Rr) << std::endl;
	
	this->regs.pc += 1;	
	this->cycles += 2;
}

void
Avr::_ldx(){
	uint8_t Rd = ((opcode >> 4) & 0x1f);
	uint16_t X = this->regs.getX();
	char sign = ' ';
	if((opcode & MSK_LDX2) == OP_LDX2){
		this->regs.r[Rd] = this->sram[X];
		++X;
		sign = '+';
	}
	else if((opcode & MSK_LDX3) == OP_LDX3){
		--X;
		this->regs.r[Rd] = this->sram[X];
		sign = '-';
	}
	else{
		this->regs.r[Rd] = this->sram[X];
	}
	this->regs.setX(X);
	
	// disassemble
	if(sign == '+')
		std::cout << "ld " << this->regs.getName(Rd) << ", x+" << std::endl;
	else if(sign == '-')
		std::cout << "ld " << this->regs.getName(Rd) << ", -x" << std::endl;
	else
		std::cout << "ld " << this->regs.getName(Rd) << ", x" << std::endl;

	this->regs.pc += 1;	
	this->cycles += 2;
}

void
Avr::_ldy(){
	uint8_t Rd = ((opcode >> 4) & 0x1f);
	uint16_t Y = this->regs.getY();
	uint16_t q = 0;
	char sign = ' ';
	if((opcode & MSK_LDY2) == OP_LDY2){
		this->regs.r[Rd] = this->sram[Y];
		++Y;
		sign = '+';
	}
	else if((opcode & MSK_LDY3) == OP_LDY3){
		--Y;
		this->regs.r[Rd] = this->sram[Y];
		sign = '-';
	}
	else{
		q = ((opcode >> 8) & 0x20) | ((opcode >> 7) & 0x18) | (opcode & 0x7);
		this->regs.r[Rd] = this->sram[Y+q];
	}
	this->regs.setY(Y);
	
	// disassemble
	if(sign == '+')
		std::cout << "ld " << this->regs.getName(Rd) << ", y+" << std::endl;
	else if(sign == '-')
		std::cout << "ld " << this->regs.getName(Rd) << ", -y" << std::endl;
	else
		if(q)
			std::cout << "ldd " << this->regs.getName(Rd) << ", y+" << (int)(q) << std::endl;
		else
			std::cout << "ld " << this->regs.getName(Rd) << ", y" << std::endl;
	
	this->regs.pc += 1;	
	this->cycles += 2;
}

void
Avr::_ldz(){
	uint8_t Rd = ((opcode >> 4) & 0x1f);
	uint16_t Z = this->regs.getZ();
	uint16_t q = 0;
	char sign = ' ';
	if((opcode & MSK_LDZ2) == OP_LDZ2){
		this->regs.r[Rd] = this->sram[Z];
		++Z;
		sign = '+';
	}
	else if((opcode & MSK_LDZ3) == OP_LDZ3){
		--Z;
		this->regs.r[Rd] = this->sram[Z];
		sign = '-';
	}
	else{
		q = ((opcode >> 8) & 0x20) | ((opcode >> 7) & 0x18) | (opcode & 0x7);
		this->regs.r[Rd] = this->sram[Z+q];
	}
	this->regs.setZ(Z);

	// disassemble
	if(sign == '+')
		std::cout << "ld " << this->regs.getName(Rd) << ", z+" << std::endl;
	else if(sign == '-')
		std::cout << "ld " << this->regs.getName(Rd) << ", -z" << std::endl;
	else
		if(q)
			std::cout << "ldd " << this->regs.getName(Rd) << ", z+" << (int)(q) << std::endl;
		else
			std::cout << "ld " << this->regs.getName(Rd) << ", z" << std::endl;
	
	this->regs.pc += 1;	
	this->cycles += 2;
}

void
Avr::_break(){
	stopped = true;
	
	std::cout << "break" << std::endl;

	this->regs.pc += 1;	
	this->cycles += 1;
}

void
Avr::_asr(){
	uint8_t Rd = (opcode >> 4) & 0x1f;
	uint8_t R = this->regs.r[Rd] >> 1;
	if(getBit(this->regs.r[Rd], 7))
		R |= 0x80;
	unsigned int Rd0 = getBit(this->regs.r[Rd], 0);
	unsigned int R7 = getBit(R, 7);
	unsigned int n, c, v;
	// check for the C flag
	if(Rd0) {this->regs.setC(); c=1;}
	else {this->regs.clearC(); c=0;}
	// check for the Z flag
	if(!R) this->regs.setZ();
	else this->regs.clearZ();
	// check for the N flag
	if(R7) {this->regs.setN(); n=1;}
	else {this->regs.clearN(); n=0;}
	// check for the V flag
	if(n^c) {this->regs.setV(); v=1;}
	else {this->regs.clearV(); v=0;}
	// check for the S flag
	if(n^v) this->regs.setS();
	else this->regs.clearS();
	// write back the result
	this->regs.r[Rd] = R;

	// disassemble
	std::cout << "asr " << this->regs.getName(Rd) << std::endl;

	this->regs.pc += 1;	
	this->cycles += 1;
}

void
Avr::_bld(){
	uint8_t Rd = (opcode >> 4) & 0x1f;
	uint8_t b = opcode & 0x7;
	uint8_t R = this->regs.r[Rd];
	if(this->regs.isT())
		setBit(R, (int)b);
	else
		clearBit(R, (int)b);
	// write back the result
	this->regs.r[Rd] = R;

	// disassemble
	std::cout << "bld " << this->regs.getName(Rd) << ", " << (int)(b) << std::endl;

	this->regs.pc += 1;
	this->cycles += 1;
}

void
Avr::_bst(){
	uint8_t Rd = (opcode >> 4) & 0x1f;
	uint8_t b = opcode & 0x7;
	if(getBit(this->regs.r[Rd], (int)b))
		this->regs.setT();
	else
		this->regs.clearT();

	// disassemble
	std::cout << "bst " << this->regs.getName(Rd) << ", " << (int)(b) << std::endl;
	
	this->regs.pc += 1;
	this->cycles += 1;
}

void
Avr::_cbi(){
	uint8_t A = ((opcode >> 3) & 0x1f) + 0x20;
	uint8_t b = opcode & 0x7;
	clearBit(this->sram[A], b);

	// disassemble
	std::cout << "cbi 0x" << std::hex << (int)(A-0x20) << std::dec << ", " << (int)(b) << std::endl;

	this->regs.pc += 1;
	this->cycles += 2;
}

void
Avr::_sbi(){
	uint8_t A = ((opcode >> 3) & 0x1f) + 0x20;
	uint8_t b = opcode & 0x7;
	setBit(this->sram[A], b);

	// disassemble
	std::cout << "sbi 0x" << std::hex << (int)(A-0x20) << std::dec << ", " << (int)(b) << std::endl;

	this->regs.pc += 1;
	this->cycles += 2;
}

void
Avr::_cpse(){
	uint8_t Rd = (opcode >> 4) & 0x1f;
	uint8_t Rr = ((opcode >> 5) & 0x10) | (opcode & 0xf);
	
	// disassemble
	std::cout << "cpse " << this->regs.getName(Rd) << ", " << this->regs.getName(Rr) << std::endl;

	if(this->regs.r[Rd] == this->regs.r[Rr]){
		uint16_t op2 = this->flash[this->regs.pc+1];
		unsigned int c = 2;
		if((op2 & MSK_CALL) == OP_CALL) ++c;
		else if((op2 & MSK_JMP) == OP_JMP) ++c;
		else if((op2 & MSK_LDS32) == OP_LDS32) ++c;
		else if((op2 & MSK_STS32) == OP_STS32) ++c;
		this->regs.pc += c;
		this->cycles += c;
	}else{
		this->regs.pc += 1;
		this->cycles += 1;
	}
}

void
Avr::_ijmp(){
	// disassemble
	std::cout << "ijmp" << std::endl;

	this->regs.pc = this->regs.getZ();
	this->cycles += 2;
}

void
Avr::_icall(){
	this->sram[this->regs.getSP()] = (uint8_t)(this->regs.pc + 1);
	this->sram[this->regs.getSP()-1] = (uint8_t)((this->regs.pc + 1) >> 8);
	
	this->regs.setSP(this->regs.getSP()-2);

	// disassemble
	std::cout << "icall" << std::endl;
	
	this->regs.pc = this->regs.getZ();
	this->cycles += 3;
}

void
Avr::_jmp(){
	/**
	 * Only 16-bit address are supported!!!
	 * ((opcode >> 3) & 0x3e) | (opcode & 0x1) not used
	 **/
	uint16_t K = this->flash[this->regs.pc+1];

	// disassemble
	std::cout << "jmp 0x" << std::hex << (unsigned int)(K*2) << std::dec << std::endl;

	this->regs.pc = K;
	this->cycles += 3;
}

void
Avr::_call(){
	/**
	 * Only 16-bit address are supported!!!
	 * ((opcode >> 3) & 0x3e) | (opcode & 0x1) not used
	 **/
	uint16_t K = this->flash[this->regs.pc+1];

	this->sram[this->regs.getSP()] = (uint8_t)(this->regs.pc + 2);
	this->sram[this->regs.getSP()-1] = (uint8_t)((this->regs.pc + 2) >> 8);
	
	this->regs.setSP(this->regs.getSP()-2);

	// disassemble
	std::cout << "call 0x" << std::hex << (unsigned int)(K*2) << std::dec << std::endl;
	
	this->regs.pc = K;
	this->cycles += 4;
}

void
Avr::_reti(){
	this->regs.setSP(this->regs.getSP()+2);
	this->regs.setI();

	// disassemble
	std::cout << "reti\t\t" << "; 0x" << ((int)((this->sram[this->regs.getSP()-1] << 8) | (this->sram[this->regs.getSP()])*2)) << std::endl;

	this->regs.pc = (this->sram[this->regs.getSP()-1] << 8) | (this->sram[this->regs.getSP()]);	
	this->cycles += 4;
}

void
Avr::_mul(){
	uint8_t Rr = ((opcode >> 5) & 0x10) | (opcode & 0xf);
	uint8_t Rd = ((opcode >> 4) & 0x1f);
	uint16_t R = this->regs.r[Rd] * this->regs.r[Rr];
	// check for the C flag
	unsigned int R15 = getBit(R, 15);
	if(R15) this->regs.setC();
	else this->regs.clearC();
	// check for the Z flag
	if(!R) this->regs.setZ();
	else this->regs.clearZ();
	// write back the result
	this->regs.r[REG_R0] = (uint8_t)(R);
	this->regs.r[REG_R1] = (uint8_t)(R >> 8);
	
	// disassemble
	std::cout << "mul " << this->regs.getName(Rd) << ", " << this->regs.getName(Rr) << std::endl;

	this->regs.pc += 1;
	this->cycles += 2;
}

void
Avr::_muls(){
	uint8_t r = (opcode & 0xf) + 16;
	uint8_t d = ((opcode >> 4) & 0xf) + 16;
	int8_t Rr = this->regs.r[r];
	int8_t Rd = this->regs.r[d];
	int16_t extRd = (int16_t)Rd, extRr = (int16_t)Rr;
	int16_t R = (uint16_t)(extRd * extRr);
	// check for the C flag
	unsigned int R15 = getBit((uint16_t)R, 15);
	if(R15) this->regs.setC();
	else this->regs.clearC();
	// check for the Z flag
	if(!R) this->regs.setZ();
	else this->regs.clearZ();
	// write back the result
	this->regs.r[REG_R0] = (uint8_t)(R);
	this->regs.r[REG_R1] = (uint8_t)(R >> 8);

	// disassemble
	std::cout << "muls " << this->regs.getName(d) << ", " << this->regs.getName(r) << std::endl;

	this->regs.pc += 1;
	this->cycles += 2;
}

void
Avr::_mulsu(){
	uint8_t r = (opcode & 0x7) + 16;
	uint8_t d = ((opcode >> 4) & 0x7) + 16;
	int8_t Rr = this->regs.r[r];
	int8_t Rd = this->regs.r[d];
	int16_t extRd = (int16_t)Rd, extRr = (int16_t)Rr;
	if(extRr & 0x8000)
		extRr &= 0x00ff;
	int16_t R = (uint16_t)(extRd * extRr);
	// check for the C flag
	unsigned int R15 = getBit((uint16_t)R, 15);
	if(R15) this->regs.setC();
	else this->regs.clearC();
	// check for the Z flag
	if(!R) this->regs.setZ();
	else this->regs.clearZ();
	// write back the result
	this->regs.r[REG_R0] = (uint8_t)(R);
	this->regs.r[REG_R1] = (uint8_t)(R >> 8);

	// disassemble
	std::cout << "mulsu " << this->regs.getName(d) << ", " << this->regs.getName(r) << std::endl;

	this->regs.pc += 1;
	this->cycles += 2;
}

void
Avr::_fmul(){
	uint8_t r = (opcode & 0x7) + 16;
	uint8_t d = ((opcode >> 4) & 0x7) + 16;
	uint8_t Rr = this->regs.r[r];
	uint8_t Rd = this->regs.r[d];
	uint16_t R = Rd * Rr;
	// check for the C flag
	unsigned int R15 = getBit(R, 15);
	if(R15) this->regs.setC();
	else this->regs.clearC();
	// check for the Z flag
	if(!R) this->regs.setZ();
	else this->regs.clearZ();
	// write back the result
	R <<= 1;
	this->regs.r[REG_R0] = (uint8_t)(R);
	this->regs.r[REG_R1] = (uint8_t)(R >> 8);
	
	// disassemble
	std::cout << "fmul " << this->regs.getName(d) << ", " << this->regs.getName(r) << std::endl;

	this->regs.pc += 1;
	this->cycles += 2;
}

void
Avr::_fmuls(){
	uint8_t r = (opcode & 0x7) + 16;
	uint8_t d = ((opcode >> 4) & 0x7) + 16;
	int8_t Rr = this->regs.r[r];
	int8_t Rd = this->regs.r[d];
	int16_t extRd = (int16_t)Rd, extRr = (int16_t)Rr;
	int16_t R = (uint16_t)(extRd * extRr);
	// check for the C flag
	unsigned int R15 = getBit((uint16_t)R, 15);
	if(R15) this->regs.setC();
	else this->regs.clearC();
	// check for the Z flag
	if(!R) this->regs.setZ();
	else this->regs.clearZ();
	// write back the result
	R <<= 1;
	this->regs.r[REG_R0] = (uint8_t)(R);
	this->regs.r[REG_R1] = (uint8_t)(R >> 8);

	// disassemble
	std::cout << "fmuls " << this->regs.getName(d) << ", " << this->regs.getName(r) << std::endl;

	this->regs.pc += 1;
	this->cycles += 2;
}

void
Avr::_fmulsu(){
	uint8_t r = (opcode & 0x7) + 16;
	uint8_t d = ((opcode >> 4) & 0x7) + 16;
	int8_t Rr = this->regs.r[r];
	int8_t Rd = this->regs.r[d];
	int16_t extRd = (int16_t)Rd, extRr = (int16_t)Rr;
	if(extRr & 0x8000)
		extRr &= 0x00ff;
	int16_t R = (uint16_t)(extRd * extRr);
	// check for the C flag
	unsigned int R15 = getBit((uint16_t)R, 15);
	if(R15) this->regs.setC();
	else this->regs.clearC();
	// check for the Z flag
	if(!R) this->regs.setZ();
	else this->regs.clearZ();
	// write back the result
	R <<= 1;
	this->regs.r[REG_R0] = (uint8_t)(R);
	this->regs.r[REG_R1] = (uint8_t)(R >> 8);

	// disassemble
	std::cout << "fmulsu " << this->regs.getName(d) << ", " << this->regs.getName(r) << std::endl;

	this->regs.pc += 1;
	this->cycles += 2;
}

void
Avr::_movw(){
	uint8_t Rr = (opcode & 0xf) << 1;
	uint8_t Rd = ((opcode >> 4) & 0xf) << 1;
	this->regs.r[Rd] = this->regs.r[Rr];
	this->regs.r[Rd+1] = this->regs.r[Rr+1];

	// disassemble
	std::cout << "movw " << this->regs.getName(Rd) << ", " << this->regs.getName(Rr) << std::endl;
	
	this->regs.pc += 1;
	this->cycles += 1;	
}

void
Avr::_sbrc(){
	uint8_t Rr = (opcode >> 4) & 0x1f;
	uint8_t b = opcode & 0x7;

	// disassemble
	std::cout << "sbrc " << this->regs.getName(Rr) << ", " << (int)(b) << std::endl;

	if(!getBit(this->regs.r[Rr], b)){
		uint16_t op2 = this->flash[this->regs.pc+1];
		unsigned int c = 2;
		if((op2 & MSK_CALL) == OP_CALL) ++c;
		else if((op2 & MSK_JMP) == OP_JMP) ++c;
		else if((op2 & MSK_LDS32) == OP_LDS32) ++c;
		else if((op2 & MSK_STS32) == OP_STS32) ++c;
		this->regs.pc += c;
		this->cycles += c;
	}
	else{
		this->regs.pc += 1;
		this->cycles += 1;
	}
}

void
Avr::_sbrs(){
	uint8_t Rr = (opcode >> 4) & 0x1f;
	uint8_t b = opcode & 0x7;

	// disassemble
	std::cout << "sbrs " << this->regs.getName(Rr) << ", " << (int)(b) << std::endl;

	if(getBit(this->regs.r[Rr], b)){
		uint16_t op2 = this->flash[this->regs.pc+1];
		unsigned int c = 2;
		if((op2 & MSK_CALL) == OP_CALL) ++c;
		else if((op2 & MSK_JMP) == OP_JMP) ++c;
		else if((op2 & MSK_LDS32) == OP_LDS32) ++c;
		else if((op2 & MSK_STS32) == OP_STS32) ++c;
		this->regs.pc += c;
		this->cycles += c;
	}
	else{
		this->regs.pc += 1;
		this->cycles += 1;
	}
}

void
Avr::_sleep(){
	this->sleeping = true;

	std::cout << "sleep" << std::endl;

	this->regs.pc += 1;
	this->cycles += 1;
}

void
Avr::_wdr(){
	this->watchdog_timer = 0;

	std::cout << "wdr" << std::endl;

	this->regs.pc += 1;
	this->cycles += 1;
}

void
Avr::_sbci(){
	uint8_t Rd = ((opcode >> 4) & 0xf) + 16;
	uint8_t K = (((opcode >> 4) & 0xf0) | (opcode & 0xf));
	uint8_t R = this->regs.r[Rd] - K;
	if(this->regs.isC())
		R -= 1;
	int n, v;
	unsigned int Rd3 = getBit(this->regs.r[Rd], 3);
	unsigned int R3 = getBit(R, 3);
	unsigned int K3 = getBit(K, 3);
	unsigned int Rd7 = getBit(this->regs.r[Rd], 7);
	unsigned int R7 = getBit(R, 7);
	unsigned int K7 = getBit(K, 7);
	// check for the H flag
	if(((!Rd3)&K3) | (K3&R3) | (R3&(!Rd3))) this->regs.setH();
	else this->regs.clearH();
	// check for the N flag
	if(R7) {this->regs.setN(); n=1;}
	else {this->regs.clearN(); n=0;}
	// check for the V flag
	if((Rd7&(!K7)&(!R7)) | ((!Rd7)&K7&R7)) {this->regs.setV(); v=1;}
	else {this->regs.clearV(); v=0;}
	// check for the S flag
	if(n^v) this->regs.setS();
	else this->regs.clearS();
	// check for the Z flag
	if(R) this->regs.clearZ();
	// check for the C flag
	if(((!Rd7)&K7) | (K7&R7) | (R7&(!Rd7))) this->regs.setC();
	else this->regs.clearC();
	// write back the result
	this->regs.r[Rd] = R;

	// disassemble
	std::cout << "sbci " << this->regs.getName(Rd) << ", 0x" << std::hex << (int)(K) << std::dec << std::endl;
	
	this->regs.pc += 1;
	this->cycles += 1;
}

void
Avr::_sbc(){
	uint8_t Rr = (((opcode >> 5) & 0x10) | (opcode & 0xf));
	uint8_t Rd = ((opcode >> 4) & 0x1f);
	uint8_t R = this->regs.r[Rd] - this->regs.r[Rr];
	if(this->regs.isC())
		R -= 1;
	int n, v;
	unsigned int Rr3 = getBit(this->regs.r[Rr], 3);
	unsigned int Rd3 = getBit(this->regs.r[Rd], 3);
	unsigned int R3 = getBit(R, 3);
	unsigned int Rr7 = getBit(this->regs.r[Rr], 7);
	unsigned int Rd7 = getBit(this->regs.r[Rd], 7);
	unsigned int R7 = getBit(R, 7);
	// check for the H flag
	if(((!Rd3)&Rr3) | (Rr3&R3) | (R3&(!Rd3))) this->regs.setH();
	else this->regs.clearH();
	// check for the N flag
	if(R7) {this->regs.setN(); n=1;}
	else {this->regs.clearN(); n=0;}
	// check for the V flag
	if((Rd7&(!Rr7)&(!R7)) | ((!Rd7)&Rr7&R7)) {this->regs.setV(); v=1;}
	else {this->regs.clearV(); v=0;}
	// check for the S flag
	if(n^v) this->regs.setS();
	else this->regs.clearS();
	// check for the Z flag
	if(R) this->regs.clearZ();
	// check for the C flag
	if(((!Rd7)&Rr7) | (Rr7&R7) | (R7&(!Rd7))) this->regs.setC();
	else this->regs.clearC();
	// write back the result
	this->regs.r[Rd] = R;

	// disassemble
	std::cout << "sbc " << this->regs.getName(Rd) << ", " << this->regs.getName(Rr) << std::endl;
	
	this->regs.pc += 1;
	this->cycles += 1;
}

void
Avr::_sbic(){
	uint8_t A = ((opcode >> 3) & 0x1f) + 0x20;
	uint8_t b = opcode & 0x7;

	// disassemble
	std::cout << "sbic 0x" << (int)(A-0x20) << ", " << (int)(b) << std::endl;

	if(!getBit(this->sram[A], b)){
		uint16_t op2 = this->flash[this->regs.pc+1];
		unsigned int c = 2;
		if((op2 & MSK_CALL) == OP_CALL) ++c;
		else if((op2 & MSK_JMP) == OP_JMP) ++c;
		else if((op2 & MSK_LDS32) == OP_LDS32) ++c;
		else if((op2 & MSK_STS32) == OP_STS32) ++c;
		this->regs.pc += c;
		this->cycles += c;
	}
	else{
		this->regs.pc += 1;
		this->cycles += 1;
	}
}

void
Avr::_sbis(){
	uint8_t A = ((opcode >> 3) & 0x1f) + 0x20;
	uint8_t b = opcode & 0x7;

	// disassemble
	std::cout << "sbis 0x" << (int)(A-0x20) << ", " << (int)(b) << std::endl;

	if(getBit(this->sram[A], b)){
		uint16_t op2 = this->flash[this->regs.pc+1];
		unsigned int c = 2;
		if((op2 & MSK_CALL) == OP_CALL) ++c;
		else if((op2 & MSK_JMP) == OP_JMP) ++c;
		else if((op2 & MSK_LDS32) == OP_LDS32) ++c;
		else if((op2 & MSK_STS32) == OP_STS32) ++c;
		this->regs.pc += c;
		this->cycles += c;
	}
	else{
		this->regs.pc += 1;
		this->cycles += 1;
	}
}

void
Avr::_lpm(){
	uint16_t Z = this->regs.getZ() >> 1;
	uint16_t word = this->flash[Z];
	uint8_t byte;
	std::string param;
	// High or Low byte ?
	if(this->regs.getZ() & 0x1)	// High
		byte = word >> 8;
	else						// Low
		byte = word & 0xff;
	if((this->opcode & MSK_LPM1) == OP_LPM1){
		this->regs.r[REG_R0] = byte;
		param = "";
	}
	else if((this->opcode & MSK_LPM2) == OP_LPM2){
		uint8_t Rd = (opcode >> 4) & 0x1f;
		this->regs.r[Rd] = byte;
		param = " " + this->regs.getName(Rd) + ", z";
	}
	else if((this->opcode & MSK_LPM3) == OP_LPM3){
		uint8_t Rd = (opcode >> 4) & 0x1f;
		this->regs.r[Rd] = byte;
		this->regs.setZ(this->regs.getZ()+1);
		param = " " + this->regs.getName(Rd) + ", z+";
	}

	// disassemble
	std::cout << "lpm" + param << std::endl;

	this->regs.pc += 1;
	this->cycles += 3;
}

void
Avr::_elpm(){
	uint32_t ZZ = (this->regs.r[REG_RAMPZ] << 16) | this->regs.getZ();
	uint32_t ZZZ = ZZ >> 1;
	uint16_t word = this->flash[ZZZ];
	uint8_t byte;
	std::string param;
	// High or Low byte ?
	if(this->regs.getZ() & 0x1)	// High
		byte = word >> 8;
	else						// Low
		byte = word & 0xff;
	if((this->opcode & MSK_ELPM1) == OP_ELPM1){
		this->regs.r[REG_R0] = byte;
		param = "";
	}
	else if((this->opcode & MSK_ELPM2) == OP_ELPM2){
		uint8_t Rd = (opcode >> 4) & 0x1f;
		this->regs.r[Rd] = byte;
		param = " " + this->regs.getName(Rd) + ", z";
	}
	else if((this->opcode & MSK_ELPM3) == OP_ELPM3){
		uint8_t Rd = (opcode >> 4) & 0x1f;
		this->regs.r[Rd] = byte;
		++ZZ;
		this->regs.setZ((uint16_t)ZZ);
		this->regs.r[REG_RAMPZ] = (uint8_t)(ZZ >> 16);
		param = " " + this->regs.getName(Rd) + ", z+";
	}

	// disassemble
	std::cout << "elpm" + param << std::endl;

	this->regs.pc += 1;
	this->cycles += 3;
}

void
Avr::addDevice(Device *d)
{
	this->devices.push_back(d);
}
