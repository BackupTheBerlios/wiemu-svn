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
	sram = new Mem(this);
	initFLASH(0x10000);			// 128 KB Flash
	initEEPROM(0x1000);			// 4 KB EEPROM
	regs = new Regs(this);
	regs->init();
	timer = new Timer();
	timer->setMCU(this);
	spi = new Spi();
	spi->setMCU(this);
	addInternalDevice((InternalDevice*)spi);
	adc = new Adc();
	adc->setMCU(this);
	addInternalDevice((InternalDevice*)adc);
	nextInterrupt = 0;
	clock.reset();
	clock.setFreq(AVR_FREQ);
	instructions = 0;
	watchdog_timer = 0;
	stopped = false;
	sleeping = false;
	debug = new Debugger();
	
#ifdef DEBUG
	regs->dump();
#endif
	std::cout << "SRAM Size = " << (sram->getSize()/1024) << " KB" << std::endl;
	std::cout << "FLASH Size = " << (fsize/1024) << " KB" << std::endl;
	std::cout << "EEPROM Size = " << (esize/1024) << " KB" << std::endl;
}

Avr::~Avr(){
#ifdef DEBUG
	std::cout << "~Avr()" << std::endl;
#endif
	delete regs;
	delete sram;
	delete timer;
	delete spi;
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

uint64_t
Avr::getCycles(void){
	return clock.getCycles();
}

Clock
Avr::getClock(void){
	return clock;
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
	regs->oldpc = regs->pc;

	if(stopped)
		return;
	if(!interrupts.empty()){
		Clock clock_prev = clock;
		interrupt(interrupts.front());
		interrupts.pop();
		timer->update(clock - clock_prev);
	}

	Clock clock_prev = clock;
	readPins();

	if(this->sleeping == false){
		if(regs->pc*2 >= fw.getSize()){
			debug->getFile() << "FLASH: ILLEGAL ADDRESS: ";
			debug->getFile() << std::hex << regs->pc*2 << std::dec << std::endl;
			this->stopped = true;
			return;
		}
		opcode = this->flash[regs->pc];
#ifdef DEBUG
		debug->getFile() << "PC=" << std::hex << std::setw(4) << regs->pc*2 /*<< ", opcode=" << std::setw(4) << (int)opcode << ", "*/;
		debug->getFile() << std::dec << "\t";
#endif
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
		// Increment the instructions' counter
		instructions++;
	}else{
		if((*sram)[REG_MCUCR + AVR_IO_BASE] & (1 << 5))
			clock++;
		else // TODO:
			std::cerr << "WAKEUP :)" << std::endl;
	}

	timer->update(clock - clock_prev);

	writePins();
}

void
Avr::run(){
		std::ofstream logf("pc.log");
		while(true){
			if(this->stopped)
				break;
			//if(this->sleeping){
				//std::cerr << "SLEEPING" << std::endl;
				//clock++;
				//timer->update(1);
			//}
			//uint16_t pc = regs->pc;
			//logf << "PC = " << std::hex << regs->pc*2 << std::dec << std::endl;
			if(regs->pc*2 == 0x11dc)
				std::cerr << getCycles() << " Inside if" << std::endl;
			if(regs->pc*2 == 0x11fe)
				std::cerr << getCycles() << " Inside if2" << std::endl;
			if(regs->pc*2 == 0x1202)
				std::cerr << getCycles() << " CALL 1" << std::endl;
			if(regs->pc*2 == 0x120e)
				std::cerr << getCycles() << " CALL 2" << std::endl;
			step();
		}
		regs->dump();
		logf.close();
}

void
Avr::illegal(){
		debug->getFile() << "ILLEGAL INSTRUCTION" << std::endl;
		regs->dump();
		this->stopped = true;
}

void
Avr::_add(){
	uint8_t Rr = ((opcode >> 5) & 0x10) | (opcode & 0xf);
	uint8_t Rd = ((opcode >> 4) & 0x1f);
	uint8_t R =(*regs)[Rd] +(*regs)[Rr];
	int n, v;
	// check for the H flag
	unsigned int Rr3 = getBit((*regs)[Rr], 3);
	unsigned int Rd3 = getBit((*regs)[Rd], 3);
	unsigned int R3 = getBit(R, 3);
	if((Rd3&Rr3) | (Rr3&(!R3)) | ((!R3)&Rd3)) regs->setH();
	else regs->clearH();
	// check for the C flag
	unsigned int Rr7 = getBit((*regs)[Rr], 7);
	unsigned int Rd7 = getBit((*regs)[Rd], 7);
	unsigned int R7 = getBit(R, 7);
	if((Rd7&Rr7) | (Rr7&(!R7)) | ((!R7)&Rd7)) regs->setC();
	else regs->clearC();
	// check for the N flag
	if(R7) {regs->setN(); n=1;}
	else {regs->clearN(); n=0;}
	// check for the V flag
	if((Rd7&Rr7&(!R7)) | ((!Rd7)&(!Rr7)&R7)) {regs->setV(); v=1;}
	else {regs->clearV(); v=0;}
	// check for the S flag
	if(n^v) regs->setS();
	else regs->clearS();
	// check for the Z flag
	if(!R) regs->setZ();
	else regs->clearZ();
	// write back the result
	(*regs)[Rd] = R;
	
	// disassemble
	if(debug->isFileOpened() && debug->isDisassemblyEnabled())
		debug->getFile() << "add " << regs->getName(Rd) << ", " << regs->getName(Rr) << std::endl;
	
	regs->pc += 1;
	clock += 1;
}

void
Avr::_adc(){
	uint8_t Rr = ((opcode >> 5) & 0x10) | (opcode & 0xf);
	//uint8_t Rd = ((opcode >> 4) & 0x10) | ((opcode & 0x1f0) >> 4);
	uint8_t Rd = ((opcode >> 4) & 0x1f);
	uint8_t R =(*regs)[Rd] +(*regs)[Rr];
	if(regs->isC()) R += 1;
	int n, v;
	// check for the H flag
	unsigned int Rr3 = getBit((*regs)[Rr], 3);
	unsigned int Rd3 = getBit((*regs)[Rd], 3);
	unsigned int R3 = getBit(R, 3);
	if((Rd3&Rr3) | (Rr3&(!R3)) | ((!R3)&Rd3)) regs->setH();
	else regs->clearH();
	// check for the C flag
	unsigned int Rr7 = getBit((*regs)[Rr], 7);
	unsigned int Rd7 = getBit((*regs)[Rd], 7);
	unsigned int R7 = getBit(R, 7);
	if((Rd7&Rr7) | (Rr7&(!R7)) | ((!R7)&Rd7)) regs->setC();
	else regs->clearC();
	// check for the N flag
	if(R7) {regs->setN(); n=1;}
	else {regs->clearN(); n=0;}
	// check for the V flag
	if((Rd7&Rr7&(!R7)) | ((!Rd7)&(!Rr7)&R7)) {regs->setV(); v=1;}
	else {regs->clearV(); v=0;}
	// check for the S flag
	if(n^v) regs->setS();
	else regs->clearS();
	// check for the Z flag
	if(!R) regs->setZ();
	else regs->clearZ();
	// write back the result
	(*regs)[Rd] = R;
	
	// disassemble
	if(debug->isFileOpened() && debug->isDisassemblyEnabled())
		debug->getFile() << "adc " << regs->getName(Rd) << ", " << regs->getName(Rr) << std::endl;
	
	regs->pc += 1;
	clock += 1;
}

void
Avr::_ldi(){
	uint8_t Rd = ((opcode >> 4) & 0xf) + 16;
	uint8_t K = ((opcode >> 4) & 0xf0) | (opcode & 0xf);
	// write back the result
	(*regs)[Rd] = K;
	
	// disassemble
	if(debug->isFileOpened() && debug->isDisassemblyEnabled())
		debug->getFile() << "ldi " << regs->getName(Rd) << ", 0x" << std::hex << (unsigned int)K << std::dec << std::endl;

	regs->pc += 1;
	clock += 1;
}

void
Avr::_and(){
	uint8_t Rr = ((opcode >> 5) & 0x10) | (opcode & 0xf);
	uint8_t Rd = ((opcode >> 4) & 0x1f);
	uint8_t R =(*regs)[Rd] &(*regs)[Rr];
	unsigned int R7 = getBit(R, 7);
	int n, v = 0;
	// check for the Z flag
	if(!R) regs->setZ();
	else regs->clearZ();
	// check for the N flag
	if(R7) {regs->setN(); n=1;}
	else {regs->clearN(); n=0;}
	// clear the V flag
	regs->clearV();
	// check for the S flag
	if(n^v) regs->setS();
	else regs->clearS();
	// write back the result
	(*regs)[Rd] = R;
	
	// disassemble
	if(debug->isFileOpened() && debug->isDisassemblyEnabled())
		debug->getFile() << "and " << regs->getName(Rd) << ", " << regs->getName(Rr) << std::endl;
	
	regs->pc += 1;
	clock += 1;
}

void
Avr::_or(){
	uint8_t Rr = ((opcode >> 5) & 0x10) | (opcode & 0xf);
	uint8_t Rd = ((opcode >> 4) & 0x1f);
	uint8_t R =(*regs)[Rd] |(*regs)[Rr];
	unsigned int R7 = getBit(R, 7);
	int n, v = 0;
	// check for the Z flag
	if(!R) regs->setZ();
	else regs->clearZ();
	// check for the N flag
	if(R7) {regs->setN(); n=1;}
	else {regs->clearN(); n=0;}
	// clear the V flag
	regs->clearV();
	// check for the S flag
	if(n^v) regs->setS();
	else regs->clearS();
	// write back the result
	(*regs)[Rd] = R;
	
	// disassemble
	if(debug->isFileOpened() && debug->isDisassemblyEnabled())
		debug->getFile() << "or " << regs->getName(Rd) << ", " << regs->getName(Rr) << std::endl;
	
	regs->pc += 1;
	clock += 1;
}

void
Avr::_eor(){
	uint8_t Rr = ((opcode >> 5) & 0x10) | (opcode & 0xf);
	uint8_t Rd = ((opcode >> 4) & 0x1f);
	uint8_t R =(*regs)[Rd] ^(*regs)[Rr];
	unsigned int R7 = getBit(R, 7);
	int n, v = 0;
	// check for the Z flag
	if(!R) regs->setZ();
	else regs->clearZ();
	// check for the N flag
	if(R7) {regs->setN(); n=1;}
	else {regs->clearN(); n=0;}
	// clear the V flag
	regs->clearV();
	// check for the S flag
	if(n^v) regs->setS();
	else regs->clearS();
	// write back the result
	(*regs)[Rd] = R;
	
	// disassemble
	if(debug->isFileOpened() && debug->isDisassemblyEnabled())
		debug->getFile() << "eor " << regs->getName(Rd) << ", " << regs->getName(Rr) << std::endl;

	regs->pc += 1;
	clock += 1;
}

void
Avr::_andi(){
	uint8_t Rd = ((opcode >> 4) & 0xf) + 16;
	uint8_t K = ((opcode >> 4) & 0xf0) | (opcode & 0xf);
	uint8_t R =(*regs)[Rd] & K;
	unsigned int R7 = getBit(R, 7);
	int n, v = 0;
	// check for the Z flag
	if(!R) regs->setZ();
	else regs->clearZ();
	// check for the N flag
	if(R7) {regs->setN(); n=1;}
	else {regs->clearN(); n=0;}
	// clear the V flag
	regs->clearV();
	// check for the S flag
	if(n^v) regs->setS();
	else regs->clearS();
	// write back the result
	(*regs)[Rd] = R;
	
	// disassemble
	if(debug->isFileOpened() && debug->isDisassemblyEnabled())
		debug->getFile() << "andi " << regs->getName(Rd) << ", 0x" << std::hex << (unsigned int)K << std::dec << std::endl;

	regs->pc += 1;
	clock += 1;
}

void
Avr::_ori(){
	uint8_t Rd = ((opcode >> 4) & 0xf) + 16;
	uint8_t K = ((opcode >> 4) & 0xf0) | (opcode & 0xf);
	uint8_t R =(*regs)[Rd] | K;
	unsigned int R7 = getBit(R, 7);
	int n, v = 0;
	// check for the Z flag
	if(!R) regs->setZ();
	else regs->clearZ();
	// check for the N flag
	if(R7) {regs->setN(); n=1;}
	else {regs->clearN(); n=0;}
	// clear the V flag
	regs->clearV();
	// check for the S flag
	if(n^v) regs->setS();
	else regs->clearS();
	// write back the result
	(*regs)[Rd] = R;
	
	// disassemble
	if(debug->isFileOpened() && debug->isDisassemblyEnabled())
		debug->getFile() << "ori " << regs->getName(Rd) << ", 0x" << std::hex << (unsigned int)K << std::dec << std::endl;

	regs->pc += 1;
	clock += 1;
}

void
Avr::_nop(){
	// disassemble
	if(debug->isFileOpened() && debug->isDisassemblyEnabled())
		debug->getFile() << "nop" << std::endl;
	
	regs->pc += 1;
	clock += 1;
}

void
Avr::_mov(){
	uint8_t Rr = (((opcode >> 5) & 0x10) | (opcode & 0xf));
	uint8_t Rd = ((opcode >> 4) & 0x1f);
	(*regs)[Rd] =(*regs)[Rr];
	
	// disassemble
	if(debug->isFileOpened() && debug->isDisassemblyEnabled())
		debug->getFile() << "mov " << regs->getName(Rd) << ", " << regs->getName(Rr) << std::endl;

	regs->pc += 1;
	clock += 1;
}

void
Avr::_sub(){
	uint8_t Rr = (((opcode >> 5) & 0x10) | (opcode & 0xf));
	uint8_t Rd = ((opcode >> 4) & 0x1f);
	uint8_t R =(*regs)[Rd] -(*regs)[Rr];
	int n, v;
	unsigned int Rr3 = getBit((*regs)[Rr], 3);
	unsigned int Rd3 = getBit((*regs)[Rd], 3);
	unsigned int R3 = getBit(R, 3);
	unsigned int Rr7 = getBit((*regs)[Rr], 7);
	unsigned int Rd7 = getBit((*regs)[Rd], 7);
	unsigned int R7 = getBit(R, 7);
	// check for the H flag
	if(((!Rd3)&Rr3) | (Rr3&R3) | (R3&(!Rd3))) regs->setH();
	else regs->clearH();
	// check for the N flag
	if(R7) {regs->setN(); n=1;}
	else {regs->clearN(); n=0;}
	// check for the V flag
	if((Rd7&(!Rr7)&(!R7)) | ((!Rd7)&Rr7&R7)) {regs->setV(); v=1;}
	else {regs->clearV(); v=0;}
	// check for the S flag
	if(n^v) regs->setS();
	else regs->clearS();
	// check for the Z flag
	if(!R) regs->setZ();
	else regs->clearZ();
	// check for the C flag
	if(((!Rd7)&Rr7) | (Rr7&R7) | (R7&(!Rd7))) regs->setC();
	else regs->clearC();
	// write back the result
	(*regs)[Rd] = R;

	regs->pc += 1;
	clock += 1;
		
	// disassemble
	if(debug->isFileOpened() && debug->isDisassemblyEnabled())
		debug->getFile() << "sub " << regs->getName(Rd) << ", " << regs->getName(Rr) << std::endl;
}

void
Avr::_subi(){
	uint8_t Rd = ((opcode >> 4) & 0xf) + 16;
	uint8_t K = (((opcode >> 4) & 0xf0) | (opcode & 0xf));
	uint8_t R =(*regs)[Rd] - K;
	int n, v;
	unsigned int Rd3 = getBit((*regs)[Rd], 3);
	unsigned int R3 = getBit(R, 3);
	unsigned int K3 = getBit(K, 3);
	unsigned int Rd7 = getBit((*regs)[Rd], 7);
	unsigned int R7 = getBit(R, 7);
	unsigned int K7 = getBit(K, 7);
	// check for the H flag
	if(((!Rd3)&K3) | (K3&R3) | (R3&(!Rd3))) regs->setH();
	else regs->clearH();
	// check for the N flag
	if(R7) {regs->setN(); n=1;}
	else {regs->clearN(); n=0;}
	// check for the V flag
	if((Rd7&(!K7)&(!R7)) | ((!Rd7)&K7&R7)) {regs->setV(); v=1;}
	else {regs->clearV(); v=0;}
	// check for the S flag
	if(n^v) regs->setS();
	else regs->clearS();
	// check for the Z flag
	if(!R) regs->setZ();
	else regs->clearZ();
	// check for the C flag
	if(((!Rd7)&K7) | (K7&R7) | (R7&(!Rd7))) regs->setC();
	else regs->clearC();
	// write back the result
	(*regs)[Rd] = R;

	// disassemble
	if(debug->isFileOpened() && debug->isDisassemblyEnabled())
		debug->getFile() << "subi " << regs->getName(Rd) << ", 0x" << std::hex << (unsigned int)K << std::dec << std::endl;
		
	regs->pc += 1;
	clock += 1;
}

void
Avr::_adiw(){
	uint8_t d = (opcode >> 4) & 0x3;
	uint8_t K = (((opcode >> 2) & 0x30) | (opcode & 0xf));
	uint8_t Rdl= (d << 1) + 24;			// Rd = 24 + Rd*2
	uint8_t Rdh= Rdl + 1;				// Rd = 24 + Rd*2 + 1
	uint16_t R = (((*regs)[Rdh] << 8) | ((*regs)[Rdl])) + K;
	int n, v;
	unsigned int Rdh7 = getBit((*regs)[Rdh], 7);
	unsigned int R15 = getBit(R, 15);
	// check for the N flag
	if(R15) {regs->setN(); n=1;}
	else {regs->clearN(); n=0;}
	// check for the V flag
	if((!Rdh7) & R15) {regs->setV(); v=1;}
	else {regs->clearV(); v=0;}
	// check for the S flag
	if(n^v) regs->setS();
	else regs->clearS();
	// check for the Z flag
	if(!R) regs->setZ();
	else regs->clearZ();
	// check for the C flag
	if((!R15)&Rdh7) regs->setC();
	else regs->clearC();
	// write back the result
	(*regs)[Rdl] = (uint8_t)(R);
	(*regs)[Rdh] = (uint8_t)(R >> 8);

	// disassemble
	if(debug->isFileOpened() && debug->isDisassemblyEnabled())
		debug->getFile() << "adiw " << regs->getName(Rdl) << ", 0x" << std::hex << (unsigned int)K << std::dec << std::endl;

	regs->pc += 1;
	clock += 2;
}

void
Avr::_sbiw(){
	uint8_t d = (opcode >> 4) & 0x3;
	uint8_t K = (((opcode >> 2) & 0x30) | (opcode & 0xf));
	uint8_t Rdl = (d << 1) + 24;			// Rd = 24 + Rd*2
	uint8_t Rdh = Rdl + 1;					// Rd = 24 + Rd*2 + 1
	uint16_t R = (((*regs)[Rdh] << 8) | ((*regs)[Rdl])) - K;
	int n, v;
	unsigned int Rdh7 = getBit((*regs)[Rdh], 7);
	unsigned int R15 = getBit(R, 15);
	// check for the N flag
	if(R15) {regs->setN(); n=1;}
	else {regs->clearN(); n=0;}
	// check for the V flag
	if(Rdh7 & (!R15)) {regs->setV(); v=1;}
	else {regs->clearV(); v=0;}
	// check for the S flag
	if(n^v) regs->setS();
	else regs->clearS();
	// check for the Z flag
	if(!R) regs->setZ();
	else regs->clearZ();
	// check for the C flag
	if(R15&(!Rdh7)) regs->setC();
	else regs->clearC();
	// write back the result
	(*regs)[Rdl] = (uint8_t)(R);
	(*regs)[Rdh] = (uint8_t)(R >> 8);
	
	// disassemble
	if(debug->isFileOpened() && debug->isDisassemblyEnabled())
		debug->getFile() << "sbiw " << regs->getName(Rdl) << ", 0x" << std::hex << (unsigned int)K << std::dec << std::endl;

	regs->pc += 1;
	clock += 2;
}

void
Avr::_bclr(){
	uint8_t s = (opcode >> 4) & 0x7;
	switch(s){
			case 0:
				std::cout << "clc" << std::endl;
				regs->clearC();
				break;
			case 1:
				debug->getFile() << "clz" << std::endl;
				regs->clearZ();
				break;
			case 2:
				std::cout << "cln" << std::endl;
				regs->clearN();
				break;
			case 3:
				debug->getFile() << "clv" << std::endl;
				regs->clearV();
				break;
			case 4:
				std::cout << "cls" << std::endl;
				regs->clearS();
				break;
			case 5:
				debug->getFile() << "clh" << std::endl;
				regs->clearH();
				break;
			case 6:
				std::cout << "clt" << std::endl;
				regs->clearT();
				break;
			case 7:
				debug->getFile() << "cli" << std::endl;
				regs->clearI();
				break;
	}
	
	regs->pc += 1;
	clock += 1;
}

void
Avr::_bset(){
	uint8_t s = (opcode >> 4) & 0x7;
	switch(s){
			case 0:
				if(debug->isFileOpened() && debug->isDisassemblyEnabled())
					debug->getFile() << "sec" << std::endl;
				regs->setC();
				break;
			case 1:
				if(debug->isFileOpened() && debug->isDisassemblyEnabled())
					debug->getFile() << "sez" << std::endl;
				regs->setZ();
				break;
			case 2:
				if(debug->isFileOpened() && debug->isDisassemblyEnabled())
					debug->getFile() << "sen" << std::endl;
				regs->setN();
				break;
			case 3:
				if(debug->isFileOpened() && debug->isDisassemblyEnabled())
					debug->getFile() << "sev" << std::endl;
				regs->setV();
				break;
			case 4:
				if(debug->isFileOpened() && debug->isDisassemblyEnabled())
					debug->getFile() << "ses" << std::endl;
				regs->setS();
				break;
			case 5:
				if(debug->isFileOpened() && debug->isDisassemblyEnabled())
					debug->getFile() << "seh" << std::endl;
				regs->setH();
				break;
			case 6:
				if(debug->isFileOpened() && debug->isDisassemblyEnabled())
					debug->getFile() << "set" << std::endl;
				regs->setT();
				break;
			case 7:
				if(debug->isFileOpened() && debug->isDisassemblyEnabled())
					debug->getFile() << "sei" << std::endl;
				regs->setI();
				break;
	}
	
	regs->pc += 1;
	clock += 1;
}

void
Avr::_rjmp(){
	int16_t K = opcode & 0xfff;
	if(K & 0x800) 
			K = -1 * (((~K) + 1) & 0xfff);		// 2th complement

	// disassemble
	if(debug->isFileOpened() && debug->isDisassemblyEnabled()){
		if(K<0)
			debug->getFile() << "rjmp .-" << std::hex << (int)(K*-2) << "\t\t; 0x" << (int)((regs->pc + K + 1)*2) << std::dec << std::endl;
		else
			debug->getFile() << "rjmp .+" << std::hex << (int)(K*2) << "\t\t; 0x" << (int)((regs->pc + K + 1)*2) << std::dec << std::endl;
	}
	regs->pc += K + 1;
	clock += 2;
}

void
Avr::_neg(){
	uint8_t Rd = (opcode >> 4) & 0x1f;
	uint8_t R = 0 -(*regs)[Rd];
	int n, v;
	unsigned int Rd3 = getBit((*regs)[Rd], 3);
	unsigned int R3 = getBit(R, 3);
	unsigned int R7 = getBit(R, 7);
	// check for the H flag
	if(R3 | Rd3) regs->setH();
	else regs->clearH();
	// check for the N flag
	if(R7) {regs->setN(); n=1;}
	else {regs->clearN(); n=0;}
	// check for the V flag
	if(R == 0x80) {regs->setV(); v=1;}
	else {regs->clearV(); v=0;}
	// check for the S flag
	if(n^v) regs->setS();
	else regs->clearS();
	// check for the Z flag
	if(!R) regs->setZ();
	else regs->clearZ();
	// check for the C flag
	if(R) regs->setC();
	else regs->clearC();	
	// write back the result	
	(*regs)[Rd] = R;
	
	// disassemble
	if(debug->isFileOpened() && debug->isDisassemblyEnabled())
		debug->getFile() << "neg " << regs->getName(Rd) << std::endl;

	regs->pc += 1;
	clock += 1;
}

void
Avr::_inc(){
	uint8_t Rd = (opcode >> 4) & 0x1f;
	uint8_t R =(*regs)[Rd] + 1;
	int n, v;
	unsigned int R7 = getBit(R, 7);
	// check for the N flag
	if(R7) {regs->setN(); n=1;}
	else {regs->clearN(); n=0;}
	// check for the V flag
	if(R == 0x80) {regs->setV(); v=1;}
	else {regs->clearV(); v=0;}
	// check for the S flag
	if(n^v) regs->setS();
	else regs->clearS();
	// check for the Z flag
	if(!R) regs->setZ();
	else regs->clearZ();
	// write back the result	
	(*regs)[Rd] = R;
	
	// disassemble
	if(debug->isFileOpened() && debug->isDisassemblyEnabled())
		debug->getFile() << "inc " << regs->getName(Rd) << std::endl;

	regs->pc += 1;
	clock += 1;
}

void
Avr::_dec(){
	uint8_t Rd = (opcode >> 4) & 0x1f;
	uint8_t R =(*regs)[Rd] - 1;
	int n, v;
	unsigned int R7 = getBit(R, 7);
	// check for the N flag
	if(R7) {regs->setN(); n=1;}
	else {regs->clearN(); n=0;}
	// check for the V flag
	if(R == 0x7f) {regs->setV(); v=1;}
	else {regs->clearV(); v=0;}
	// check for the S flag
	if(n^v) regs->setS();
	else regs->clearS();
	// check for the Z flag
	if(!R) regs->setZ();
	else regs->clearZ();
	// write back the result	
	(*regs)[Rd] = R;
	
	// disassemble
	if(debug->isFileOpened() && debug->isDisassemblyEnabled())
		debug->getFile() << "dec " << regs->getName(Rd) << std::endl;
	
	regs->pc += 1;
	clock += 1;
}

void
Avr::_brbc(){
	uint8_t s = opcode & 0x7;
	int16_t K = (opcode >> 3) & 0x7f;
	if(K & 0x40) 
			K = -1 * (((~K) + 1) & 0x7f);		// 2th complement

	// disassemble
	if(debug->isFileOpened() && debug->isDisassemblyEnabled()){
		if(K<0)
			debug->getFile() << "brbc " << std::hex << (int)s << ", .-"  << (int)(K*-2) << "\t\t; 0x" << (int)((regs->pc + K + 1)*2) << std::dec << std::endl;
		else
			debug->getFile() << "brbc " << std::hex << (int)s << ", .+"  << (int)(K*2) << "\t\t; 0x" << (int)((regs->pc + K + 1)*2) << std::dec << std::endl;

	}
	if(!getBit((*regs)[REG_SREG], s)){
		regs->pc = regs->pc + 1 + K;
		clock += 2;
	}
	else{
		regs->pc += 1;
		clock += 1;
	}
}

void
Avr::_brbs(){
	uint8_t s = opcode & 0x7;
	int16_t K = (opcode >> 3) & 0x7f;
	if(K & 0x40) 
			K = -1 * (((~K) + 1) & 0x7f);		// 2th complement

	// disassemble
	if(debug->isFileOpened() && debug->isDisassemblyEnabled()){
		if(K<0)
			debug->getFile() << "brbs " << std::hex << (int)s << ", .-"  << (int)(K*-2) << "\t\t; 0x" << (int)((regs->pc + K + 1)*2) << std::dec << std::endl;
		else
			debug->getFile() << "brbs " << std::hex << (int)s << ", .+"  << (int)(K*2) << "\t\t; 0x" << (int)((regs->pc + K + 1)*2) << std::dec << std::endl;
	}
	if(getBit((*regs)[REG_SREG], s)){
		regs->pc = regs->pc + 1 + K;
		clock += 2;
	}
	else{
		regs->pc += 1;
		clock += 1;
	}
}

void
Avr::_com(){
	uint8_t Rd = (opcode >> 4) & 0x1f;
	uint8_t R = 0xff -(*regs)[Rd];
	int n, v;
	unsigned int R7 = getBit(R, 7);
	// check for the N flag
	if(R7) {regs->setN(); n=1;}
	else {regs->clearN(); n=0;}
	// check for the V flag
	regs->clearV();
	v=0;
	// check for the S flag
	if(n^v) regs->setS();
	else regs->clearS();
	// check for the Z flag
	if(!R) regs->setZ();
	else regs->clearZ();
	// check for the C flag
	regs->setC();
	// write back the result	
	(*regs)[Rd] = R;
	
	// disassemble
	if(debug->isFileOpened() && debug->isDisassemblyEnabled())
		debug->getFile() << "com " << regs->getName(Rd) << std::endl;
	
	regs->pc += 1;
	clock += 1;
}

void
Avr::_cp(){
	uint8_t Rr = ((opcode >> 5) & 0x10) | (opcode & 0xf);
	uint8_t Rd = ((opcode >> 4) & 0x1f);
	uint8_t R =(*regs)[Rd] -(*regs)[Rr];
	int n, v;
	unsigned int Rd3 = getBit((*regs)[Rd], 3);
	unsigned int Rr3 = getBit((*regs)[Rr], 3);
	unsigned int R3 = getBit(R, 3);
	unsigned int Rd7 = getBit((*regs)[Rd], 7);
	unsigned int Rr7 = getBit((*regs)[Rr], 7);
	unsigned int R7 = getBit(R, 7);
	// check for the H flag
	if(((!Rd3)&Rr3) | (Rr3&R3) | (R3&(!Rd3))) regs->setH();
	else regs->clearH();
	// check for the N flag
	if(R7) {regs->setN(); n=1;}
	else {regs->clearN(); n=0;}
	// check for the V flag
	if((Rd7&(!Rr7)&(!R7)) | ((!Rd7)&Rr7&R7)){regs->setV(); v=1;}
	else {regs->clearV(); v=0;}
	// check for the S flag
	if(n^v) regs->setS();
	else regs->clearS();
	// check for the Z flag
	if(!R) regs->setZ();
	else regs->clearZ();
	// check for the C flag
	if(((!Rd7)&Rr7) | (Rr7&R7) | (R7&(!Rd7))) regs->setC();
	else regs->clearC();
	
	// disassemble
	if(debug->isFileOpened() && debug->isDisassemblyEnabled())
		debug->getFile() << "cp " << regs->getName(Rd) << ", " << regs->getName(Rr) << std::endl;
	
	regs->pc += 1;
	clock += 1;
}

void
Avr::_cpc(){
	uint8_t Rr = ((opcode >> 5) & 0x10) | (opcode & 0xf);
	uint8_t Rd = ((opcode >> 4) & 0x1f);
	uint8_t R =(*regs)[Rd] -(*regs)[Rr];
	if(regs->isC())
		R -= 1;
	int n, v;
	unsigned int Rd3 = getBit((*regs)[Rd], 3);
	unsigned int Rr3 = getBit((*regs)[Rr], 3);
	unsigned int R3 = getBit(R, 3);
	unsigned int Rd7 = getBit((*regs)[Rd], 7);
	unsigned int Rr7 = getBit((*regs)[Rr], 7);
	unsigned int R7 = getBit(R, 7);
	// check for the H flag
	if(((!Rd3)&Rr3) | (Rr3&R3) | (R3&(!Rd3))) regs->setH();
	else regs->clearH();
	// check for the N flag
	if(R7) {regs->setN(); n=1;}
	else {regs->clearN(); n=0;}
	// check for the V flag
	if((Rd7&(!Rr7)&(!R7)) | ((!Rd7)&Rr7&R7)){regs->setV(); v=1;}
	else {regs->clearV(); v=0;}
	// check for the S flag
	if(n^v) regs->setS();
	else regs->clearS();
	// check for the Z flag
	if((!R)&(regs->isZ())) regs->setZ();
	else regs->clearZ();
	// check for the C flag
	if(((!Rd7)&Rr7) | (Rr7&R7) | (R7&(!Rd7))) regs->setC();
	else regs->clearC();
	
	// disassemble
	if(debug->isFileOpened() && debug->isDisassemblyEnabled())
		debug->getFile() << "cpc " << regs->getName(Rd) << ", " << regs->getName(Rr) << std::endl;
		
	regs->pc += 1;
	clock += 1;
}

void
Avr::_cpi(){
	uint8_t Rd = ((opcode >> 4) & 0xf) + 16;
	uint8_t K = (((opcode & 0xf00) >> 4) | (opcode & 0xf));
	uint8_t R =(*regs)[Rd] - K;
	int n, v;
	unsigned int Rd3 = getBit((*regs)[Rd], 3);
	unsigned int K3 = getBit(K, 3);
	unsigned int R3 = getBit(R, 3);
	unsigned int Rd7 = getBit((*regs)[Rd], 7);
	unsigned int K7 = getBit(K, 7);
	unsigned int R7 = getBit(R, 7);
	// check for the H flag
	if(((!Rd3)&K3) | (K3&R3) | (R3&(!Rd3))) regs->setH();
	else regs->clearH();
	// check for the N flag
	if(R7) {regs->setN(); n=1;}
	else {regs->clearN(); n=0;}
	// check for the V flag
	if((Rd7&(!K7)&(!R7)) | ((!Rd7)&K7&R7)){regs->setV(); v=1;}
	else {regs->clearV(); v=0;}
	// check for the S flag
	if(n^v) regs->setS();
	else regs->clearS();
	// check for the Z flag
	if(!R) regs->setZ();
	else regs->clearZ();
	// check for the C flag
	if(((!Rd7)&K7) | (K7&R7) | (R7&(!Rd7))) regs->setC();
	else regs->clearC();
	
	// disassemble
	if(debug->isFileOpened() && debug->isDisassemblyEnabled())
		debug->getFile() << "cpi " << regs->getName(Rd) << ", 0x" << std::hex << (unsigned int)K << std::dec << std::endl;
	
	regs->pc += 1;
	clock += 1;
}

void
Avr::_lsr(){
	uint8_t Rd = (opcode >> 4) & 0x1f;
	uint8_t R =(*regs)[Rd] >> 1;
	int n=0, c, v;
	unsigned int Rd0 = getBit((*regs)[Rd], 0);
	// check for the N flag
	regs->clearN();
	// check for the C flag
	if(Rd0){regs->setC(); c=1;}
	else{regs->clearC(); c=0;}
	// check for the V flag
	if(n^c){regs->setV(); v=1;}
	else{regs->clearV(); v=0;}
	// check for the S flag
	if(n^v) regs->setS();
	else regs->clearS();
	// check for the Z flag
	if(!R) regs->setZ();
	else regs->clearZ();
	// write back the result	
	(*regs)[Rd] = R;
	
	// disassemble
	if(debug->isFileOpened() && debug->isDisassemblyEnabled())
		debug->getFile() << "lsr " << regs->getName(Rd) << std::endl;
		
	regs->pc += 1;
	clock += 1;
}

void
Avr::_swap(){
	uint8_t Rd = (opcode >> 4) & 0x1f;
	uint8_t R = ((*regs)[Rd] >> 4) | (((*regs)[Rd] & 0xf) << 4);
	// write back the result
	(*regs)[Rd] = R;

	// disassemble
	if(debug->isFileOpened() && debug->isDisassemblyEnabled())
		debug->getFile() << "swap " << regs->getName(Rd) << std::endl;
	
	regs->pc += 1;
	clock += 1;
}

void
Avr::_push(){
	uint8_t Rr = (opcode >> 4) & 0x1f;
	uint16_t SP = (((*regs)[REG_SPH] << 8) |(*regs)[REG_SPL]);
	
	// (*sram)[SP] =(*regs)[Rr];
	sram->write(SP, (*regs)[Rr]);
	
	SP -= 1;
	(*regs)[REG_SPH] = (uint8_t)(SP >> 8);
	(*regs)[REG_SPL] = (uint8_t)(SP);
	
	// disassemble
	if(debug->isFileOpened() && debug->isDisassemblyEnabled())
		debug->getFile() << "push " << regs->getName(Rr) << std::endl;
		
	regs->pc += 1;
	clock += 2;
}

void
Avr::_pop(){
	uint8_t Rr = (opcode >> 4) & 0x1f;
	uint16_t SP = (((*regs)[REG_SPH] << 8) |(*regs)[REG_SPL]);
	
	SP += 1;
	(*regs)[REG_SPH] = (uint8_t)(SP >> 8);
	(*regs)[REG_SPL] = (uint8_t)(SP);
	
	// (*regs)[Rr] = (*sram)[SP];
	(*regs)[Rr] = sram->read(SP);
	
	// disassemble
	if(debug->isFileOpened() && debug->isDisassemblyEnabled())
		debug->getFile() << "pop " << regs->getName(Rr) << std::endl;
		
	regs->pc += 1;
	clock += 2;
}

void
Avr::_out(){
	uint8_t Rr = (opcode >> 4) & 0x1f;
	uint8_t A = (((opcode >> 5) & 0x30) | (opcode & 0xf)) + 0x20;

	// (*sram)[A] = (*regs)[Rr];
	sram->write(A, (*regs)[Rr]);

	// timer->outp(A - 0x20);

	// disassemble
	if(debug->isFileOpened() && debug->isDisassemblyEnabled())
		debug->getFile() << "out 0x" << std::hex << (unsigned int)(A-0x20) << std::dec << ", " << regs->getName(Rr) << std::endl;
	
	regs->pc += 1;
	clock += 1;
}

void
Avr::_in(){
	uint8_t Rd = (opcode >> 4) & 0x1f;
	uint8_t A = (((opcode >> 5) & 0x30) | (opcode & 0xf)) + 0x20;

	// timer->inp(A - 0x20);

	// (*regs)[Rd] = (*sram)[A];
	(*regs)[Rd] = sram->read(A);

	// disassemble
	if(debug->isFileOpened() && debug->isDisassemblyEnabled())
		debug->getFile() << "in " << regs->getName(Rd) << ", 0x" << std::hex << (unsigned int)(A-0x20) << std::dec << std::endl;
	
	regs->pc += 1;
	clock += 1;
}

void
Avr::_rcall(){
	int16_t K = opcode & 0xfff;
	if(K & 0x800) 
			K = -1 * (((~K) + 1) & 0xfff);		// 2th complement

	// (*sram)[regs->getSP()] = (uint8_t)(regs->pc + 1);
	sram->write(regs->getSP(), (uint8_t)(regs->pc + 1));
	// (*sram)[regs->getSP()-1] = (uint8_t)((regs->pc + 1) >> 8);
	sram->write(regs->getSP()-1, (uint8_t)((regs->pc + 1) >> 8));
	
	regs->setSP(regs->getSP()-2);
	
	// disassemble
	if(debug->isFileOpened() && debug->isDisassemblyEnabled()){
		if(K<0)
			debug->getFile() << "rcall .-" << std::hex << (int)(K*-2) << "\t\t; 0x" << (int)((regs->pc + K + 1)*2) << std::dec << std::endl;
		else
			debug->getFile() << "rcall .+" << std::hex << (int)(K*2) << "\t\t; 0x" << (int)((regs->pc + K + 1)*2) << std::dec << std::endl;
	}
	regs->pc += K + 1;
	clock += 3;
}

void
Avr::_ret(){
	regs->setSP(regs->getSP()+2);

	// disassemble
	if(debug->isFileOpened() && debug->isDisassemblyEnabled())
		// debug->getFile() << "ret\t\t" << "; 0x" << ((int)(((*sram)[regs->getSP()-1] << 8) | ((*sram)[regs->getSP()])*2)) << std::endl;
		debug->getFile() << "ret\t\t" << "; 0x" << std::hex << ((int)((sram->read(regs->getSP()-1) << 8) | (sram->read(regs->getSP()))*2)) << std::dec << std::endl;
	
	// regs->pc = ((*sram)[regs->getSP()-1] << 8) | ((*sram)[regs->getSP()]);
	regs->pc = (sram->read(regs->getSP()-1) << 8) | (sram->read(regs->getSP()));
	clock += 4;
}

void
Avr::_ror(){
	uint8_t Rd = (opcode >> 4) & 0x1f;
	uint8_t R =(*regs)[Rd] >> 1;
	if(regs->isC())
		R |= 0x80;
	unsigned int Rd0 = getBit((*regs)[Rd], 0);
	unsigned int R7 = getBit(R, 7);
	unsigned int n, c, v;
	// check for the C flag
	if(Rd0) {regs->setC(); c=1;}
	else {regs->clearC(); c=0;}
	// check for the Z flag
	if(!R) regs->setZ();
	else regs->clearZ();
	// check for the N flag
	if(R7) {regs->setN(); n=1;}
	else {regs->clearN(); n=0;}
	// check for the V flag
	if(n^c) {regs->setV(); v=1;}
	else {regs->clearV(); v=0;}
	// check for the S flag
	if(n^v) regs->setS();
	else regs->clearS();
	// write back the result
	(*regs)[Rd] = R;
	
	// disassemble
	if(debug->isFileOpened() && debug->isDisassemblyEnabled())
		debug->getFile() << "ror " << regs->getName(Rd) << std::endl;
		
	regs->pc += 1;	
	clock += 1;
}

void
Avr::_sts(){
	uint8_t Rr = ((opcode >> 4) & 0xf) + 16;
	uint8_t K = ((opcode >> 4) & 0x7) | (opcode & 0xf);
	// (*sram)[K] =(*regs)[Rr];
	sram->write(K, (*regs)[Rr]);
		
	// disassemble
	if(debug->isFileOpened() && debug->isDisassemblyEnabled())
		debug->getFile() << "sts 0x" << std::hex << (unsigned int)(K) << std::dec << ", " << regs->getName(Rr) << std::endl;

	regs->pc += 1;	
	clock += 1;
}

void
Avr::_sts32(){
	uint8_t Rr = ((opcode >> 4) & 0x1f);
	uint16_t K = this->flash[regs->pc+1];
	if(K >= sram->getSize())
		debug->getFile() << "SRAM: ILLEGAL ADDRESS" << std::endl;
	// (*sram)[K] =(*regs)[Rr];
	sram->write(K, (*regs)[Rr]);

	// disassemble
	if(debug->isFileOpened() && debug->isDisassemblyEnabled())
		debug->getFile() << "sts 0x" << std::hex << (unsigned int)(K) << std::dec << ", " << regs->getName(Rr) << std::endl;
	
	regs->pc += 2;	
	clock += 2;
}

void
Avr::_lds(){
	uint8_t Rd = ((opcode >> 4) & 0xf) + 16;
	uint8_t K = ((opcode >> 4) & 0x7) | (opcode & 0xf);
	// (*regs)[Rd] = (*sram)[K];
	(*regs)[Rd] = sram->read(K);

	// disassemble
	if(debug->isFileOpened() && debug->isDisassemblyEnabled())
		debug->getFile() << "lds " << regs->getName(Rd) << ", 0x" << std::hex << (unsigned int)(K) << std::dec << std::endl;

		
	regs->pc += 1;	
	clock += 1;
}

void
Avr::_lds32(){
	uint8_t Rd = ((opcode >> 4) & 0x1f);
	uint16_t K = this->flash[regs->pc+1];
	if(K >= sram->getSize())
		debug->getFile() << "SRAM: ILLEGAL ADDRESS" << std::endl;
	// (*regs)[Rd] = (*sram)[K];
	(*regs)[Rd] = sram->read(K);

	// disassemble
	if(debug->isFileOpened() && debug->isDisassemblyEnabled())
		debug->getFile() << "lds " << regs->getName(Rd) << ", 0x" << std::hex << (unsigned int)(K) << std::dec << std::endl;

	regs->pc += 2;	
	clock += 2;
}

void
Avr::_stx(){
	uint8_t Rr = ((opcode >> 4) & 0x1f);
	uint16_t X = regs->getX();
	char sign = ' ';			// Only used by disassembler
	if((opcode & MSK_STX2) == OP_STX2){
		// (*sram)[X] = (*regs)[Rr];
		sram->write(X, (*regs)[Rr]);
		++X;
		sign = '+';
	}
	else if((opcode & MSK_STX3) == OP_STX3){
		--X;
		// (*sram)[X] =(*regs)[Rr];
		sram->write(X, (*regs)[Rr]);
		sign = '-';
	}
	else{
		// (*sram)[X] =(*regs)[Rr];
		sram->write(X, (*regs)[Rr]);
	}
	regs->setX(X);
	
	// disassemble
	if(debug->isFileOpened() && debug->isDisassemblyEnabled()){
		if(sign == '+')
			debug->getFile() << "st x+, " << regs->getName(Rr) << std::endl;
		else if(sign == '-')
			debug->getFile() << "st -x, " << regs->getName(Rr) << std::endl;
		else
			debug->getFile() << "st x, " << regs->getName(Rr) << std::endl;
	}
	regs->pc += 1;	
	clock += 2;
}

void
Avr::_sty(){
	uint8_t Rr = ((opcode >> 4) & 0x1f);
	uint16_t Y = regs->getY();
	uint16_t q = 0;
	char sign = ' ';			// Only used by disassembler
	if((opcode & MSK_STY2) == OP_STY2){
		// (*sram)[Y] =(*regs)[Rr];
		sram->write(Y, (*regs)[Rr]);
		++Y;
		sign = '+';
	}
	else if((opcode & MSK_STY3) == OP_STY3){
		--Y;
		// (*sram)[Y] =(*regs)[Rr];
		sram->write(Y, (*regs)[Rr]);
		sign = '-';
	}
	else{
		q = ((opcode >> 8) & 0x20) | ((opcode >> 7) & 0x18) | (opcode & 0x7);
		// (*sram)[Y+q] =(*regs)[Rr];
		sram->write(Y+q, (*regs)[Rr]);
	}
	regs->setY(Y);
	
	// disassemble
	if(debug->isFileOpened() && debug->isDisassemblyEnabled()){
		if(sign == '+')
			debug->getFile() << "st y+, " << regs->getName(Rr) << std::endl;
		else if(sign == '-')
			debug->getFile() << "st -y, " << regs->getName(Rr) << std::endl;
		else
			if(q)
				debug->getFile() << "std y+" << (int)(q) << ", " << regs->getName(Rr) << std::endl;
			else
				debug->getFile() << "st y, " << regs->getName(Rr) << std::endl;
	}
	regs->pc += 1;	
	clock += 2;
}

void
Avr::_stz(){
	uint8_t Rr = ((opcode >> 4) & 0x1f);
	uint16_t Z = regs->getZ();
	uint16_t q = 0;
	char sign = ' ';			// Only used by disassembler
	if((opcode & MSK_STZ2) == OP_STZ2){
		// (*sram)[Z] =(*regs)[Rr];
		sram->write(Z, (*regs)[Rr]);
		++Z;
		sign = '+';
	}
	else if((opcode & MSK_STZ3) == OP_STZ3){
		--Z;
		// (*sram)[Z] =(*regs)[Rr];
		sram->write(Z, (*regs)[Rr]);
		sign = '-';
	}
	else{
		q = ((opcode >> 8) & 0x20) | ((opcode >> 7) & 0x18) | (opcode & 0x7);
		// (*sram)[Z+q] =(*regs)[Rr];
		sram->write(Z+q, (*regs)[Rr]);
	}
	regs->setZ(Z);

	// disassemble
	if(debug->isFileOpened() && debug->isDisassemblyEnabled()){
		if(sign == '+')
			debug->getFile() << "st z+, " << regs->getName(Rr) << std::endl;
		else if(sign == '-')
			debug->getFile() << "st -z, " << regs->getName(Rr) << std::endl;
		else
			if(q)
				debug->getFile() << "std z+" << (int)(q) << ", " << regs->getName(Rr) << std::endl;
			else
				debug->getFile() << "st z, " << regs->getName(Rr) << std::endl;
	}
	regs->pc += 1;	
	clock += 2;
}

void
Avr::_ldx(){
	uint8_t Rd = ((opcode >> 4) & 0x1f);
	uint16_t X = regs->getX();
	char sign = ' ';
	if((opcode & MSK_LDX2) == OP_LDX2){
		// (*regs)[Rd] = (*sram)[X];
		(*regs)[Rd] = sram->read(X);
		++X;
		sign = '+';
	}
	else if((opcode & MSK_LDX3) == OP_LDX3){
		--X;
		// (*regs)[Rd] = (*sram)[X];
		(*regs)[Rd] = sram->read(X);
		sign = '-';
	}
	else{
		// (*regs)[Rd] = (*sram)[X];
		(*regs)[Rd] = sram->read(X);
	}
	regs->setX(X);
	
	// disassemble
	if(debug->isFileOpened() && debug->isDisassemblyEnabled()){
		if(sign == '+')
			debug->getFile() << "ld " << regs->getName(Rd) << ", x+" << std::endl;
		else if(sign == '-')
			debug->getFile() << "ld " << regs->getName(Rd) << ", -x" << std::endl;
		else
			debug->getFile() << "ld " << regs->getName(Rd) << ", x" << std::endl;
	}
	regs->pc += 1;	
	clock += 2;
}

void
Avr::_ldy(){
	uint8_t Rd = ((opcode >> 4) & 0x1f);
	uint16_t Y = regs->getY();
	uint16_t q = 0;
	char sign = ' ';
	if((opcode & MSK_LDY2) == OP_LDY2){
		// (*regs)[Rd] = (*sram)[Y];
		(*regs)[Rd] = sram->read(Y);
		++Y;
		sign = '+';
	}
	else if((opcode & MSK_LDY3) == OP_LDY3){
		--Y;
		// (*regs)[Rd] = (*sram)[Y];
		(*regs)[Rd] = sram->read(Y);
		sign = '-';
	}
	else{
		q = ((opcode >> 8) & 0x20) | ((opcode >> 7) & 0x18) | (opcode & 0x7);
		// (*regs)[Rd] = (*sram)[Y+q];
		(*regs)[Rd] = sram->read(Y+q);
	}
	regs->setY(Y);
	
	// disassemble
	if(debug->isFileOpened() && debug->isDisassemblyEnabled()){
		if(sign == '+')
			debug->getFile() << "ld " << regs->getName(Rd) << ", y+" << std::endl;
		else if(sign == '-')
			debug->getFile() << "ld " << regs->getName(Rd) << ", -y" << std::endl;
		else
			if(q)
				debug->getFile() << "ldd " << regs->getName(Rd) << ", y+" << (int)(q) << std::endl;
			else
				debug->getFile() << "ld " << regs->getName(Rd) << ", y" << std::endl;
	}
	regs->pc += 1;	
	clock += 2;
}

void
Avr::_ldz(){
	uint8_t Rd = ((opcode >> 4) & 0x1f);
	uint16_t Z = regs->getZ();
	uint16_t q = 0;
	char sign = ' ';
	if((opcode & MSK_LDZ2) == OP_LDZ2){
		// (*regs)[Rd] = (*sram)[Z];
		(*regs)[Rd] = sram->read(Z);
		++Z;
		sign = '+';
	}
	else if((opcode & MSK_LDZ3) == OP_LDZ3){
		--Z;
		// (*regs)[Rd] = (*sram)[Z];
		(*regs)[Rd] = sram->read(Z);
		sign = '-';
	}
	else{
		q = ((opcode >> 8) & 0x20) | ((opcode >> 7) & 0x18) | (opcode & 0x7);
		// (*regs)[Rd] = (*sram)[Z+q];
		(*regs)[Rd] = sram->read(Z+q);
	}
	regs->setZ(Z);

	// disassemble
	if(debug->isFileOpened() && debug->isDisassemblyEnabled()){
		if(sign == '+')
			debug->getFile() << "ld " << regs->getName(Rd) << ", z+" << std::endl;
		else if(sign == '-')
			debug->getFile() << "ld " << regs->getName(Rd) << ", -z" << std::endl;
		else
			if(q)
				debug->getFile() << "ldd " << regs->getName(Rd) << ", z+" << (int)(q) << std::endl;
			else
				debug->getFile() << "ld " << regs->getName(Rd) << ", z" << std::endl;
	}
	regs->pc += 1;	
	clock += 2;
}

void
Avr::_break(){
	stopped = true;
	// disassemble
	if(debug->isFileOpened() && debug->isDisassemblyEnabled())
		debug->getFile() << "break" << std::endl;

	regs->pc += 1;	
	clock += 1;
}

void
Avr::_asr(){
	uint8_t Rd = (opcode >> 4) & 0x1f;
	uint8_t R =(*regs)[Rd] >> 1;
	if(getBit((*regs)[Rd], 7))
		R |= 0x80;
	unsigned int Rd0 = getBit((*regs)[Rd], 0);
	unsigned int R7 = getBit(R, 7);
	unsigned int n, c, v;
	// check for the C flag
	if(Rd0) {regs->setC(); c=1;}
	else {regs->clearC(); c=0;}
	// check for the Z flag
	if(!R) regs->setZ();
	else regs->clearZ();
	// check for the N flag
	if(R7) {regs->setN(); n=1;}
	else {regs->clearN(); n=0;}
	// check for the V flag
	if(n^c) {regs->setV(); v=1;}
	else {regs->clearV(); v=0;}
	// check for the S flag
	if(n^v) regs->setS();
	else regs->clearS();
	// write back the result
	(*regs)[Rd] = R;

	// disassemble
	if(debug->isFileOpened() && debug->isDisassemblyEnabled())
		debug->getFile() << "asr " << regs->getName(Rd) << std::endl;

	regs->pc += 1;	
	clock += 1;
}

void
Avr::_bld(){
	uint8_t Rd = (opcode >> 4) & 0x1f;
	uint8_t b = opcode & 0x7;
	uint8_t R =(*regs)[Rd];
	if(regs->isT())
		setBit(R, (int)b);
	else
		clearBit(R, (int)b);
	// write back the result
	(*regs)[Rd] = R;

	// disassemble
	if(debug->isFileOpened() && debug->isDisassemblyEnabled())
		debug->getFile() << "bld " << regs->getName(Rd) << ", " << (int)(b) << std::endl;

	regs->pc += 1;
	clock += 1;
}

void
Avr::_bst(){
	uint8_t Rd = (opcode >> 4) & 0x1f;
	uint8_t b = opcode & 0x7;
	if(getBit((*regs)[Rd], (int)b))
		regs->setT();
	else
		regs->clearT();

	// disassemble
	if(debug->isFileOpened() && debug->isDisassemblyEnabled())
		debug->getFile() << "bst " << regs->getName(Rd) << ", " << (int)(b) << std::endl;
	
	regs->pc += 1;
	clock += 1;
}

void
Avr::_cbi(){
	uint8_t A = ((opcode >> 3) & 0x1f) + 0x20;
	uint8_t b = opcode & 0x7;
	// clearBit((*sram)[A], b);
	sram->clearBit(A, b);

	// disassemble
	if(debug->isFileOpened() && debug->isDisassemblyEnabled())
		debug->getFile() << "cbi 0x" << std::hex << (int)(A-0x20) << std::dec << ", " << (int)(b) << std::endl;

	regs->pc += 1;
	clock += 2;
}

void
Avr::_sbi(){
	uint8_t A = ((opcode >> 3) & 0x1f) + 0x20;
	uint8_t b = opcode & 0x7;
	//setBit((*sram)[A], b);
	sram->setBit(A, b);

	// disassemble
	if(debug->isFileOpened() && debug->isDisassemblyEnabled())
		debug->getFile() << "sbi 0x" << std::hex << (int)(A-0x20) << std::dec << ", " << (int)(b) << std::endl;

	regs->pc += 1;
	clock += 2;
}

void
Avr::_cpse(){
	uint8_t Rd = (opcode >> 4) & 0x1f;
	uint8_t Rr = ((opcode >> 5) & 0x10) | (opcode & 0xf);
	
	// disassemble
	if(debug->isFileOpened() && debug->isDisassemblyEnabled())
		debug->getFile() << "cpse " << regs->getName(Rd) << ", " << regs->getName(Rr) << std::endl;

	if((*regs)[Rd] ==(*regs)[Rr]){
		uint16_t op2 = this->flash[regs->pc+1];
		unsigned int c = 2;
		if((op2 & MSK_CALL) == OP_CALL) ++c;
		else if((op2 & MSK_JMP) == OP_JMP) ++c;
		else if((op2 & MSK_LDS32) == OP_LDS32) ++c;
		else if((op2 & MSK_STS32) == OP_STS32) ++c;
		regs->pc += c;
		clock += c;
	}else{
		regs->pc += 1;
		clock += 1;
	}
}

void
Avr::_ijmp(){
	// disassemble
	if(debug->isFileOpened() && debug->isDisassemblyEnabled())
		debug->getFile() << "ijmp" << std::endl;

	regs->pc = regs->getZ();
	clock += 2;
}

void
Avr::_icall(){
	// (*sram)[regs->getSP()] = (uint8_t)(regs->pc + 1);
	sram->write(regs->getSP(), (uint8_t)(regs->pc + 1));
	// (*sram)[regs->getSP()-1] = (uint8_t)((regs->pc + 1) >> 8);
	sram->write(regs->getSP()-1, (uint8_t)((regs->pc + 1) >> 8));
	
	regs->setSP(regs->getSP()-2);

	// disassemble
	if(debug->isFileOpened() && debug->isDisassemblyEnabled())
		debug->getFile() << "icall" << std::endl;
	
	regs->pc = regs->getZ();
	clock += 3;
}

void
Avr::_jmp(){
	/**
	 * Only 16-bit address are supported!!!
	 * ((opcode >> 3) & 0x3e) | (opcode & 0x1) not used
	 **/
	uint16_t K = this->flash[regs->pc+1];

	// disassemble
	if(debug->isFileOpened() && debug->isDisassemblyEnabled())
		debug->getFile() << "jmp 0x" << std::hex << (unsigned int)(K*2) << std::dec << std::endl;

	regs->pc = K;
	clock += 3;
}

void
Avr::_call(){
	/**
	 * Only 16-bit address are supported!!!
	 * ((opcode >> 3) & 0x3e) | (opcode & 0x1) not used
	 **/
	uint16_t K = this->flash[regs->pc+1];

	// (*sram)[regs->getSP()] = (uint8_t)(regs->pc + 2);
	sram->write(regs->getSP(), (uint8_t)(regs->pc + 2));
	// (*sram)[regs->getSP()-1] = (uint8_t)((regs->pc + 2) >> 8);
	sram->write(regs->getSP()-1, (uint8_t)((regs->pc + 2) >> 8));
	
	regs->setSP(regs->getSP()-2);

	// disassemble
	if(debug->isFileOpened() && debug->isDisassemblyEnabled())
		debug->getFile() << "call 0x" << std::hex << (unsigned int)(K*2) << std::dec << std::endl;
	
	regs->pc = K;
	clock += 4;
}

void
Avr::_reti(){
	regs->setSP(regs->getSP()+2);
	regs->setI();

	// disassemble
	if(debug->isFileOpened() && debug->isDisassemblyEnabled())
		// debug->getFile() << "reti\t\t" << "; 0x" << ((unsigned int)(((*sram)[regs->getSP()-1] << 8) | ((*sram)[regs->getSP()])*2)) << std::endl;
		debug->getFile() << "reti\t\t" << "; 0x" << ((unsigned int)((sram->read(regs->getSP()-1) << 8) | (sram->read(regs->getSP()))*2)) << std::endl;

	// regs->pc = ((*sram)[regs->getSP()-1] << 8) | ((*sram)[regs->getSP()]);
	regs->pc = (sram->read(regs->getSP()-1) << 8) | (sram->read(regs->getSP()));		
	clock += 4;
}

void
Avr::_mul(){
	uint8_t Rr = ((opcode >> 5) & 0x10) | (opcode & 0xf);
	uint8_t Rd = ((opcode >> 4) & 0x1f);
	uint16_t R =(*regs)[Rd] *(*regs)[Rr];
	// check for the C flag
	unsigned int R15 = getBit(R, 15);
	if(R15) regs->setC();
	else regs->clearC();
	// check for the Z flag
	if(!R) regs->setZ();
	else regs->clearZ();
	// write back the result
	(*regs)[REG_R0] = (uint8_t)(R);
	(*regs)[REG_R1] = (uint8_t)(R >> 8);
	
	// disassemble
	if(debug->isFileOpened() && debug->isDisassemblyEnabled())
		debug->getFile() << "mul " << regs->getName(Rd) << ", " << regs->getName(Rr) << std::endl;

	regs->pc += 1;
	clock += 2;
}

void
Avr::_muls(){
	uint8_t r = (opcode & 0xf) + 16;
	uint8_t d = ((opcode >> 4) & 0xf) + 16;
	int8_t Rr =(*regs)[r];
	int8_t Rd =(*regs)[d];
	int16_t extRd = (int16_t)Rd, extRr = (int16_t)Rr;
	int16_t R = (uint16_t)(extRd * extRr);
	// check for the C flag
	unsigned int R15 = getBit((uint16_t)R, 15);
	if(R15) regs->setC();
	else regs->clearC();
	// check for the Z flag
	if(!R) regs->setZ();
	else regs->clearZ();
	// write back the result
	(*regs)[REG_R0] = (uint8_t)(R);
	(*regs)[REG_R1] = (uint8_t)(R >> 8);

	// disassemble
	if(debug->isFileOpened() && debug->isDisassemblyEnabled())
		debug->getFile() << "muls " << regs->getName(d) << ", " << regs->getName(r) << std::endl;

	regs->pc += 1;
	clock += 2;
}

void
Avr::_mulsu(){
	uint8_t r = (opcode & 0x7) + 16;
	uint8_t d = ((opcode >> 4) & 0x7) + 16;
	int8_t Rr =(*regs)[r];
	int8_t Rd =(*regs)[d];
	int16_t extRd = (int16_t)Rd, extRr = (int16_t)Rr;
	if(extRr & 0x8000)
		extRr &= 0x00ff;
	int16_t R = (uint16_t)(extRd * extRr);
	// check for the C flag
	unsigned int R15 = getBit((uint16_t)R, 15);
	if(R15) regs->setC();
	else regs->clearC();
	// check for the Z flag
	if(!R) regs->setZ();
	else regs->clearZ();
	// write back the result
	(*regs)[REG_R0] = (uint8_t)(R);
	(*regs)[REG_R1] = (uint8_t)(R >> 8);

	// disassemble
	if(debug->isFileOpened() && debug->isDisassemblyEnabled())
		debug->getFile() << "mulsu " << regs->getName(d) << ", " << regs->getName(r) << std::endl;

	regs->pc += 1;
	clock += 2;
}

void
Avr::_fmul(){
	uint8_t r = (opcode & 0x7) + 16;
	uint8_t d = ((opcode >> 4) & 0x7) + 16;
	uint8_t Rr =(*regs)[r];
	uint8_t Rd =(*regs)[d];
	uint16_t R = Rd * Rr;
	// check for the C flag
	unsigned int R15 = getBit(R, 15);
	if(R15) regs->setC();
	else regs->clearC();
	// check for the Z flag
	if(!R) regs->setZ();
	else regs->clearZ();
	// write back the result
	R <<= 1;
	(*regs)[REG_R0] = (uint8_t)(R);
	(*regs)[REG_R1] = (uint8_t)(R >> 8);
	
	// disassemble
	if(debug->isFileOpened() && debug->isDisassemblyEnabled())
		debug->getFile() << "fmul " << regs->getName(d) << ", " << regs->getName(r) << std::endl;

	regs->pc += 1;
	clock += 2;
}

void
Avr::_fmuls(){
	uint8_t r = (opcode & 0x7) + 16;
	uint8_t d = ((opcode >> 4) & 0x7) + 16;
	int8_t Rr =(*regs)[r];
	int8_t Rd =(*regs)[d];
	int16_t extRd = (int16_t)Rd, extRr = (int16_t)Rr;
	int16_t R = (uint16_t)(extRd * extRr);
	// check for the C flag
	unsigned int R15 = getBit((uint16_t)R, 15);
	if(R15) regs->setC();
	else regs->clearC();
	// check for the Z flag
	if(!R) regs->setZ();
	else regs->clearZ();
	// write back the result
	R <<= 1;
	(*regs)[REG_R0] = (uint8_t)(R);
	(*regs)[REG_R1] = (uint8_t)(R >> 8);

	// disassemble
	if(debug->isFileOpened() && debug->isDisassemblyEnabled())
		debug->getFile() << "fmuls " << regs->getName(d) << ", " << regs->getName(r) << std::endl;

	regs->pc += 1;
	clock += 2;
}

void
Avr::_fmulsu(){
	uint8_t r = (opcode & 0x7) + 16;
	uint8_t d = ((opcode >> 4) & 0x7) + 16;
	int8_t Rr =(*regs)[r];
	int8_t Rd =(*regs)[d];
	int16_t extRd = (int16_t)Rd, extRr = (int16_t)Rr;
	if(extRr & 0x8000)
		extRr &= 0x00ff;
	int16_t R = (uint16_t)(extRd * extRr);
	// check for the C flag
	unsigned int R15 = getBit((uint16_t)R, 15);
	if(R15) regs->setC();
	else regs->clearC();
	// check for the Z flag
	if(!R) regs->setZ();
	else regs->clearZ();
	// write back the result
	R <<= 1;
	(*regs)[REG_R0] = (uint8_t)(R);
	(*regs)[REG_R1] = (uint8_t)(R >> 8);

	// disassemble
	if(debug->isFileOpened() && debug->isDisassemblyEnabled())
		debug->getFile() << "fmulsu " << regs->getName(d) << ", " << regs->getName(r) << std::endl;

	regs->pc += 1;
	clock += 2;
}

void
Avr::_movw(){
	uint8_t Rr = (opcode & 0xf) << 1;
	uint8_t Rd = ((opcode >> 4) & 0xf) << 1;
	(*regs)[Rd] =(*regs)[Rr];
	(*regs)[Rd+1] =(*regs)[Rr+1];

	// disassemble
	if(debug->isFileOpened() && debug->isDisassemblyEnabled())
		debug->getFile() << "movw " << regs->getName(Rd) << ", " << regs->getName(Rr) << std::endl;
	
	regs->pc += 1;
	clock += 1;	
}

void
Avr::_sbrc(){
	uint8_t Rr = (opcode >> 4) & 0x1f;
	uint8_t b = opcode & 0x7;

	// disassemble
	if(debug->isFileOpened() && debug->isDisassemblyEnabled())
		debug->getFile() << "sbrc " << regs->getName(Rr) << ", " << (int)(b) << std::endl;

	if(!getBit((*regs)[Rr], b)){
		uint16_t op2 = this->flash[regs->pc+1];
		unsigned int c = 2;
		if((op2 & MSK_CALL) == OP_CALL) ++c;
		else if((op2 & MSK_JMP) == OP_JMP) ++c;
		else if((op2 & MSK_LDS32) == OP_LDS32) ++c;
		else if((op2 & MSK_STS32) == OP_STS32) ++c;
		regs->pc += c;
		clock += c;
	}
	else{
		regs->pc += 1;
		clock += 1;
	}
}

void
Avr::_sbrs(){
	uint8_t Rr = (opcode >> 4) & 0x1f;
	uint8_t b = opcode & 0x7;

	// disassemble
	if(debug->isFileOpened() && debug->isDisassemblyEnabled())
		debug->getFile() << "sbrs " << regs->getName(Rr) << ", " << (int)(b) << std::endl;

	if(getBit((*regs)[Rr], b)){
		uint16_t op2 = this->flash[regs->pc+1];
		unsigned int c = 2;
		if((op2 & MSK_CALL) == OP_CALL) ++c;
		else if((op2 & MSK_JMP) == OP_JMP) ++c;
		else if((op2 & MSK_LDS32) == OP_LDS32) ++c;
		else if((op2 & MSK_STS32) == OP_STS32) ++c;
		regs->pc += c;
		clock += c;
	}
	else{
		regs->pc += 1;
		clock += 1;
	}
}

void
Avr::_sleep(){
	this->sleeping = true;

	if(debug->isFileOpened() && debug->isDisassemblyEnabled())
		debug->getFile() << "sleep" << std::endl;
	//std::cerr << "sleeping ..... ZZZzzzz" << std::endl;

	regs->pc += 1;
	clock += 1;
}

void
Avr::_wdr(){
	this->watchdog_timer = 0;

	if(debug->isFileOpened() && debug->isDisassemblyEnabled())
		debug->getFile() << "wdr" << std::endl;

	regs->pc += 1;
	clock += 1;
}

void
Avr::_sbci(){
	uint8_t Rd = ((opcode >> 4) & 0xf) + 16;
	uint8_t K = (((opcode >> 4) & 0xf0) | (opcode & 0xf));
	uint8_t R =(*regs)[Rd] - K;
	if(regs->isC())
		R -= 1;
	int n, v;
	unsigned int Rd3 = getBit((*regs)[Rd], 3);
	unsigned int R3 = getBit(R, 3);
	unsigned int K3 = getBit(K, 3);
	unsigned int Rd7 = getBit((*regs)[Rd], 7);
	unsigned int R7 = getBit(R, 7);
	unsigned int K7 = getBit(K, 7);
	// check for the H flag
	if(((!Rd3)&K3) | (K3&R3) | (R3&(!Rd3))) regs->setH();
	else regs->clearH();
	// check for the N flag
	if(R7) {regs->setN(); n=1;}
	else {regs->clearN(); n=0;}
	// check for the V flag
	if((Rd7&(!K7)&(!R7)) | ((!Rd7)&K7&R7)) {regs->setV(); v=1;}
	else {regs->clearV(); v=0;}
	// check for the S flag
	if(n^v) regs->setS();
	else regs->clearS();
	// check for the Z flag
	if(R) regs->clearZ();
	// check for the C flag
	if(((!Rd7)&K7) | (K7&R7) | (R7&(!Rd7))) regs->setC();
	else regs->clearC();
	// write back the result
	(*regs)[Rd] = R;

	// disassemble
	if(debug->isFileOpened() && debug->isDisassemblyEnabled())
		debug->getFile() << "sbci " << regs->getName(Rd) << ", 0x" << std::hex << (int)(K) << std::dec << std::endl;
	
	regs->pc += 1;
	clock += 1;
}

void
Avr::_sbc(){
	uint8_t Rr = (((opcode >> 5) & 0x10) | (opcode & 0xf));
	uint8_t Rd = ((opcode >> 4) & 0x1f);
	uint8_t R =(*regs)[Rd] -(*regs)[Rr];
	if(regs->isC())
		R -= 1;
	int n, v;
	unsigned int Rr3 = getBit((*regs)[Rr], 3);
	unsigned int Rd3 = getBit((*regs)[Rd], 3);
	unsigned int R3 = getBit(R, 3);
	unsigned int Rr7 = getBit((*regs)[Rr], 7);
	unsigned int Rd7 = getBit((*regs)[Rd], 7);
	unsigned int R7 = getBit(R, 7);
	// check for the H flag
	if(((!Rd3)&Rr3) | (Rr3&R3) | (R3&(!Rd3))) regs->setH();
	else regs->clearH();
	// check for the N flag
	if(R7) {regs->setN(); n=1;}
	else {regs->clearN(); n=0;}
	// check for the V flag
	if((Rd7&(!Rr7)&(!R7)) | ((!Rd7)&Rr7&R7)) {regs->setV(); v=1;}
	else {regs->clearV(); v=0;}
	// check for the S flag
	if(n^v) regs->setS();
	else regs->clearS();
	// check for the Z flag
	if(R) regs->clearZ();
	// check for the C flag
	if(((!Rd7)&Rr7) | (Rr7&R7) | (R7&(!Rd7))) regs->setC();
	else regs->clearC();
	// write back the result
	(*regs)[Rd] = R;

	// disassemble
	if(debug->isFileOpened() && debug->isDisassemblyEnabled())
		debug->getFile() << "sbc " << regs->getName(Rd) << ", " << regs->getName(Rr) << std::endl;
	
	regs->pc += 1;
	clock += 1;
}

void
Avr::_sbic(){
	uint8_t A = ((opcode >> 3) & 0x1f) + 0x20;
	uint8_t b = opcode & 0x7;

	// disassemble
	if(debug->isFileOpened() && debug->isDisassemblyEnabled())
		debug->getFile() << "sbic 0x" << (int)(A-0x20) << ", " << (int)(b) << std::endl;

	// if(!getBit((*sram)[A], b)){
	if(!sram->getBit(A, b)){
		uint16_t op2 = this->flash[regs->pc+1];
		unsigned int c = 2;
		if((op2 & MSK_CALL) == OP_CALL) ++c;
		else if((op2 & MSK_JMP) == OP_JMP) ++c;
		else if((op2 & MSK_LDS32) == OP_LDS32) ++c;
		else if((op2 & MSK_STS32) == OP_STS32) ++c;
		regs->pc += c;
		clock += c;
	}
	else{
		regs->pc += 1;
		clock += 1;
	}
}

void
Avr::_sbis(){
	uint8_t A = ((opcode >> 3) & 0x1f) + 0x20;
	uint8_t b = opcode & 0x7;

	// disassemble
	if(debug->isFileOpened() && debug->isDisassemblyEnabled())
		debug->getFile() << "sbis 0x" << (int)(A-0x20) << ", " << (int)(b) << std::endl;

	// if(getBit((*sram)[A], b)){
	if(sram->getBit(A, b)){
		uint16_t op2 = this->flash[regs->pc+1];
		unsigned int c = 2;
		if((op2 & MSK_CALL) == OP_CALL) ++c;
		else if((op2 & MSK_JMP) == OP_JMP) ++c;
		else if((op2 & MSK_LDS32) == OP_LDS32) ++c;
		else if((op2 & MSK_STS32) == OP_STS32) ++c;
		regs->pc += c;
		clock += c;
	}
	else{
		regs->pc += 1;
		clock += 1;
	}
}

void
Avr::_lpm(){
	uint16_t Z = regs->getZ() >> 1;
	uint16_t word = this->flash[Z];
	uint8_t byte;
	std::string param;
	// High or Low byte ?
	if(regs->getZ() & 0x1)	// High
		byte = word >> 8;
	else			// Low
		byte = word & 0xff;
	if((this->opcode & MSK_LPM1) == OP_LPM1){
		(*regs)[REG_R0] = byte;
		param = "";
	}
	else if((this->opcode & MSK_LPM2) == OP_LPM2){
		uint8_t Rd = (opcode >> 4) & 0x1f;
		(*regs)[Rd] = byte;
		param = " " + regs->getName(Rd) + ", z";
	}
	else if((this->opcode & MSK_LPM3) == OP_LPM3){
		uint8_t Rd = (opcode >> 4) & 0x1f;
		(*regs)[Rd] = byte;
		regs->setZ(regs->getZ()+1);
		param = " " + regs->getName(Rd) + ", z+";
	}

	// disassemble
	if(debug->isFileOpened() && debug->isDisassemblyEnabled())
		debug->getFile() << "lpm" + param << std::endl;

	regs->pc += 1;
	clock += 3;
}

void
Avr::_elpm(){
	uint32_t ZZ = ((*regs)[REG_RAMPZ] << 16) | regs->getZ();
	uint32_t ZZZ = ZZ >> 1;
	uint16_t word = this->flash[ZZZ];
	uint8_t byte;
	std::string param;
	// High or Low byte ?
	if(regs->getZ() & 0x1)	// High
		byte = word >> 8;
	else				// Low
		byte = word & 0xff;
	if((this->opcode & MSK_ELPM1) == OP_ELPM1){
		(*regs)[REG_R0] = byte;
		param = "";
	}
	else if((this->opcode & MSK_ELPM2) == OP_ELPM2){
		uint8_t Rd = (opcode >> 4) & 0x1f;
		(*regs)[Rd] = byte;
		param = " " + regs->getName(Rd) + ", z";
	}
	else if((this->opcode & MSK_ELPM3) == OP_ELPM3){
		uint8_t Rd = (opcode >> 4) & 0x1f;
		(*regs)[Rd] = byte;
		++ZZ;
		regs->setZ((uint16_t)ZZ);
		(*regs)[REG_RAMPZ] = (uint8_t)(ZZ >> 16);
		param = " " + regs->getName(Rd) + ", z+";
	}

	// disassemble
	if(debug->isFileOpened() && debug->isDisassemblyEnabled())
		debug->getFile() << "elpm" + param << std::endl;

	regs->pc += 1;
	clock += 3;
}

void
Avr::dumpRegs()
{
	regs->dump();	
}


void
Avr::addInternalDevice(InternalDevice *d)
{
	this->intrn_devices.push_back(d);
}

void
Avr::addIOListener(uint8_t ioport, InternalDevice *d)
{
	sram->addIOListener(ioport, d);
}

void
Avr::addClockEvent(Event *e)
{
	clock.addEvent(e);
}

void
Avr::awake()
{
	if(sleeping){
		sleeping = false;
		clock += 4;
	}
}

void
Avr::fireInterrupt(uint8_t intr)
{
	//std::cerr << "Interrupt TRY [" << (int)intr << "] @ " << clock.getCycles() << " PC=" << std::hex << regs->pc*2 << std::dec << std::endl;
	//interrupts.push(intr);
	//if(intr == 17 && regs->isI())
	//	std::cerr << "Interrupt [" << (int)intr << "] @ " << clock.getCycles() << " PC=" <<  std::hex << regs->pc*2 << std::dec << std::endl;
	if(regs->isI()){
	//	std::cerr << "Interrupt [" << (int)intr << "] @ " << clock.getCycles() << " PC=" <<  std::hex << regs->pc*2 << std::dec << std::endl;
		awake();
		interrupts.push(intr);
	}
}

void
Avr::interrupt(uint8_t intr)
{
	// (*sram)[regs->getSP()] = (uint8_t)(regs->pc);
	sram->write(regs->getSP(), (uint8_t)(regs->pc));
	// (*sram)[regs->getSP()-1] = (uint8_t)((regs->pc) >> 8);
	sram->write(regs->getSP()-1, (uint8_t)((regs->pc) >> 8));
	
	regs->setSP(regs->getSP()-2);
	regs->clearI();

	regs->pc = intr * 2;
	regs->oldpc = regs->pc;
	clock += 3;		/* for compatibility with AVR Studio */
}

void
Avr::setDebugger(Debugger *debug)
{
	this->debug = debug;
}

uint16_t
Avr::getPC(){
	return regs->pc;
}

