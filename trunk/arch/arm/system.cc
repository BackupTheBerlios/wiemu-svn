#include <cstdlib>
#include <iostream>
#include <iomanip>
#include <cstring>
#include "system.hh"

System::System(){
#ifdef DEBUG
	std::cout << "System()" << std::endl;
#endif
	fw.setFileName("test/flash.img");
	std::cout << "Flash Size = " << (fw.getSize()/1024) << " KB" << std::endl;
	if(!fw.loadImage()){
		std::cerr << "Error: Loading Firmware Image" << std::endl;
		exit(0);
	}
	const char *m = fw.getFirm();
	// Initialize RAM
	initMem(0x2000000);			// 32 MB RAM
	std::cout << "RAM Size = " << (this->msize/1024) << " KB" << std::endl;
	bcopy(m, this->mem, fw.getSize());
}

System::~System(){
#ifdef DEBUG
	std::cout << "~System()" << std::endl;
#endif
}

void
System::initMem(unsigned int size){
		this->msize = size * sizeof(char);
		this->mem = (char *)malloc(msize);
}

void toBitArray(uint32_t word, uint8_t* bits){
	unsigned int bitmask = 1;
	for(int i=0 ; i<32 ; i++){
		if(word & bitmask)
			bits[i] = 1;
		else
			bits[i] = 0;
		bitmask <<= 1;
	}
}

bool System::checkNegative(char opr, uint32_t x, uint32_t y){
	uint32_t result = 0;
	if(opr == '+'){
		result = x + y;
		if(result & (1 << 31))
			return true;
	}
	return false;
}

bool System::checkZero(char opr, uint32_t x, uint32_t y){
	uint32_t result = 0;
	if(opr == '+'){
		result = x + y;
		if(result == 0)
			return true;
	}
	return false;
}

bool System::checkCarry(char opr, uint32_t x, uint32_t y){
	uint64_t result = 0;
	if(opr == '+'){
		result = (uint64_t)x + (uint64_t)y;
		if(result >> 32)
			return true;
	}
	return false;
}

bool System::checkOverflow(char opr, uint32_t x, uint32_t y){
	uint32_t result = 0;
	if(opr == '+'){
		if ((x >> 31) == (y >> 31)){
			result = x + y;
			if((result >> 31) != (y >> 31))
				return true;
		}
	}
	return false;
}

void System::step(void){
/*
	// Big Endian
	uint8_t byte1 = mem[regs.r[REG_R15]+0];
	uint8_t byte2 = mem[regs.r[REG_R15]+1];
	uint8_t byte3 = mem[regs.r[REG_R15]+2];
	uint8_t byte4 = mem[regs.r[REG_R15]+3];
*/
	// Little Endian
	uint8_t byte1 = mem[regs.r[REG_R15]+3];
	uint8_t byte2 = mem[regs.r[REG_R15]+2];
	uint8_t byte3 = mem[regs.r[REG_R15]+1];
	uint8_t byte4 = mem[regs.r[REG_R15]+0];
	
	regs.r[REG_R15]+= 4;
	uint32_t word = (byte1 << 24) | (byte2 << 16) | (byte3 << 8) | byte4;
	uint8_t b[32];
	toBitArray(word, b);

#ifdef DEBUG
	// Print the instruction word
	std::cout << "Instruction: 0x" << std::hex << word << ", ";
	for(int i=31 ; i>=0 ; i--)
		std::cout << (uint8_t)(b[i] + '0');
	std::cout << "b" << std::endl;
#endif

	// Grab the condition 4-bits
	// unsigned char cond = (byte1 & 0xf0) >> 4;

	if(!b[27] && !b[26] && !b[25] && !b[24] &&
		!b[23] && !b[22] && b[7] && !b[6] &&
		!b[5] && b[4]){
#ifdef DEBUG
		std::cout << "CAT#2: Multiply" << std::endl;
#endif
	}
	else if(!b[27] && !b[26] && !b[25] && !b[24] &&
		b[23] && b[7] && !b[6] && !b[5]
		&& b[4]){
#ifdef DEBUG
		std::cout << "CAT#3: Multiply Long" << std::endl;
#endif
	}
	else if(!b[27] && !b[26] && !b[25] && b[24] &&
		!b[23] && !b[21] && !b[20] && !b[11] &&
		!b[10] && !b[9] && !b[8] && b[7] && 
		!b[6] && !b[5] && b[4]){
#ifdef DEBUG
		std::cout << "CAT#4: Single Data Swap" << std::endl;
#endif
	}
	else if(!b[27] && !b[26] && !b[25] && b[24] &&
		!b[23] && !b[22] && b[21] && !b[20] &&
		b[19] && b[18] && b[17] && b[16] && 
		b[15] && b[14] && b[13] && b[12] &&
		b[11] && b[10] && b[9] && b[8] &&
		!b[7] && !b[6] && !b[5] && b[4]){
#ifdef DEBUG
		std::cout << "CAT#5: Branch and Exchange" << std::endl;
#endif
	}
	else if(!b[27] && !b[26] && !b[25] && !b[22] &&
		!b[11] && !b[10] && !b[9] && !b[8] &&
		b[7] && b[4]){
#ifdef DEBUG
		std::cout << "CAT#6: Halfword Data Transfer Register Offset" << std::endl;
#endif
	}
	else if(!b[27] && !b[26] && !b[25] && b[22] &&
		b[7] && b[4]){
#ifdef DEBUG
		std::cout << "CAT#7: Halfword Data Transfer Immediate Offset" << std::endl;
#endif
	}
	else if(!b[27] && b[26]){
#ifdef DEBUG
		std::cout << "CAT#8: Single Data Transfer" << std::endl;
#endif
	}
	else if(!b[27] && b[26] && b[25] && b[4]){
#ifdef DEBUG
		std::cout << "CAT#9: Undefined" << std::endl;
#endif
	}
	else if(b[27] && !b[26] && !b[25]){
#ifdef DEBUG
		std::cout << "CAT#10: Block Data Transfer" << std::endl;
#endif
	}
	else if(b[27] && !b[26] && b[25]){
#ifdef DEBUG
		std::cout << "CAT#11: Branch" << std::endl;
#endif
	}
	else if(b[27] && b[26] && !b[25]){
#ifdef DEBUG
		std::cout << "CAT#12: Coprocessor Data Transfer" << std::endl;
#endif
	}
	else if(b[27] && b[26] && b[25] && !b[24] && !b[4]){
#ifdef DEBUG
		std::cout << "CAT#13: Coprocessor Data Operation" << std::endl;
#endif
	}
	else if(b[27] && b[26] && b[25] && !b[24] && b[4]){
#ifdef DEBUG
		std::cout << "CAT#14: Coprocessor Register Transfer" << std::endl;
#endif
	}
	else if(b[27] && b[26] && b[25] && b[24]){
#ifdef DEBUG
		std::cout << "CAT#15: Software Interrupt" << std::endl;
#endif
	}
	// Data Processing
	else if(!b[27] && !b[26]){
#ifdef DEBUG
		std::cout << "CAT #1: Data Processing" << std::endl;
#endif
		cat1(b);
	}
#ifdef DEBUG
	regs.dump();
#endif
}

void System::cat1(uint8_t *b){
		uint8_t opcode = (b[24] << 3) | (b[23] << 2) | (b[22] << 1) | b[21];
		switch(opcode){
			case OP_AND:
#ifdef DEBUG
				std::cout << "OPCODE: AND" << std::endl;
#endif
				break;
			case OP_EOR:
#ifdef DEBUG
				std::cout << "OPCODE: EOR" << std::endl;
#endif
				break;
			case OP_SUB:
#ifdef DEBUG
				std::cout << "OPCODE: SUB" << std::endl;
#endif
				break;
			case OP_RSB:
#ifdef DEBUG
				std::cout << "OPCODE: RSB" << std::endl;
#endif
				break;
			case OP_ADD:
#ifdef DEBUG
				std::cout << "OPCODE: ADD" << std::endl;
#endif
				add(b);
				break;
			case OP_ADC:
#ifdef DEBUG
				std::cout << "OPCODE: ADC" << std::endl;
#endif
				break;
			case OP_SBC:
#ifdef DEBUG
				std::cout << "OPCODE: SBC" << std::endl;
#endif
				break;
			case OP_RSC:
#ifdef DEBUG
				std::cout << "OPCODE: RSC" << std::endl;
#endif
				break;
			case OP_TST:
#ifdef DEBUG
				std::cout << "OPCODE: TST" << std::endl;
#endif
				break;
			case OP_TEQ:
#ifdef DEBUG
				std::cout << "OPCODE: TEQ" << std::endl;
#endif
				break;
			case OP_CMP:
#ifdef DEBUG
				std::cout << "OPCODE: CMP" << std::endl;
#endif
				break;
			case OP_CMN:
#ifdef DEBUG
				std::cout << "OPCODE: CMN" << std::endl;
#endif
				break;
			case OP_ORR:
#ifdef DEBUG
				std::cout << "OPCODE: ORR" << std::endl;
#endif
				break;
			case OP_MOV:
#ifdef DEBUG
				std::cout << "OPCODE: MOV" << std::endl;
#endif
				break;
			case OP_BIC:
#ifdef DEBUG
				std::cout << "OPCODE: BIC" << std::endl;
#endif
				break;
			case OP_MVN:
#ifdef DEBUG
				std::cout << "OPCODE: MVN" << std::endl;
#endif
				break;
		}
		regs.dump();
}

void System::add(uint8_t *b){
	uint8_t Rn = (b[19] << 3) | (b[18] << 2) | (b[17] << 1) | b[16];
	uint8_t Rd = (b[15] << 3) | (b[14] << 2) | (b[13] << 1) | b[12];
	uint8_t Rm = (b[3] << 3 ) | (b[2] << 2) | (b[1] << 1) | b[0];
	uint8_t Shift = (b[11] << 7) | (b[10] << 6) | (b[9] << 5) | (b[8] << 4) |
			(b[7] << 3) | (b[6] << 2) | (b[5] << 1) | b[4];
	uint8_t Rotate = (b[11] << 3) | (b[10] << 2) | (b[9] << 1) | b[8];
	uint32_t Op2 = 0;
	for(int i=0 ; i<12 ; i++)
		Op2 |= b[i] << i;
	uint32_t Imm = Op2;
	bool N=false, Z=false, C=false, O=false;

	//regs.r[0] = 0x8FFFFFFF;
	//regs.r[2] = 0x8FFFFFFF;

#ifdef DISASM
	std::cout << std::setfill('0') << std::setw(8) << std::hex << regs.r[REG_R15]-4 << ": ";
	if(b[20])
		std::cout << "ADDS ";
	else
		std::cout << "ADD ";
	std::cout << "R" << std::dec << (unsigned int)Rd << ", ";
	std::cout << "R" << std::dec << (unsigned int)Rn << ", ";
	if(b[25])
		std::cout << "#" << std::hex << (unsigned int)Imm << std::endl;
	else
		std::cout << "R" << std::dec << (unsigned int)Rm << std::endl;
#endif
	if(b[25]){	// Immediate ?
		N = checkNegative('+', regs.r[Rn], Imm << Rotate);
		Z = checkZero('+', regs.r[Rn], Imm << Rotate);
		C = checkCarry('+', regs.r[Rn], Imm << Rotate);
		O = checkOverflow('+', regs.r[Rn], Imm << Rotate);
		regs.r[Rd] = regs.r[Rn] + (Imm << Rotate);
	}else{		// Register
		N = checkNegative('+', regs.r[Rn], (regs.r[Rm] << Shift));
		Z = checkZero('+', regs.r[Rn], (regs.r[Rm] << Shift));
		C = checkCarry('+', regs.r[Rn], (regs.r[Rm] << Shift));
		O = checkOverflow('+', regs.r[Rn], (regs.r[Rm] << Shift));
		regs.r[Rd] = regs.r[Rn] + (regs.r[Rm] << Shift);
	}
	if(b[20]){	// Suffix ?
		// Negative flag
		if(N) regs.cpsr |= (1 << 31); else regs.cpsr &= ~(1 << 31);
		// Zero flag
		if(Z) regs.cpsr |= (1 << 30); else regs.cpsr &= ~(1 << 30);
		// Carry flag
		if(C) regs.cpsr |= (1 << 29); else regs.cpsr &= ~(1 << 29);
		// Overflow flag
		if(O) regs.cpsr |= (1 << 28); else regs.cpsr &= ~(1 << 28);
	}
}

