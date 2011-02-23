#ifndef REGS_HH
#define REGS_HH

#include <stdint.h>

#define REG_R0 0x0
#define REG_R1 0x1
#define REG_R2 0x2
#define REG_R3 0x3
#define REG_R4 0x4
#define REG_R5 0x5
#define REG_R6 0x6
#define REG_R7 0x7
#define REG_R8 0x8
#define REG_R9 0x9
#define REG_R10 0xa
#define REG_R11 0xb
#define REG_R12 0xc
#define REG_R13 0xd	// Stack
#define REG_R14 0xe	// Ret
#define REG_R15 0xf	// PC

class Regs{
public:
	uint32_t r[16];	// General Registers
	uint32_t cpsr;	// Current Program Status Register
	Regs();
	~Regs();
	void dump(void);
};

#endif

