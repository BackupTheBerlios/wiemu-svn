#ifndef SYSTEM_HH
#define SYSTEM_HH

#include "../../firmware.hh"
#include "regs.hh"

#define OP_AND 0x0
#define OP_EOR 0x1
#define OP_SUB 0x2
#define OP_RSB 0x3
#define OP_ADD 0x4
#define OP_ADC 0x5
#define OP_SBC 0x6
#define OP_RSC 0x7
#define OP_TST 0x8
#define OP_TEQ 0x9
#define OP_CMP 0xa
#define OP_CMN 0xb
#define OP_ORR 0xc
#define OP_MOV 0xd
#define OP_BIC 0xe
#define OP_MVN 0xf

class System{
private:
	Regs regs;
	Firmware fw;
	char *mem;
	int msize;		// Memory size
	void initMem(unsigned int);
	bool checkNegative(char opr, uint32_t, uint32_t);
	bool checkZero(char opr, uint32_t, uint32_t);
	bool checkCarry(char opr, uint32_t, uint32_t);
	bool checkOverflow(char opr, uint32_t, uint32_t);
	void cat1(uint8_t *);
	void add(uint8_t *);	
public:
	System();
	~System();
	void step(void);
};

#endif

