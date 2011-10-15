/**
    avr.hh
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

#ifndef AVR_HH
#define AVR_HH

#include "../../include/mcu.hh"
#include "../../include/clock.hh"
#include "../../include/firmware.hh"
#include "../../include/device.hh"
#include "../../include/internaldevice.hh"
#include "../../include/debugger.hh"
#include "mem.hh"
#include "regs.hh"
#include "pins.hh"
#include "timer.hh"
#include "spi.hh"
#include "adc.hh"
#include <vector>
#include <queue>

#define	AVR_FREQ	7372800 	// ~7MB

#define MSK_ADC		0xfc00
#define MSK_ADD		0xfc00
#define MSK_ADIW	0xff00
#define MSK_AND		0xfc00
#define MSK_ANDI	0xf000
#define MSK_ASR		0xfe0f
#define MSK_BCLR	0xff8f
#define MSK_BLD		0xfe08
#define MSK_BRBC	0xfc00
#define MSK_BRBS	0xfc00
#define MSK_BREAK	0xffff
#define MSK_BSET	0xff8f
#define MSK_BST		0xfe08
#define MSK_CALL	0xfe0e
#define MSK_CBI		0xff00
#define MSK_COM		0xfe0f
#define MSK_CP		0xfc00
#define MSK_CPC		0xfc00
#define MSK_CPI		0xf000
#define MSK_CPSE	0xfc00
#define MSK_DEC		0xfe0f
#define MSK_ELPM1	0xffff
#define MSK_ELPM2	0xfe0f
#define MSK_ELPM3	0xfe0f
#define MSK_EOR		0xfc00
#define MSK_FMUL	0xff88
#define MSK_FMULS	0xff88
#define MSK_FMULSU	0xff88
#define MSK_ICALL	0xffff
#define MSK_IJMP	0xffff
#define MSK_IN		0xf800
#define MSK_INC		0xfe0f
#define MSK_JMP		0xfe0e
#define MSK_LDI		0xf000
#define MSK_LDS		0xf800
#define MSK_LDS32	0xfe0f
#define MSK_LDX1	0xfe0f
#define MSK_LDX2	0xfe0f
#define MSK_LDX3	0xfe0f
#define MSK_LDY1	0xfe0f		// Not needed, implemented via LDY4
#define MSK_LDY2	0xfe0f
#define MSK_LDY3	0xfe0f
#define MSK_LDY4	0xd208
#define MSK_LDZ1	0xfe0f		// Not needed, implemented via LDZ4
#define MSK_LDZ2	0xfe0f
#define MSK_LDZ3	0xfe0f
#define MSK_LDZ4	0xd208
#define MSK_LPM1	0xffff
#define MSK_LPM2	0xfe0f
#define MSK_LPM3	0xfe0f
#define MSK_LSR		0xfe0f
#define MSK_MOV		0xfc00
#define MSK_MOVW	0xff00
#define MSK_MUL		0xfc00
#define MSK_MULS	0xff00
#define MSK_MULSU	0xff88
#define MSK_NEG		0xfe0f
#define MSK_NOP		0xffff
#define MSK_OR		0xfc00
#define MSK_ORI		0xf000
#define MSK_OUT		0xf800
#define MSK_POP		0xfe0f
#define MSK_PUSH	0xfe0f
#define MSK_RCALL	0xf000
#define MSK_RET		0xffff
#define MSK_RETI	0xffff
#define MSK_RJMP	0xf000
#define MSK_ROR		0xfe0f
#define MSK_SBC		0xfc00
#define MSK_SBCI	0xf000
#define MSK_SBI		0xff00
#define MSK_SBIC	0xff00
#define MSK_SBIS	0xff00
#define MSK_SBIW	0xff00
#define MSK_SBRC	0xfe08
#define MSK_SBRS	0xfe08
#define MSK_SLEEP	0xffff
#define MSK_STS		0xf800
#define MSK_STS32	0xfe0f
#define MSK_STX1	0xfe0f
#define MSK_STX2	0xfe0f
#define MSK_STX3	0xfe0f
#define MSK_STY1	0xfe0f		// Not needed, implemented via STY4
#define MSK_STY2	0xfe0f
#define MSK_STY3	0xfe0f
#define MSK_STY4	0xd208
#define MSK_STZ1	0xfe0f		// Not needed, implemented via STZ4
#define MSK_STZ2	0xfe0f
#define MSK_STZ3	0xfe0f
#define MSK_STZ4	0xd208
#define MSK_SUB		0xfc00
#define MSK_SUBI	0xf000
#define MSK_SWAP	0xfe0f
#define MSK_WDR		0xffff

#define OP_ADC		0x1c00
#define OP_ADD	 	0x0c00
#define OP_ADIW		0x9600
#define OP_AND		0x2000
#define OP_ANDI		0x7000
#define OP_ASR		0x9405
#define OP_BCLR		0x9488
#define OP_BLD		0xf800
#define OP_BRBC		0xf400
#define OP_BRBS		0xf000
#define OP_BREAK	0x9598
#define OP_BSET		0x9408
#define OP_BST		0xfa00
#define OP_CALL		0x940e
#define OP_CBI		0x9800
#define OP_COM		0x9400
#define OP_CP		0x1400
#define OP_CPC		0x0400
#define OP_CPI		0x3000
#define OP_CPSE		0x1000
#define OP_DEC		0x940a
#define OP_ELPM1	0x95d8
#define OP_ELPM2	0x9006
#define OP_ELPM3	0x9007
#define OP_EOR		0x2400
#define OP_FMUL		0x0308
#define OP_FMULS	0x0380
#define OP_FMULSU	0x0388
#define OP_ICALL	0x9509
#define OP_IJMP		0x9409
#define OP_IN		0xb000
#define OP_INC		0x9403
#define OP_JMP		0x940c
#define OP_LDI		0xe000
#define OP_LDS		0xa000
#define OP_LDS32	0x9000
#define OP_LDX1		0x900c
#define OP_LDX2		0x900d
#define OP_LDX3		0x900e
#define OP_LDY1		0x8008		// Not needed, implemented via LDY4
#define OP_LDY2		0x9009
#define OP_LDY3		0x900a
#define OP_LDY4		0x8008
#define OP_LDZ1		0x8000		// Not needed, implemented via LDZ4
#define OP_LDZ2		0x9001
#define OP_LDZ3		0x9002
#define OP_LDZ4		0x8000
#define OP_LPM1		0x95c8
#define OP_LPM2		0x9004
#define OP_LPM3		0x9005
#define OP_LSR		0x9406
#define OP_MOV		0x2c00
#define OP_MOVW		0x0100
#define OP_MUL		0x9c00
#define OP_MULS		0x0200
#define OP_MULSU	0x0300
#define OP_NEG		0x9401
#define OP_NOP		0x0000
#define OP_OR		0x2800
#define OP_ORI		0x6000
#define OP_OUT		0xb800
#define OP_POP		0x900f
#define OP_PUSH		0x920f
#define OP_RCALL	0xd000
#define OP_RET		0x9508
#define OP_RETI		0x9518
#define OP_RJMP		0xc000
#define OP_ROR		0x9407
#define OP_SBC		0x0800
#define OP_SBCI		0x4000
#define OP_SBI		0x9a00
#define OP_SBIC		0x9900
#define OP_SBIS		0x9b00
#define OP_SBIW		0x9700
#define OP_SBRC		0xfc00
#define OP_SBRS		0xfe00
#define OP_SLEEP	0x9588
#define OP_STS		0xa800
#define OP_STS32	0x9200
#define OP_STX1		0x920c
#define OP_STX2		0x920d
#define OP_STX3		0x920e
#define OP_STY1		0x8208		// Not needed, implemented via STY4
#define OP_STY2		0x9209
#define OP_STY3		0x920a
#define OP_STY4		0x8208
#define OP_STZ1		0x8200		// Not needed, implemented via STZ4
#define OP_STZ2		0x9201
#define OP_STZ3		0x9202
#define OP_STZ4		0x8200
#define OP_SUB		0x1800
#define OP_SUBI		0x5000
#define OP_SWAP		0x9402
#define OP_WDR		0x95a8


class Avr: public Mcu{
// Friend classes
	friend class Regs;
	friend class Mem;
// Friend internal devices
	friend class Timer;
	friend class Spi;
	friend class Adc;
private:
	Regs *regs;
	Firmware fw;
	Timer *timer;
	Mem *sram;
	Spi *spi;
	Adc *adc;
	uint16_t *flash;
	uint8_t *eeprom;
	Clock clock;					// System clock (cycles)
	Debugger *debug;				// The Debugger
	Pin pins[AVR_NUM_PINS];				// MCU Pins
	int fsize;					// Flash size
	int esize;					// EEPROM size
	uint16_t opcode;				// Current opcode
	unsigned int watchdog_timer;			// Watchdog timer
	bool stopped;					// Used by BREAK
	bool sleeping;
	unsigned int instructions;			// Number of instructions executed
	std::vector<InternalDevice *> intrn_devices;	// List of attached internal decices
	std::queue<uint8_t> interrupts;
	uint8_t nextInterrupt;
	void initSRAM(unsigned int);			// Data Memory
	void initFLASH(unsigned int);			// Program Memory
	void initEEPROM(unsigned int);			// EEPROM Memory
	void initPins();				// Initialize MCU Pins
	void readPins();				// Read from MCU Pins to IO Regs
	void writePins();				// Write from IO Regs to MCU Pins
	void readPin(uint16_t, uint16_t, uint16_t, uint16_t, uint8_t);
	void writePin(uint16_t, uint16_t, uint16_t, uint16_t, uint8_t);
	void printPins();
	void reverseEndian(uint16_t &);
	unsigned int getBit(uint8_t, unsigned int);
	unsigned int getBit(uint16_t, unsigned int);
	void setBit(uint8_t &, unsigned int);
	void clearBit(uint8_t &, unsigned int);
	void _add();
	void _adc();
	void _and();
	void _ldi();
	void _or();
	void _eor();
	void _andi();
	void _ori();
	void _nop();
	void _mov();
	void _sub();
	void _adiw();
	void _sbiw();
	void _bclr();
	void _bset();
	void _subi();
	void _rjmp();
	void _neg();
	void _inc();
	void _dec();
	void _brbc();
	void _brbs();
	void _com();
	void _cp();
	void _cpc();
	void _cpi();
	void _lsr();
	void _swap();
	void _push();
	void _pop();
	void _out();
	void _in();
	void _rcall();
	void _ret();
	void _ror();
	void _sts();
	void _sts32();
	void _lds();
	void _lds32();
	void _stx();
	void _sty();
	void _stz();
	void _ldx();
	void _ldy();
	void _ldz();
	void _break();
	void _asr();
	void _bld();
	void _bst();
	void _cbi();
	void _sbi();
	void _cpse();
	void _ijmp();
	void _icall();
	void _jmp();
	void _call();
	void _reti();
	void _mul();
	void _muls();
	void _mulsu();
	void _fmul();
	void _fmuls();
	void _fmulsu();
	void _movw();
	void _sbrc();
	void _sbrs();
	void _sleep();
	void _wdr();
	void _sbci();
	void _sbc();
	void _sbic();
	void _sbis();
	void _lpm();
	void _elpm();
	void illegal();
	void awake();
	void interrupt(uint8_t);
	void fireInterrupt(uint8_t);
	void addInternalDevice(InternalDevice *);
public:
	Avr();
	~Avr();
	void loadImage(std::string);
	Pin* getPins();
	void step(void);
	void run(void);
	uint64_t getCycles(void);
	Clock getClock(void);
	void setDebugger(Debugger *);
	void dumpRegs(void);
	void addIOListener(uint8_t, InternalDevice *);
	void addClockEvent(Event *);
	uint16_t getPC();
};

#endif

