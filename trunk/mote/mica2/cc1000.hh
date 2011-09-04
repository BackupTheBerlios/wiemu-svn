/**
    cc1000.hh
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

#ifndef CC1000_HH
#define CC1000_HH

#define	CC1000_NUM_PINS		28
#define	CC1000_NUM_REGS		0x47

#include <stdint.h>
#include <queue>
#include "../../include/device"
#include "../../include/pin"

#include <fstream>	// <---- temp

#define		LOW			false
#define		HIGH			true

#define		CC1000_PIN_PALE		26
#define		CC1000_PIN_PCLK		24
#define		CC1000_PIN_PDATA	25
#define		CC1000_PIN_DIO		22
#define		CC1000_PIN_DCLK		23
#define		CC1000_PIN_RSSI		27

#define		CC1000_REG_MAIN		0x0		// MAIN Register
#define		CC1000_REG_FREQ_2A	0x1		// Frequency Register 2A
#define		CC1000_REG_FREQ_1A	0x2		// Frequency Register 1A
#define		CC1000_REG_FREQ_0A	0x3		// Frequency Register 0A
#define		CC1000_REG_FREQ_2B	0x4		// Frequency Register 2B
#define		CC1000_REG_FREQ_1B	0x5		// Frequency Register 1B
#define		CC1000_REG_FREQ_0B	0x6		// Frequency Register 0B
#define		CC1000_REG_FSEP1	0x7		// Frequency Separation Register 1
#define		CC1000_REG_FSEP0	0x8		// Frequency Separation Register 0
#define		CC1000_REG_CURRENT	0x9		// Current Consumption Control Register
#define		CC1000_REG_FRONT_END	0xa		// Front End Control Register
#define		CC1000_REG_PA_POW	0xb		// PA Output Power Control Register
#define		CC1000_REG_PLL		0xc		// PLL Control Register
#define		CC1000_REG_LOCK		0xd		// LOCK Status Register and signal select to CHP_OUT (LOCK) pin
#define		CC1000_REG_CAL		0xe		// VCO Calibration Control and Status Register
#define		CC1000_REG_MODEM2	0xf		// Modem Control Register 2
#define		CC1000_REG_MODEM1	0x10		// Modem Control Register 1
#define		CC1000_REG_MODEM0	0x11		// Modem Control Register 0
#define		CC1000_REG_MATCH	0x12		// Match Capacitor Array Control Register for RX and TX impedance matching
#define		CC1000_REG_FSCTRL	0x13		// Frequency Synthesiser Control Register
#define		CC1000_REG_PRESCALER	0x1c		// Prescaler and IF-strip test control register
#define		CC1000_REG_TEST6	0x40		// Test register for PLL LOOP
#define		CC1000_REG_TEST5	0x41		// Test register for PLL LOOP
#define		CC1000_REG_TEST4	0x42		// Test register for PLL LOOP
#define		CC1000_REG_TEST3	0x43		// Test register for VCO
#define		CC1000_REG_TEST2	0x44		// Test register for Calibration
#define		CC1000_REG_TEST1	0x45		// Test register for Calibration
#define		CC1000_REG_TEST0	0x46		// Test register for Calibration


#define		CC1000_MAIN_RXTX	7
#define		CC1000_MAIN_F_REG	6
#define		CC1000_MAIN_RX_PD	5
#define		CC1000_MAIN_TX_PD	4
#define		CC1000_MAIN_FS_PD	3
#define		CC1000_MAIN_CORE_PD	2
#define		CC1000_MAIN_BIAS_PD	1
#define		CC1000_MAIN_RESET_N	0

#define		CC1000_CAL_START	7
#define		CC1000_CAL_DUAL		6
#define		CC1000_CAL_WAIT		5
#define		CC1000_CAL_CURRENT	4
#define		CC1000_CAL_COMPLETE	3
#define		CC1000_CAL_ITERATE	0


class CC1000: public Device{
	// Inner classes
	class ClockEvent: public Event{
	private:
		CC1000 *cc1000;
	public:
		ClockEvent();
		void setDevice(void *);
		void fired();
	};
	class CallibrationEvent: public Event{
	private:
		CC1000 *cc1000;
	public:
		void setDevice(void *);
		void fired();
		void setTime(double);
	};
public:
	CC1000();
	~CC1000();
	void setMCU(Mcu *);
	void tick();
	Pin* getPins();
private:
	Mcu *mcu;
	std::ofstream logf;	// <----- temp
	Pin pins[CC1000_NUM_PINS];
	uint8_t registers[CC1000_NUM_REGS];
	int address_nbits, w_nbits, data_nbits;
	uint8_t address;
	bool w;
	uint8_t data;
	bool pale, pale_prev;
	bool pclk, pclk_prev;
	bool pdata, pdata_prev;
	bool dio, dio_prev;
	bool dclk, dclk_prev;
	bool rssi, rssi_prev;
	uint32_t freqA, freqB;
	uint16_t fsep;
	std::queue<void (CC1000::*)()> callbacks;

	static const double BAUD_RATE[8];
	static const std::string ENCODING[4];
	static const double XOSC_FREQ[4];

	// Event Handlers
	ClockEvent clk_event;
	CallibrationEvent cal_event;

	void readAddress();
	void readW();
	void readData();
	void writeData();
	void print1();
	void print2();
	void print3();
	void dump(uint64_t);
	void resetRegisters();
	void configure(uint8_t);
	void calibrate();
	void calibrationCompleted();
	void compare();
};

#endif

