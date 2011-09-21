/**
    cc1000.cc
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

#include "cc1000.hh"
#include <iostream>

#include <fstream>
#include <cstdlib>

#include "../../arch/avr/avr.hh"

const double CC1000::BAUD_RATE[8] = {
	0.6,		// 0.6 kDb
	1.2,		// 1.2 kDb
	2.4,		// 2.4 kDb
	4.8,		// 4.8 kDb
	9.6,		// 9.6 kDb
	19.2,		// 19.2 kDb
	0,		// Not used
	0		// Not used
};

const std::string CC1000::ENCODING[4] = {
	"NRZ operation",
	"Manchester operation",
	"Transparent Asyncronous UART operation",
	"Not used"
};

const double CC1000::XOSC_FREQ[4] = {
	3.6864,		// 3 -> 4 MHz crystal 3.6864MHz, Also used for 76.8 kBaud, 14.7456MHz
	7.3728,		// 6 -> 8 MHz crystal 7.3728MHz, Also used for 38.4 kBaud, 14.7456MHz
	11.0592,	// 9 -> 12 MHz crystal 11.0592MHz
	14.7456,	// 12 -> 16 MHz crystall 14.7456MHz
};

CC1000::CC1000()
{
	// CC1000 Pins
	pins[0] = Pin("AVDD");
	pins[1] = Pin("AGND");
	pins[2] = Pin("RF_IN");
	pins[3] = Pin("RF_OUT");
	pins[4] = Pin("AVDD");
	pins[5] = Pin("AGND");
	pins[6] = Pin("AGND");
	pins[7] = Pin("AGND");
	pins[8] = Pin("AVDD");
	pins[9] = Pin("L1");
	pins[10] = Pin("L2");
	pins[11] = Pin("CHP_OUT");
	pins[12] = Pin("R_BIAS");
	pins[13] = Pin("AGND");
	pins[14] = Pin("AVDD");
	pins[15] = Pin("AGND");
	pins[16] = Pin("XOSC_Q2");
	pins[17] = Pin("XOSC_Q1");
	pins[18] = Pin("AGND");
	pins[19] = Pin("DGND");
	pins[20] = Pin("DVDD");
	pins[21] = Pin("DGND");
	pins[CC1000_PIN_PALE] = Pin("PALE");
	pins[CC1000_PIN_PCLK] = Pin("PCLK");
	pins[CC1000_PIN_PDATA] = Pin("PDATA");
	pins[CC1000_PIN_DIO] = Pin("DIO");
	pins[CC1000_PIN_DCLK] = Pin("DCLK");
	pins[CC1000_PIN_RSSI] = Pin("RSSI");
	
	// CC1000 Control Registers
	registers[CC1000_REG_MAIN] = 0x0; //0x3e;
	resetRegisters();

	// Initialize the internal dummy pins
	pale = pins[CC1000_PIN_PALE].get();
	pclk = pins[CC1000_PIN_PCLK].get();
	pdata = pins[CC1000_PIN_PDATA].get();
	dclk = LOW;
	dio = LOW;
	address = 0;
	w = false;
	data = 0;
	rx_pd = false;
	tx_pd = false;

	clk_event.setDevice((void *)this);
	cal_event.setDevice((void *)this);
	dclk_event.setDevice((void *)this);	

	logf.open("cc1000.log");
	logf2.open("cc1000_read.log");
}

CC1000::~CC1000()
{
	logf.close();
	logf2.close();
}

void CC1000::setMCU(Mcu *mcu)
{
	this->mcu = mcu;
	mcu->addClockEvent(&clk_event);
}

void
CC1000::tick()
{
	if(!callbacks.empty()){
		void (CC1000::*func)() = callbacks.front();
		(this->*func)();
		// The callbacks should clean themselves!
	}
	// Control
	pale_prev = pale;
	pale = pins[CC1000_PIN_PALE].get();
	pclk_prev = pclk;
	pclk = pins[CC1000_PIN_PCLK].get();
	if(pale_prev == HIGH && pale == LOW){		// Address
		address = 0;
		address_nbits = 0;
		w_nbits = 0;
		w = false;
		// Clear callbacks queue
		while(!callbacks.empty())
			callbacks.pop();
		callbacks.push(&CC1000::readAddress);
		callbacks.push(&CC1000::readW);
	}
	else if(pale_prev == LOW && pale == HIGH){	// Data
		data = 0;
		data_nbits = 0;
		if(w){
			callbacks.push(&CC1000::writeData);
		}else{
			callbacks.push(&CC1000::readData);
		}
	}
	// TODO: implement subclocks instead of events
	mcu->addClockEvent(&clk_event);
}

void
CC1000::readAddress()
{
	if(pclk_prev == LOW && pclk == HIGH){
#ifdef DEBUG
		//std::cerr << "(A) PCLK Toggled " << address_nbits << std::endl;
#endif
		address <<= 1;
		if(pins[CC1000_PIN_PDATA].get())
			address |= 1;
		++address_nbits;
		if(address_nbits == 7){
			print1();
			callbacks.pop();		// Clean myself!
		}
	}
}

void
CC1000::readW()
{
	if(pclk_prev == LOW && pclk == HIGH){
#ifdef DEBUG
		//std::cerr << "(W) PCLK Toggled " << w_nbits << std::endl;
#endif
		if(pins[CC1000_PIN_PDATA].get())
			w = true;
		else
			w = false;
		++w_nbits;
		if(w_nbits == 1){
			print2();
			callbacks.pop();		// Clean myself!
		}
	}
}

void
CC1000::writeData()
{
	if(pclk_prev == HIGH && pclk == LOW){
		if(data_nbits == 0)
			data = 0;
#ifdef DEBUG
		//std::cerr << "(D) PCLK Toggled " << data_nbits << " with " << (int)pins[CC1000_PIN_PDATA].get() << std::endl;
#endif
		if(pins[CC1000_PIN_PDATA].get())
			data |= 1;
		++data_nbits;
		if(data_nbits == 8){
			registers[address] = data;
			print3();
			callbacks.pop();		// Clean myself!
			configure(address);		// Update configuration
		}
		data <<= 1;
	}
}

void
CC1000::readData()
{
	if(pclk_prev == HIGH && pclk == LOW){
		if(data_nbits == 0){
			data = registers[address];
		}
		if(data & 0x80){
			pins[CC1000_PIN_PDATA].set();
#ifdef DEBUG
			//std::cerr << "(D) PCLK Toggled " << data_nbits << " with 1" << std::endl;
#endif
		}
		else{
			pins[CC1000_PIN_PDATA].clear();
#ifdef DEBUG
			//std::cerr << "(D) PCLK Toggled " << data_nbits << " with 0" << std::endl;
#endif
		}
		++data_nbits;
		if(data_nbits == 8){
			print3();
			callbacks.pop();		// Clean myself!
			dump3(mcu->getCycles(), address);
		}
		data <<= 1;
	}
}

void
CC1000::print1()
{
#ifdef DEBUG
	//std::cerr << "Address = " << std::hex << (int)address << std::dec;
#endif
}

void
CC1000::print2()
{
#ifdef DEBUG
	//std::cerr << " W = " << std::hex << (int)w << std::dec;
#endif
}

void
CC1000::print3()
{
#ifdef DEBUG
	//std::cerr << " Data = " << std::hex << (int)registers[address] << std::dec << std::endl;
#endif
}

void
CC1000::dump(uint64_t time)
{
	logf << "SCLK = " << (int)time << std::endl;
	logf << "MAIN = " << (int)registers[CC1000_REG_MAIN] << std::endl;
	logf << "FREQ_2A = " << (int)registers[CC1000_REG_FREQ_2A] << std::endl;
	logf << "FREQ_1A = " << (int)registers[CC1000_REG_FREQ_1A] << std::endl;
	logf << "FREQ_0A = " << (int)registers[CC1000_REG_FREQ_0A] << std::endl;
	logf << "FREQ_2B = " << (int)registers[CC1000_REG_FREQ_2B] << std::endl;
	logf << "FREQ_1B = " << (int)registers[CC1000_REG_FREQ_1B] << std::endl;
	logf << "FREQ_0B = " << (int)registers[CC1000_REG_FREQ_0B] << std::endl;
	logf << "FSEP1 = " << (int)registers[CC1000_REG_FSEP1] << std::endl;
	logf << "FSEP0 = " << (int)registers[CC1000_REG_FSEP0] << std::endl;
	logf << "CURRENT = " << (int)registers[CC1000_REG_CURRENT] << std::endl;
	logf << "FRONT_END = " << (int)registers[CC1000_REG_FRONT_END] << std::endl;
	logf << "PA_POW = " << (int)registers[CC1000_REG_PA_POW] << std::endl;
	logf << "PLL = " << (int)registers[CC1000_REG_PLL] << std::endl;
	logf << "LOCK = " << (int)registers[CC1000_REG_LOCK] << std::endl;
	logf << "CAL = " << (int)registers[CC1000_REG_CAL] << std::endl;
	logf << "MODEM2 = " << (int)registers[CC1000_REG_MODEM2] << std::endl;
	logf << "MODEM1 = " << (int)registers[CC1000_REG_MODEM1] << std::endl;
	logf << "MODEM0 = " << (int)registers[CC1000_REG_MODEM0] << std::endl;
	logf << "MATCH = " << (int)registers[CC1000_REG_MATCH] << std::endl;
	logf << "FSCTRL = " << (int)registers[CC1000_REG_FSCTRL] << std::endl;
	logf << "PRESCALER = " << (int)registers[CC1000_REG_PRESCALER] << std::endl;
	logf << "TEST6 = " << (int)registers[CC1000_REG_TEST6] << std::endl;
	logf << "TEST5 = " << (int)registers[CC1000_REG_TEST5] << std::endl;
	logf << "TEST4 = " << (int)registers[CC1000_REG_TEST4] << std::endl;
	logf << "TEST3 = " << (int)registers[CC1000_REG_TEST3] << std::endl;
	logf << "TEST2 = " << (int)registers[CC1000_REG_TEST2] << std::endl;
	logf << "TEST1 = " << (int)registers[CC1000_REG_TEST1] << std::endl;
	logf << "TEST0 = " << (int)registers[CC1000_REG_TEST0] << std::endl;
	logf << std::endl << std::endl;
	logf.flush();
}

void
CC1000::dump2(uint64_t time, int reg)
{
	logf << "SCLK = " << (int)time << "\tPC = " << ((Avr *)mcu)->getPC()*2 << "\t";
	switch(reg){
		case CC1000_REG_MAIN:
			logf << "MAIN = " << (int)registers[CC1000_REG_MAIN] << std::endl;
			break;
		case CC1000_REG_FREQ_2A:
			logf << "FREQ2A = " << (int)registers[CC1000_REG_FREQ_2A] << std::endl;
			break;
		case CC1000_REG_FREQ_1A:
			logf << "FREQ1A = " << (int)registers[CC1000_REG_FREQ_1A] << std::endl;
			break;
		case CC1000_REG_FREQ_0A:
			logf << "FREQ0A = " << (int)registers[CC1000_REG_FREQ_0A] << std::endl;
			break;
		case CC1000_REG_FREQ_2B:
			logf << "FREQ2B = " << (int)registers[CC1000_REG_FREQ_2B] << std::endl;
			break;
		case CC1000_REG_FREQ_1B:
			logf << "FREQ1B = " << (int)registers[CC1000_REG_FREQ_1B] << std::endl;
			break;
		case CC1000_REG_FREQ_0B:
			logf << "FREQ0B = " << (int)registers[CC1000_REG_FREQ_0B] << std::endl;
			break;
		case CC1000_REG_FSEP1:
			logf << "FSEP1 = " << (int)registers[CC1000_REG_FSEP1] << std::endl;
			break;
		case CC1000_REG_FSEP0:
			logf << "FSEP0 = " << (int)registers[CC1000_REG_FSEP0] << std::endl;
			break;
		case CC1000_REG_CURRENT:
			logf << "CURRENT = " << (int)registers[CC1000_REG_CURRENT] << std::endl;
			break;
		case CC1000_REG_FRONT_END:
			logf << "FRONT_END = " << (int)registers[CC1000_REG_FRONT_END] << std::endl;
			break;
		case CC1000_REG_PA_POW:
			logf << "PA_POW = " << (int)registers[CC1000_REG_PA_POW] << std::endl;
			break;
		case CC1000_REG_PLL:
			logf << "PLL = " << (int)registers[CC1000_REG_PLL] << std::endl;
			break;
		case CC1000_REG_LOCK:
			logf << "LOCK = " << (int)registers[CC1000_REG_LOCK] << std::endl;
			break;
		case CC1000_REG_CAL:
			logf << "CAL = " << (int)registers[CC1000_REG_CAL] << std::endl;
			break;
		case CC1000_REG_MODEM2:
			logf << "MODEM2 = " << (int)registers[CC1000_REG_MODEM2] << std::endl;
			break;
		case CC1000_REG_MODEM1:
			logf << "MODEM1 = " << (int)registers[CC1000_REG_MODEM1] << std::endl;
			break;
		case CC1000_REG_MODEM0:
			logf << "MODEM0 = " << (int)registers[CC1000_REG_MODEM0] << std::endl;
			break;
		case CC1000_REG_MATCH:
			logf << "MATCH = " << (int)registers[CC1000_REG_MATCH] << std::endl;
			break;
		case CC1000_REG_FSCTRL:
			logf << "FSCTRL = " << (int)registers[CC1000_REG_FSCTRL] << std::endl;
			break;
		case CC1000_REG_PRESCALER:
			logf << "PRESCALER = " << (int)registers[CC1000_REG_PRESCALER] << std::endl;
			break;
		case CC1000_REG_TEST6:
			logf << "TEST6 = " << (int)registers[CC1000_REG_TEST6] << std::endl;
			break;
		case CC1000_REG_TEST5:
			logf << "TEST5 = " << (int)registers[CC1000_REG_TEST5] << std::endl;
			break;
		case CC1000_REG_TEST4:
			logf << "TEST4 = " << (int)registers[CC1000_REG_TEST4] << std::endl;
			break;
		case CC1000_REG_TEST3:
			logf << "TEST3 = " << (int)registers[CC1000_REG_TEST3] << std::endl;
			break;
		case CC1000_REG_TEST2:
			logf << "TEST2 = " << (int)registers[CC1000_REG_TEST2] << std::endl;
			break;
		case CC1000_REG_TEST1:
			logf << "TEST1 = " << (int)registers[CC1000_REG_TEST1] << std::endl;
			break;
		case CC1000_REG_TEST0:
			logf << "TEST0 = " << (int)registers[CC1000_REG_TEST0] << std::endl;
			break;
	}
	logf.flush();
}

void
CC1000::dump3(uint64_t time, int reg)
{
	logf2 << "SCLK = " << (int)time << "\tPC = " << ((Avr *)mcu)->getPC()*2 << "\t";
	switch(reg){
		case CC1000_REG_MAIN:
			logf2 << "MAIN = " << (int)registers[CC1000_REG_MAIN] << std::endl;
			break;
		case CC1000_REG_FREQ_2A:
			logf2 << "FREQ2A = " << (int)registers[CC1000_REG_FREQ_2A] << std::endl;
			break;
		case CC1000_REG_FREQ_1A:
			logf2 << "FREQ1A = " << (int)registers[CC1000_REG_FREQ_1A] << std::endl;
			break;
		case CC1000_REG_FREQ_0A:
			logf2 << "FREQ0A = " << (int)registers[CC1000_REG_FREQ_0A] << std::endl;
			break;
		case CC1000_REG_FREQ_2B:
			logf2 << "FREQ2B = " << (int)registers[CC1000_REG_FREQ_2B] << std::endl;
			break;
		case CC1000_REG_FREQ_1B:
			logf2 << "FREQ1B = " << (int)registers[CC1000_REG_FREQ_1B] << std::endl;
			break;
		case CC1000_REG_FREQ_0B:
			logf2 << "FREQ0B = " << (int)registers[CC1000_REG_FREQ_0B] << std::endl;
			break;
		case CC1000_REG_FSEP1:
			logf2 << "FSEP1 = " << (int)registers[CC1000_REG_FSEP1] << std::endl;
			break;
		case CC1000_REG_FSEP0:
			logf2 << "FSEP0 = " << (int)registers[CC1000_REG_FSEP0] << std::endl;
			break;
		case CC1000_REG_CURRENT:
			logf2 << "CURRENT = " << (int)registers[CC1000_REG_CURRENT] << std::endl;
			break;
		case CC1000_REG_FRONT_END:
			logf2 << "FRONT_END = " << (int)registers[CC1000_REG_FRONT_END] << std::endl;
			break;
		case CC1000_REG_PA_POW:
			logf2 << "PA_POW = " << (int)registers[CC1000_REG_PA_POW] << std::endl;
			break;
		case CC1000_REG_PLL:
			logf2 << "PLL = " << (int)registers[CC1000_REG_PLL] << std::endl;
			break;
		case CC1000_REG_LOCK:
			logf2 << "LOCK = " << (int)registers[CC1000_REG_LOCK] << std::endl;
			break;
		case CC1000_REG_CAL:
			logf2 << "CAL = " << (int)registers[CC1000_REG_CAL] << std::endl;
			break;
		case CC1000_REG_MODEM2:
			logf2 << "MODEM2 = " << (int)registers[CC1000_REG_MODEM2] << std::endl;
			break;
		case CC1000_REG_MODEM1:
			logf2 << "MODEM1 = " << (int)registers[CC1000_REG_MODEM1] << std::endl;
			break;
		case CC1000_REG_MODEM0:
			logf2 << "MODEM0 = " << (int)registers[CC1000_REG_MODEM0] << std::endl;
			break;
		case CC1000_REG_MATCH:
			logf2 << "MATCH = " << (int)registers[CC1000_REG_MATCH] << std::endl;
			break;
		case CC1000_REG_FSCTRL:
			logf2 << "FSCTRL = " << (int)registers[CC1000_REG_FSCTRL] << std::endl;
			break;
		case CC1000_REG_PRESCALER:
			logf2 << "PRESCALER = " << (int)registers[CC1000_REG_PRESCALER] << std::endl;
			break;
		case CC1000_REG_TEST6:
			logf2 << "TEST6 = " << (int)registers[CC1000_REG_TEST6] << std::endl;
			break;
		case CC1000_REG_TEST5:
			logf2 << "TEST5 = " << (int)registers[CC1000_REG_TEST5] << std::endl;
			break;
		case CC1000_REG_TEST4:
			logf2 << "TEST4 = " << (int)registers[CC1000_REG_TEST4] << std::endl;
			break;
		case CC1000_REG_TEST3:
			logf2 << "TEST3 = " << (int)registers[CC1000_REG_TEST3] << std::endl;
			break;
		case CC1000_REG_TEST2:
			logf2 << "TEST2 = " << (int)registers[CC1000_REG_TEST2] << std::endl;
			break;
		case CC1000_REG_TEST1:
			logf2 << "TEST1 = " << (int)registers[CC1000_REG_TEST1] << std::endl;
			break;
		case CC1000_REG_TEST0:
			logf2 << "TEST0 = " << (int)registers[CC1000_REG_TEST0] << std::endl;
			break;
	}
	logf2.flush();
}

void
CC1000::resetRegisters()
{
	registers[CC1000_REG_FREQ_2A] = 0x75;
	registers[CC1000_REG_FREQ_1A] = 0xa0;
	registers[CC1000_REG_FREQ_0A] = 0xcb;
	registers[CC1000_REG_FREQ_2B] = 0x75;
	registers[CC1000_REG_FREQ_1B] = 0xa5;
	registers[CC1000_REG_FREQ_0B] = 0x4e;
	registers[CC1000_REG_FSEP1] = 0x00;
	registers[CC1000_REG_FSEP0] = 0x59;
	registers[CC1000_REG_CURRENT] = 0xca;
	registers[CC1000_REG_FRONT_END] = 0x08;
	registers[CC1000_REG_PA_POW] = 0x0f;
	registers[CC1000_REG_PLL] = 0x10;
	registers[CC1000_REG_LOCK] = 0x00;
	registers[CC1000_REG_CAL] = 0x05;
	registers[CC1000_REG_MODEM2] = 0x96;
	registers[CC1000_REG_MODEM1] = 0x67;
	registers[CC1000_REG_MODEM0] = 0x24;
	registers[CC1000_REG_MATCH] = 0x00;
	registers[CC1000_REG_FSCTRL] = 0x01;
	registers[CC1000_REG_PRESCALER] = 0x00;
	registers[CC1000_REG_TEST6] = 0x10;
	registers[CC1000_REG_TEST5] = 0x08;
	registers[CC1000_REG_TEST4] = 0x25;
	registers[CC1000_REG_TEST3] = 0x04;
	registers[CC1000_REG_TEST2] = 0x00;		// Not initialized
	registers[CC1000_REG_TEST1] = 0x00;		// Not initialized
	registers[CC1000_REG_TEST0] = 0x00;		// Not initialized
}

void
CC1000::configure(uint8_t addr)
{
	//dump(mcu->getCycles());
	dump2(mcu->getCycles(), addr);
	switch(addr){
		case CC1000_REG_MAIN:
			// TODO: FSM
			if((registers[CC1000_REG_MAIN] & (1 << CC1000_MAIN_RESET_N)) == LOW){
				std::cerr << "Initializing CC1000..." << std::endl;
				resetRegisters();
			}
			// Receiver power down flag
			rx_pd = registers[CC1000_REG_MAIN] & (1 << CC1000_MAIN_RX_PD);
			// Trasmitter power down flag
			tx_pd = registers[CC1000_REG_MAIN] & (1 << CC1000_MAIN_TX_PD);
			// select Tx or Rx ??
			rxtx = registers[CC1000_REG_MAIN] & (1 << CC1000_MAIN_RXTX);
			if(rxtx == CC1000_RX && !rx_pd)
				std::cerr << mcu->getCycles() << " RX mode" << std::endl;
			if(rxtx == CC1000_TX && !tx_pd)
				std::cerr << mcu->getCycles() << " TX mode" << std::endl;
			if(!rx_pd || !tx_pd)
				mcu->addClockEvent(&dclk_event);
			break;
		case CC1000_REG_PA_POW:
			std::cerr << "Output Power:";
			std::cerr << " High = " << (int)(registers[CC1000_REG_PA_POW] >> 4);
			std::cerr << " Low = " << (int)(registers[CC1000_REG_PA_POW] & 0x0f) << std::endl;
			compare();
			break;
		case CC1000_REG_LOCK:
			std::cerr << "LOCK = " << (int)((registers[CC1000_REG_LOCK] & 0xf0) >> 4) << std::endl;
			break;
		case CC1000_REG_MODEM2:
			std::cerr << "MODEM2" << std::endl;
			break;
		case CC1000_REG_MODEM1:
			std::cerr << "MODEM1" << std::endl;
			break;
		case CC1000_REG_MODEM0:{
			uint8_t baudrate = (registers[CC1000_REG_MODEM0] >> 4) & 0x7;
			uint8_t dataformat = (registers[CC1000_REG_MODEM0] >> 2) & 0x3;
			uint8_t freq = registers[CC1000_REG_MODEM0] & 0x3;
			std::cerr << "MODEM0" << std::endl;
			std::cerr << "Baud rate = " << BAUD_RATE[baudrate] << " kBd" << std::endl;
			std::cerr << "Data format = " << ENCODING[dataformat] << std::endl;
			std::cerr << "XTAL frequency = " << XOSC_FREQ[freq] << " MHz" << std::endl;
			break;
		}
		case CC1000_REG_FSCTRL:
			std::cerr << "FSCTRL" << std::endl;
			break; 
		case CC1000_REG_FREQ_2A:
		case CC1000_REG_FREQ_1A:
		case CC1000_REG_FREQ_0A:
			freqA = (registers[CC1000_REG_FREQ_2A] << 16) |
				 (registers[CC1000_REG_FREQ_1A] << 8) |
				 registers[CC1000_REG_FREQ_0A];
			std::cerr << "FREQ A = " << freqA << std::endl;
			break;
		case CC1000_REG_FREQ_2B:
		case CC1000_REG_FREQ_1B:
		case CC1000_REG_FREQ_0B:
			freqB = (registers[CC1000_REG_FREQ_2B] << 16) |
				 (registers[CC1000_REG_FREQ_1B] << 8) |
				 registers[CC1000_REG_FREQ_0B];
			std::cerr << "FREQ B = " << freqB << std::endl;
			break;
		case CC1000_REG_FSEP0:
		case CC1000_REG_FSEP1:
			fsep = ((registers[CC1000_REG_FSEP1] << 8) & 0x7) |
				registers[CC1000_REG_FSEP0];
			std::cerr << "FSEP = " << fsep << std::endl;
			break;
		case CC1000_REG_CURRENT:
			std::cerr << "CURRENT = " << (int)registers[CC1000_REG_CURRENT] << std::endl;
			break;
		case CC1000_REG_MATCH:
			std::cerr << "MATCH = " << (int)registers[CC1000_REG_MATCH] << std::endl;
			break;
		case CC1000_REG_FRONT_END:
			std::cerr << "FRONT_END = " << (int)registers[CC1000_REG_FRONT_END] << std::endl;
			break;
		case CC1000_REG_PLL:
			std::cerr << "PLL = " << (int)registers[CC1000_REG_PLL] << std::endl;
			break;
		case CC1000_REG_CAL:
			std::cerr << "CAL = " << (int)registers[CC1000_REG_CAL] << std::endl;
			if(registers[CC1000_REG_CAL] & (1 << CC1000_CAL_START)){
				std::cerr << "Starting calibration..." << std::endl;
				calibrate();
			}
			break;
		case CC1000_REG_TEST4:
			std::cerr << "TEST4 = " << (int)registers[CC1000_REG_TEST4] << std::endl;
			break;		
		default:
			std::cerr << "Else: " << (int)addr << std::endl;
			break;
	}
}

void
CC1000::calibrate()
{
	uint8_t refdiv = (registers[CC1000_REG_PLL] >> 3) & 0xf;
	double FXOSC_FREQUENCY = 14745600.0;
        double cal_time = (34.0 * 1000000.0 / FXOSC_FREQUENCY) * refdiv;
	std::cerr << "Calibration should take: " << cal_time << " milliseconds" << std::endl;
	cal_event.setTime(cal_time);
	mcu->addClockEvent(&cal_event);
}

void
CC1000::calibrationCompleted()
{
	// TODO: waiting time in ms
	registers[CC1000_REG_CAL] |= 1 << CC1000_CAL_COMPLETE;
	registers[CC1000_REG_CAL] &= ~(1 << CC1000_CAL_START);
	std::cerr << "Calibration completed..." << std::endl;
}

void
CC1000::compare()
{
	uint8_t value = registers[CC1000_REG_PA_POW];
	double power;
	if(value < 0x10) {
		/* Assuming linear from 2@-19.5 to 15@-6 */
	        power = (value - 2.0) * ((-6.0 + 19.5) / (15.0 - 2.0)) - 19.5;
	} else {
	        /* Assuming linear from 4@-5 to 16@+5 */
	        power = value / 16.0;
	        power = (power - 4.0) * ((5.0 - -5.0) / (16.0 - 4.0)) - 5.0;
	}
	std::cerr << "Power (Atemu) = " << power << std::endl;
	if (value < 16)
		power = 0.12 * value - 1.8;
        else
            power = 0.00431 * value - 0.06459;
	std::cerr << "Power (Avrora) = " << power << std::endl;
}

Pin*
CC1000::getPins()
{
	return this->pins;
}

void
CC1000::DClkTick()
{
	dclk_prev = dclk;
	dclk = !dclk;
	if(dclk)
		pins[CC1000_PIN_DCLK].set();
	else
		pins[CC1000_PIN_DCLK].clear();
	// Receiver
	if(rxtx == CC1000_RX){
		dio_prev = dio;
		// TODO: receive from air
		dio = LOW;
		dio == HIGH ? pins[CC1000_PIN_DIO].set() : pins[CC1000_PIN_DIO].clear();
		// RSSI
		pins[CC1000_PIN_RSSI].setAnalogValue(1.2);
	}
	// Transmitter
	else{
		// Data
		dio_prev = dio;
		dio = pins[CC1000_PIN_DIO].get();
		// RSSI
		pins[CC1000_PIN_RSSI].setAnalogValue(1.2);
		// TODO: send to air
	}
	// If both Tx & Rx power down return otherwise continue
	if(!tx_pd || !rx_pd)
		mcu->addClockEvent(&dclk_event);
}

/*
	ClockEvent
*/
CC1000::ClockEvent::ClockEvent()
{
	cycles = 1;
}

void
CC1000::ClockEvent::setDevice(void *dev)
{
	cc1000 = (CC1000 *)dev;
}

void
CC1000::ClockEvent::fired()
{
	cc1000->tick();
}

/*
	CallibrationEvent
*/
void
CC1000::CallibrationEvent::setTime(double cal_time)
{
	cycles = (cal_time * 7372800) / 1000;
}

void
CC1000::CallibrationEvent::setDevice(void *dev)
{
	cc1000 = (CC1000 *)dev;
}

void
CC1000::CallibrationEvent::fired()
{
	cc1000->calibrationCompleted();
}

/*
	DCLK Ticker
*/
CC1000::DClkEvent::DClkEvent()
{
	cycles = 416;
}

void
CC1000::DClkEvent::setDevice(void *dev)
{
	cc1000 = (CC1000 *)dev;
}

void
CC1000::DClkEvent::fired()
{
	cc1000->DClkTick();
}

