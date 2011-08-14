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
	registers[CC1000_REG_MAIN] = 0x00;		// Not initialized
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

	// Initialize the internal dummy pins
	pale = pins[CC1000_PIN_PALE].get();
	pclk = pins[CC1000_PIN_PCLK].get();
	pdata = pins[CC1000_PIN_PDATA].get();
	dio = pins[CC1000_PIN_DIO].get();
	rssi = pins[CC1000_PIN_RSSI].get();
	address = 0;
	w = false;
	data = 0;

	logf.open("cc1000.log");
}

CC1000::~CC1000()
{

}

void
CC1000::probe(uint64_t time)
{
	dump(time);
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
	// Data
	//dio_prev = dio;
	//dio = pins[CC1000_PIN_DIO].get();
	//dclk_prev = dclk;
	//dclk = pins[CC1000_PIN_DCLK].get();
	//rssi_prev = rssi;
	//rssi = pins[CC1000_PIN_RSSI].get();
	if(pale_prev == HIGH && pale == LOW){		// Address
#ifdef DEBUG
		std::cerr << time << ": Addressing" << std::endl;
#endif
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
#ifdef DEBUG
			std::cerr << time << ": Data (W)" << std::endl;
#endif
			callbacks.push(&CC1000::writeData);
		}else{
#ifdef DEBUG
			std::cerr << time << ": Data (R)" << std::endl;
#endif
			callbacks.push(&CC1000::readData);
		}
	}
}

void
CC1000::readAddress()
{
	if(pclk_prev == LOW && pclk == HIGH){
#ifdef DEBUG
		std::cerr << "(A) PCLK Toggled " << address_nbits << std::endl;
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
		std::cerr << "(W) PCLK Toggled " << w_nbits << std::endl;
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
		std::cerr << "(D) PCLK Toggled " << data_nbits << " with " << (int)pins[CC1000_PIN_PDATA].get() << std::endl;
#endif
		if(pins[CC1000_PIN_PDATA].get())
			data |= 1;
		++data_nbits;
		if(data_nbits == 8){
			registers[address] = data;
			print3();
			callbacks.pop();		// Clean myself!
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
			std::cerr << "(D) PCLK Toggled " << data_nbits << " with 1" << std::endl;
#endif
		}
		else{
			pins[CC1000_PIN_PDATA].clear();
#ifdef DEBUG
			std::cerr << "(D) PCLK Toggled " << data_nbits << " with 0" << std::endl;
#endif
		}
		++data_nbits;
		if(data_nbits == 8){
			print3();
			callbacks.pop();		// Clean myself!
		}
		data <<= 1;
	}
}

void
CC1000::print1()
{
#ifdef DEBUG
	std::cerr << "Address = " << std::hex << (int)address << std::dec << std::endl;
#endif
}

void
CC1000::print2()
{
#ifdef DEBUG
	std::cerr << "W = " << std::hex << (int)w << std::dec << std::endl;
#endif
}

void
CC1000::print3()
{
#ifdef DEBUG
	std::cerr << "Data = " << std::hex << (int)registers[address] << std::dec << std::endl;
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
	logf << "TEST5 = " << (int)registers[CC1000_REG_TEST6] << std::endl;
	logf << "TEST4 = " << (int)registers[CC1000_REG_TEST6] << std::endl;
	logf << "TEST3 = " << (int)registers[CC1000_REG_TEST6] << std::endl;
	logf << "TEST2 = " << (int)registers[CC1000_REG_TEST6] << std::endl;
	logf << "TEST1 = " << (int)registers[CC1000_REG_TEST6] << std::endl;
	logf << "TEST0 = " << (int)registers[CC1000_REG_TEST6] << std::endl;
	logf << std::endl << std::endl;
	logf.flush();
}

Pin*
CC1000::getPins()
{
	return this->pins;
}
