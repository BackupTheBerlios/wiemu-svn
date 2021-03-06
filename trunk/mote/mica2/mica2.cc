/**
    mica2.cc
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

#include "mica2.hh"

Mica2::Mica2()
{
	Pin::bind(leds.getPins()[MICA2_LED_YELLOW], mcu.getPins()[AVR_PIN_PA0]);
	Pin::bind(leds.getPins()[MICA2_LED_GREEN], mcu.getPins()[AVR_PIN_PA1]);
	Pin::bind(leds.getPins()[MICA2_LED_RED], mcu.getPins()[AVR_PIN_PA2]);
	leds.setMCU(&mcu);
	Pin::bind(cc1000.getPins()[CC1000_PIN_PALE], mcu.getPins()[AVR_PIN_PD4]);
	Pin::bind(cc1000.getPins()[CC1000_PIN_PCLK], mcu.getPins()[AVR_PIN_PD6]);
	Pin::bind(cc1000.getPins()[CC1000_PIN_PDATA], mcu.getPins()[AVR_PIN_PD7]);
	Pin::bind(cc1000.getPins()[CC1000_PIN_DIO], mcu.getPins()[AVR_PIN_PB3]);
	Pin::bind(cc1000.getPins()[CC1000_PIN_DIO], mcu.getPins()[AVR_PIN_PB2]);
	Pin::bind(cc1000.getPins()[CC1000_PIN_DCLK], mcu.getPins()[AVR_PIN_PB1]);
	Pin::bind(cc1000.getPins()[CC1000_PIN_RSSI], mcu.getPins()[AVR_PIN_PF0]);
	mcu.getPins()[AVR_PIN_PB0].clear();		// connected to the ground
	cc1000.setMCU(&mcu);
}

Mica2::~Mica2()
{

}

void
Mica2::load(std::string image){
	mcu.loadImage(image);
}

void
Mica2::run()
{
	mcu.run();
}

void
Mica2::step()
{
	mcu.step();
}

uint64_t
Mica2::getCycles()
{
	return mcu.getCycles();
}

void
Mica2::setDebugger(Debugger *debug)
{
	mcu.setDebugger(debug);
}

