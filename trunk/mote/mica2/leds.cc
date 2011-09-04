/**
    leds.cc
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

#include "leds.hh"
#include <iostream>

Leds::Leds()
{
	leds[MICA2_LED_YELLOW] = Pin("LED_Y", true);
	leds[MICA2_LED_GREEN] = Pin("LED_G", true);
	leds[MICA2_LED_RED] = Pin("LED_R", true);
	leds_prev[MICA2_LED_YELLOW] = leds[MICA2_LED_YELLOW].get();
	leds_prev[MICA2_LED_GREEN] = leds[MICA2_LED_GREEN].get();
	leds_prev[MICA2_LED_RED] = leds[MICA2_LED_RED].get();

	clk_event.setDevice((void *)this);
}

Leds::~Leds()
{

}

void
Leds::setMCU(Mcu *mcu)
{
	this->mcu = mcu;
	mcu->addClockEvent(&clk_event);
}

void
Leds::refresh()
{
	if(leds_prev[MICA2_LED_YELLOW] != leds[MICA2_LED_YELLOW].get()){
		if(!leds[MICA2_LED_YELLOW].get())
			std::cerr << mcu->getCycles() << ": Yellow ON" << std::endl;
		else
			std::cerr << mcu->getCycles() << ": Yellow OFF" << std::endl;
	}
	if(leds_prev[MICA2_LED_GREEN] != leds[MICA2_LED_GREEN].get()){
		if(!leds[MICA2_LED_GREEN].get())
			std::cerr << mcu->getCycles() << ": Green ON" << std::endl;
		else
			std::cerr << mcu->getCycles() << ": Green OFF" << std::endl;
	}
	if(leds_prev[MICA2_LED_RED] != leds[MICA2_LED_RED].get()){
		if(!leds[MICA2_LED_RED].get())
			std::cerr << mcu->getCycles() << ": Red ON" << std::endl;
		else
			std::cerr << mcu->getCycles() << ": Red OFF" << std::endl;
	}
	leds_prev[MICA2_LED_YELLOW] = leds[MICA2_LED_YELLOW].get();
	leds_prev[MICA2_LED_GREEN] = leds[MICA2_LED_GREEN].get();
	leds_prev[MICA2_LED_RED] = leds[MICA2_LED_RED].get();

	// TODO: use IO events instead
	mcu->addClockEvent(&clk_event);
}

Pin*
Leds::getPins()
{
	return this->leds;
}

/*
	ClockEvent
*/
Leds::ClockEvent::ClockEvent()
{
	cycles = 1;
}

void
Leds::ClockEvent::setDevice(void *dev)
{
	leds = (Leds *)dev;
}

void
Leds::ClockEvent::fired()
{
	leds->refresh();
}
