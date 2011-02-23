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
}

Leds::~Leds()
{

}

void
Leds::probe(uint64_t time)
{
	if(leds_prev[MICA2_LED_YELLOW] != leds[MICA2_LED_YELLOW].get()){
		if(!leds[MICA2_LED_YELLOW].get())
			std::cerr << time << ": Yellow ON" << std::endl;
		else
			std::cerr << time << ": Yellow OFF" << std::endl;
	}
	if(leds_prev[MICA2_LED_GREEN] != leds[MICA2_LED_GREEN].get()){
		if(!leds[MICA2_LED_GREEN].get())
			std::cerr << time << ": Green ON" << std::endl;
		else
			std::cerr << time << ": Green OFF" << std::endl;
	}
	if(leds_prev[MICA2_LED_RED] != leds[MICA2_LED_RED].get()){
		if(!leds[MICA2_LED_RED].get())
			std::cerr << time << ": Red ON" << std::endl;
		else
			std::cerr << time << ": Red OFF" << std::endl;
	}
	leds_prev[MICA2_LED_YELLOW] = leds[MICA2_LED_YELLOW].get();
	leds_prev[MICA2_LED_GREEN] = leds[MICA2_LED_GREEN].get();
	leds_prev[MICA2_LED_RED] = leds[MICA2_LED_RED].get();
}

Pin*
Leds::getPins()
{
	return this->leds;
}
