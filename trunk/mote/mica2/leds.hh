/**
    leds.hh
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

#ifndef LEDS_HH
#define LEDS_HH

#include <stdint.h>
#include "../../include/device.hh"
#include "../../include/pin.hh"

#define MICA2_NUM_LEDS		3
#define MICA2_LED_YELLOW	0
#define MICA2_LED_GREEN		1
#define MICA2_LED_RED		2

class Leds: public Device{
public:
	Leds();
	~Leds();
	void probe(uint64_t);
	Pin* getPins();
private:
	Pin leds[MICA2_NUM_LEDS];
	bool leds_prev[MICA2_NUM_LEDS];
};

#endif

