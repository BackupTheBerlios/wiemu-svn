/**
    mica2.hh
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

#ifndef MICA2_HH
#define MICA2_HH

#include "../../arch/avr/avr.hh"
#include "../../include/clock"
#include "../../include/mote"
#include "../../include/debugger.hh"
#include "leds.hh"
#include "cc1000.hh"

class Mica2: public Mote{
public:
	Mica2();
	~Mica2();
	void load(std::string);
	void run();
	void step();
	uint64_t getCycles();
	void display();
	void setDebugger(Debugger *);
private:
	Node *node;
	Avr mcu;
	Leds leds;
	CC1000 cc1000;
};

#endif

