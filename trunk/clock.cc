/**
    clock.cc
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

#include "include/clock"
#include <iostream>

Clock::Clock()
{
	cycles = 0;
	freq = 0;
}

Clock::~Clock()
{

}

void
Clock::reset()
{
	cycles = 0;
}


void
Clock::addEvent(Event *e)
{
	events.push(std::make_pair(cycles + e->cycles, e));
}

void
Clock::updateEvents()
{
	if(events.empty())	// no events :)
		return;
	while(events.top().first <= cycles){
		events.top().second->fired();
		events.pop();
		if(events.empty())
			break;
	}
}

uint64_t
Clock::getFreq()
{
	return freq;
}

void
Clock::setFreq(uint64_t freq)
{
	this->freq = freq;
}

uint64_t
Clock::getCycles()
{
	return cycles;
}

void
Clock::setCycles(uint64_t cycles)
{
	this->cycles = cycles;
	updateEvents();
}

Clock&
Clock::operator++(int)
{
	setCycles(cycles + 1);
	return *this;
}

Clock&
Clock::operator--(int)
{
	setCycles(cycles - 1);
	return *this;
}

Clock&
Clock::operator+=(const uint64_t c)
{
	setCycles(cycles + c);
	return *this;
}

Clock&
Clock::operator+=(const Clock c)
{
	setCycles(cycles + c.cycles);
	return *this;
}

Clock&
Clock::operator=(const Clock c)
{
	setCycles(c.cycles);
	freq = c.freq;
	return *this;
}

Clock&
Clock::operator=(const uint64_t c)
{
	setCycles(c);
	return *this;
}

Clock&
Clock::operator+(const Clock c)
{
	Clock *me = new Clock();
	me->setFreq(freq);
	me->setCycles(cycles + c.cycles);
	return *me;
}

Clock&
Clock::operator+(const uint64_t c)
{
	Clock *me = new Clock();
	me->setFreq(freq);
	me->setCycles(cycles + c);
	return *me;
}

Clock&
Clock::operator-(const Clock c)
{
	Clock *me = new Clock();
	me->setFreq(freq);
	me->setCycles(cycles - c.cycles);
	return *me;
}

Clock&
Clock::operator-(const uint64_t c)
{
	Clock *me = new Clock();
	me->setFreq(freq);
	me->setCycles(cycles - c);
	return *me;
}

