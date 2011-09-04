/**
    clock.hh
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

#ifndef CLOCK_HH
#define CLOCK_HH

#include <stdint.h>	// cstdint requires c++0x
#include <queue>
#include <utility>
#include "event"

class Event;

class PairGreaterComparator{
public:
	bool operator()(std::pair<uint64_t, Event *> &p1, std::pair<uint64_t, Event *> &p2){
		return p1.first > p2.first;
	}
};

class Clock{
private:
	uint64_t freq;
	uint64_t cycles;
	std::priority_queue<std::pair<uint64_t, Event*>, std::vector<std::pair<uint64_t, Event*> >, PairGreaterComparator> events;
	void updateEvents();
public:
	Clock();
	~Clock();
	void reset();
	void addEvent(Event*);
	uint64_t getFreq();
	void setFreq(uint64_t);
	uint64_t getCycles();
	void setCycles(uint64_t);
	Clock& operator=(const Clock);
	Clock& operator=(const uint64_t);
	Clock& operator+(const Clock);
	Clock& operator+(const uint64_t);
	Clock& operator-(const Clock);
	Clock& operator-(const uint64_t);
	Clock& operator++(int);
	Clock& operator--(int);
	Clock& operator+=(const uint64_t);
	Clock& operator+=(const Clock);
};

#endif
