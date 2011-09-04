/**
    event.hh
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

#ifndef EVENT_HH
#define EVENT_HH

class Event{
friend class Clock;
protected:
	uint64_t cycles;
public:
	virtual void setDevice(void *) = 0;		// sadly C++ does not support pure virtual constructors :(
	virtual void fired() = 0;
};

#endif
