/**
    mcu.hh
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

#ifndef MCU_HH
#define MCU_HH

#include "internaldevice.hh"
#include "device.hh"
#include "clock.hh"
#include "debugger.hh"

class Mcu{
private:
	virtual void addInternalDevice(InternalDevice *) = 0;
public:
	virtual void step(void) = 0;
	virtual void run(void) = 0;
	virtual uint64_t getCycles(void) = 0;
	virtual Clock getClock(void) = 0;
	virtual void addClockEvent(Event *) = 0;
	virtual void setDebugger(Debugger *) = 0;
};

#endif
