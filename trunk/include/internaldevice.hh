/**
    internaldevice.hh
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

/**
 * Internal devices gets access to MCU's internals
**/

#ifndef INTERNALDEVICE_HH
#define INTERNALDEVICE_HH

#include <stdint.h>

#define	DEV_IO_READ	0x0
#define	DEV_IO_WRITE	0x1
#define	DEV_IO_CHANGED	0x2

class Mcu;

/*
 * Every internal device should be a friend of it's microcontroller class
 */
class InternalDevice{
private:

public:
	virtual void setMCU(Mcu *) = 0;
	virtual void probeIO(uint8_t, uint8_t) = 0;
};

#endif
