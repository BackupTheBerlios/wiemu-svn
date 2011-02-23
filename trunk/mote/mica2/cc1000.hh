/**
    cc1000.hh
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

#ifndef CC1000_HH
#define CC1000_HH

#define	MICA2_CC1000_NUM_PINS	28

#include <stdint.h>
#include "../../include/device.hh"
#include "../../include/pin.hh"

class CC1000: public Device{
public:
	CC1000();
	~CC1000();
	void probe(uint64_t);
	Pin* getPins();
private:
	Pin pins[MICA2_CC1000_NUM_PINS];
};

#endif

