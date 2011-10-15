/**
    pin.hh
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

#ifndef PIN_HH
#define PIN_HH

#include <vector>
#include <string>
#include <stdint.h>

class Pin{
private:
	bool pin;		// The binary value
	double val;		// The analog value
	std::string name;
	bool is_bind;
	std::vector<Pin *>bpin;
public:
	Pin();
	Pin(std::string);
	Pin(bool);
	Pin(std::string, bool);
	~Pin();
	// Binary
	void set();
	void clear();
	void toggle();
	bool get();
	// Analog
	void setAnalogValue(double);
	double getAnalogValue();
	void setName(std::string);
	std::string getName();
	static void bind(Pin&, Pin&);
};

#endif
