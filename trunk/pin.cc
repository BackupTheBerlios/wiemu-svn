/**
    pin.cc
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
#include "include/pin.hh"
#include<iostream>

Pin::Pin()
{
	this->pin = false;
	this->is_bind = false;
}

Pin::Pin(std::string s)
{
	this->pin = false;
	this->name = s;
	this->is_bind = false;
}

Pin::Pin(bool p)
{
	this->pin = p;
	this->is_bind = false;
}

Pin::Pin(std::string s, bool p)
{
	this->name = s;
	this->pin = p;
	this->is_bind = false;
}

Pin::~Pin(){}

void
Pin::set()
{
	this->pin = true;
	if(is_bind)
		this->bpin->pin = this->pin;
}

void
Pin::clear()
{
	this->pin = false;
	if(is_bind)
		this->bpin->pin = this->pin;
}

void
Pin::toggle()
{
	if(this->pin)
		clear();
	else
		set();
	if(is_bind)
		this->bpin->pin = this->pin;
}

bool
Pin::get()
{
	return this->pin;
}

void
Pin::setName(std::string s)
{
	this->name = s;
}

std::string
Pin::getName()
{
	return this->name;
}

void
Pin::bind(Pin &bp1, Pin &bp2)
{
	bp1.is_bind = true;
	bp2.is_bind = true;
	bp1.bpin = &bp2;
	bp2.bpin = &bp1;
	bp1.pin = bp2.pin;
}
