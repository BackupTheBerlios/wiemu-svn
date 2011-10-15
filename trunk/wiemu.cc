/**
    wiemu.cc
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

#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include "include/node.hh"
#include "include/thread.hh"

void ver()
{
	std::cout << "WiEmu WSN Emulator" << std::endl;
	std::cout << "Copyrights (c) 2010-2011 Mohamed Aslan. All rights reserved." << std::endl;
#if defined(CXXVER) && defined(DATE)
	std::cout << "Compiled with: " << CXXVER <<  " on: " << DATE << std::endl;
#endif
}

