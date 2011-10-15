/**
    debugger.hh
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

#ifndef DEBUGGER_HH
#define DEBUGGER_HH

#include <stdint.h>
#include <fstream>

class Debugger{
private:
	bool enableDisassembler;
	std::string filename;
	std::ofstream file;
public:
	Debugger();
	void setFile(std::string);
	void enableDisassembly();
	void disableDisassembly();
	bool isDisassemblyEnabled();
	bool isFileOpened();
	std::ofstream& getFile();
};

#endif
