/**
    firmware.cc
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

#include "include/firmware.hh"
#include <iostream>
#include <string>
#include <fstream>
#include <cstdlib>		// for malloc()
#include <sys/stat.h>	// for stat()

Firmware::Firmware(){
#ifdef DEBUG
	std::cout << "Firmware()" << std::endl;
#endif
}

Firmware::~Firmware(){
#ifdef DEBUG
	std::cout << "~Firmware()" << std::endl;
#endif
}

Firmware::Firmware(std::string filename){
	setFileName(filename);
}

void
Firmware::setFileName(std::string filename){
	this->filename = filename;
}

std::string
Firmware::getFileName(void){
	return this->filename;
}

bool
Firmware::loadImage(){
	unsigned int size = getSize();
	this->firmware = (char *)malloc(size * sizeof(char));
	if(this->filename.empty())
		return false;
	std::ifstream is;
	is.open(this->filename.c_str(), std::ios::in | std::ios::binary);
	if(!is.is_open())
		return false;
	if(!is.read(this->firmware, size))
		return false;
	is.close();
	return true;
}

int
Firmware::getSize(){
	struct stat st;
	stat(this->filename.c_str(), &st);
	return st.st_size;
}

const char *Firmware::getFirm(void){
	return this->firmware;
}
